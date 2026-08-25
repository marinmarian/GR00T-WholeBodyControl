// main_web_ir.cpp
// G1 head-camera (RealSense D430i IR) sender for XRoboToolkit "Remote Vision".
//
// Ported from main_zed_tcp.cpp: keeps the OPEN_CAMERA / CLOSE_CAMERA control
// protocol verbatim (the headset connects to our listen port, sends its own
// callback ip:port, and we open a return TCP connection and stream H.264 to it),
// but replaces the ZED SDK capture with a direct v4l2 GStreamer pipeline reading
// the RealSense IR node (/dev/video2, GRAY8 640x480) and HW-encoding to H.264.
// The "only ZED cameras" rejection is removed so any Remote-Vision source works.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <glib-unix.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <thread>
#include <vector>

#include <sys/time.h>
#include <zmq.h>

#include "network_helper.hpp"

// ---------------------------------------------------------------------------
// Network protocol structures (verbatim from main_zed_tcp.cpp)
// ---------------------------------------------------------------------------
struct CameraRequestData {
  int width;
  int height;
  int fps;
  int bitrate;
  int enableMvHevc;
  int renderMode;
  int port;
  std::string camera;
  std::string ip;

  CameraRequestData()
      : width(0), height(0), fps(0), bitrate(0), enableMvHevc(0), renderMode(0),
        port(0) {}
};

struct NetworkDataProtocol {
  std::string command;
  int length;
  std::vector<uint8_t> data;

  NetworkDataProtocol() : length(0) {}
  NetworkDataProtocol(const std::string &cmd, const std::vector<uint8_t> &d)
      : command(cmd), data(d), length(d.size()) {}
};

class CameraRequestDeserializer {
public:
  static CameraRequestData deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 10) {
      throw std::invalid_argument("Data is too small for valid camera request");
    }
    size_t offset = 0;
    if (data[offset] != 0xCA || data[offset + 1] != 0xFE) {
      throw std::invalid_argument("Invalid magic bytes");
    }
    offset += 2;
    uint8_t version = data[offset++];
    if (version != 1) {
      throw std::invalid_argument("Unsupported protocol version");
    }
    CameraRequestData result;
    if (offset + 28 > data.size()) {
      throw std::invalid_argument("Data too small for integer fields");
    }
    result.width = readInt32(data, offset);
    result.height = readInt32(data, offset + 4);
    result.fps = readInt32(data, offset + 8);
    result.bitrate = readInt32(data, offset + 12);
    result.enableMvHevc = readInt32(data, offset + 16);
    result.renderMode = readInt32(data, offset + 20);
    result.port = readInt32(data, offset + 24);
    offset += 28;
    result.camera = readCompactString(data, offset);
    result.ip = readCompactString(data, offset);
    return result;
  }

private:
  static int32_t readInt32(const std::vector<uint8_t> &data, size_t offset) {
    if (offset + 4 > data.size()) {
      throw std::out_of_range("Not enough data to read int32");
    }
    return static_cast<int32_t>((data[offset]) | (data[offset + 1] << 8) |
                                (data[offset + 2] << 16) |
                                (data[offset + 3] << 24));
  }
  static std::string readCompactString(const std::vector<uint8_t> &data,
                                       size_t &offset) {
    if (offset >= data.size()) {
      throw std::out_of_range("Not enough data to read string length");
    }
    uint8_t length = data[offset++];
    if (length == 0)
      return std::string();
    if (offset + length > data.size()) {
      throw std::out_of_range("Not enough data to read string content");
    }
    std::string result(reinterpret_cast<const char *>(&data[offset]), length);
    offset += length;
    return result;
  }
};

class NetworkDataProtocolDeserializer {
public:
  static NetworkDataProtocol deserialize(const std::vector<uint8_t> &buffer) {
    if (buffer.size() < 8) {
      throw std::invalid_argument("Buffer too small for valid protocol data");
    }
    size_t offset = 0;
    int32_t commandLength = readInt32(buffer, offset);
    offset += 4;
    if (commandLength < 0 || offset + commandLength > buffer.size()) {
      throw std::invalid_argument("Invalid command length");
    }
    std::string command;
    if (commandLength > 0) {
      command = std::string(reinterpret_cast<const char *>(&buffer[offset]),
                            commandLength);
      size_t nullPos = command.find('\0');
      if (nullPos != std::string::npos)
        command = command.substr(0, nullPos);
    }
    offset += commandLength;
    if (offset + 4 > buffer.size()) {
      throw std::invalid_argument("Buffer too small for data length");
    }
    int32_t dataLength = readInt32(buffer, offset);
    offset += 4;
    if (dataLength < 0 || offset + dataLength > buffer.size()) {
      throw std::invalid_argument("Invalid data length");
    }
    std::vector<uint8_t> data;
    if (dataLength > 0) {
      data.assign(buffer.begin() + offset, buffer.begin() + offset + dataLength);
    }
    return NetworkDataProtocol(command, data);
  }

private:
  static int32_t readInt32(const std::vector<uint8_t> &data, size_t offset) {
    if (offset + 4 > data.size()) {
      throw std::out_of_range("Not enough data to read int32");
    }
    return static_cast<int32_t>((data[offset]) | (data[offset + 1] << 8) |
                                (data[offset + 2] << 16) |
                                (data[offset + 3] << 24));
  }
};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
CameraRequestData current_camera_config;

std::atomic<bool> stop_requested{false};
std::atomic<bool> streaming_active{false};
std::atomic<bool> encoding_enabled{false};
std::atomic<bool> send_enabled{false};

std::unique_ptr<std::thread> listen_thread;
std::unique_ptr<std::thread> streaming_thread;
std::mutex config_mutex;
std::condition_variable streaming_cv;
std::mutex streaming_mutex;

std::unique_ptr<TCPClient> sender_ptr;
// Newer headset clients fire OPEN_CAMERA twice and appear to expect a video
// connection per request; mirror packets onto an optional second connection.
std::unique_ptr<TCPClient> sender2_ptr;
std::mutex sender2_mutex;
std::unique_ptr<TCPServer> server_ptr;
std::string send_to_server = "";
int send_to_port = 0;

// Capture configuration (CLI-overridable; defaults = D430i left-IR head cam)
std::string g_device = "/dev/video2";
std::string g_pixfmt = "GRAY8";  // GStreamer caps format: GRAY8, YUY2, UYVY, ...
int g_cap_width = 640;
int g_cap_height = 480;
int g_cap_fps = 30;
int g_max_bitrate = 8000000;  // cap the headset's requested bitrate (it asks 20 Mbps,
                              // sized for wired; over WiFi that backlogs TCP -> lag)
std::string g_video_via = "";  // connect video to this IP instead of the headset's
                               // self-reported one (relay host, firewall bypass)
int g_flip = 0;   // rotation: 0=none 1=90ccw 2=180 3=90cw (CPU videoflip)
bool g_letterbox = true;  // preserve aspect on the requested canvas (--fit stretch to disable)

// --zmq-pub PORT: tee the capture into 20 Hz 640x480 JPEGs published over the
// decoupled_wbc composed-camera ZMQ protocol (msgpack, image key "ego_view"),
// so run_g1_data_exporter can record episodes from the same camera that feeds
// the headset. Frames flow only while a Remote Vision session is streaming.
int g_zmq_pub_port = 0;
void *g_zmq_ctx = nullptr;
void *g_zmq_pub = nullptr;
const int DATA_W = 640, DATA_H = 480;  // dataset ego_view shape (RS_VIEW_*)

// Optional second camera -> side-by-side composite (primary left, secondary right)
std::string g_device2 = "";
std::string g_pixfmt2 = "GRAY8";
int g_cap2_width = 640;
int g_cap2_height = 480;
int g_cap2_fps = 15;
int g_flip2 = 0;

template <typename T, typename... Args>
std::unique_ptr<T> make_unique_helper(Args &&...args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

bool initialize_sender() {
  // The headset can take tens of seconds to arm its video listener after
  // sending OPEN_CAMERA (seen after app restarts); keep knocking for 60 s.
  int retry = 60;
  while (retry > 0 && !sender_ptr && !stop_requested.load()) {
    try {
      sender_ptr =
          std::unique_ptr<TCPClient>(new TCPClient(send_to_server, send_to_port));
      std::cout << "Attempting to connect to " << send_to_server << ":"
                << send_to_port << std::endl;
      sender_ptr->connect();
      return true;
    } catch (const TCPException &e) {
      std::cerr << "Failed to connect to server: " << e.what() << std::endl;
      sender_ptr = nullptr;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    retry--;
  }
  return false;
}

// Forward declarations
void handleOpenCamera(const std::vector<uint8_t> &data);
void handleCloseCamera(const std::vector<uint8_t> &data);
void startStreamingThread();
void stopStreamingThread();
void streamingThreadFunction();
void listenThreadFunction(const std::string &listen_address);

// ---------------------------------------------------------------------------
// H.264 appsink -> TCP: frame each sample as [4-byte BE length][H.264] and send
// (verbatim framing from main_zed_tcp.cpp)
// ---------------------------------------------------------------------------
GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data) {
  (void)user_data;
  GstSample *sample = gst_app_sink_pull_sample(sink);
  if (!sample)
    return GST_FLOW_ERROR;

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    const uint8_t *data = map.data;
    gsize size = map.size;
    if (send_enabled.load() && sender_ptr && sender_ptr->isConnected() && data &&
        size > 0) {
      try {
        std::vector<uint8_t> packet(4 + size);
        packet[0] = (size >> 24) & 0xFF;
        packet[1] = (size >> 16) & 0xFF;
        packet[2] = (size >> 8) & 0xFF;
        packet[3] = (size) & 0xFF;
        std::copy(data, data + size, packet.begin() + 4);
        sender_ptr->sendData(packet);
        {
          std::lock_guard<std::mutex> lk(sender2_mutex);
          if (sender2_ptr && sender2_ptr->isConnected()) {
            try {
              sender2_ptr->sendData(packet);
            } catch (const TCPException &e2) {
              std::cerr << "second stream dropped: " << e2.what() << std::endl;
              sender2_ptr = nullptr;
            }
          }
        }
      } catch (const TCPException &e) {
        std::cerr << "TCP error in on_new_sample: " << e.what() << std::endl;
        streaming_active.store(false);
      } catch (const std::exception &e) {
        std::cerr << "Unexpected error in on_new_sample: " << e.what()
                  << std::endl;
        streaming_active.store(false);
      }
    }
    gst_buffer_unmap(buffer, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

// Latest head-camera JPEG (from the udp: branch), published alongside ego_view.
std::mutex g_head_mutex;
std::vector<uint8_t> g_head_jpeg;
double g_head_ts = 0.0;

// msgpack: {"timestamps":{name:<f64>...},"images":{name:<bin jpeg>...}}
static void publish_data_frame(const uint8_t *jpeg, size_t len) {
  if (!g_zmq_pub)
    return;
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  double ts = tv.tv_sec + tv.tv_usec / 1e6;

  std::vector<uint8_t> head;
  double head_ts = 0.0;
  {
    std::lock_guard<std::mutex> lk(g_head_mutex);
    head = g_head_jpeg;
    head_ts = g_head_ts;
  }
  int n_imgs = head.empty() ? 1 : 2;

  std::vector<uint8_t> m;
  m.reserve(len + head.size() + 96);
  auto put_str = [&](const char *str) {
    size_t n = strlen(str);
    m.push_back(0xa0 | (uint8_t)n);  // fixstr (all our keys are < 32 chars)
    m.insert(m.end(), str, str + n);
  };
  auto put_f64 = [&](double v) {
    m.push_back(0xcb);
    uint64_t bits;
    memcpy(&bits, &v, 8);
    for (int i = 7; i >= 0; --i)
      m.push_back((bits >> (i * 8)) & 0xFF);
  };
  auto put_bin = [&](const uint8_t *d, size_t n) {
    m.push_back(0xc6);  // bin32
    m.push_back((n >> 24) & 0xFF);
    m.push_back((n >> 16) & 0xFF);
    m.push_back((n >> 8) & 0xFF);
    m.push_back(n & 0xFF);
    m.insert(m.end(), d, d + n);
  };
  m.push_back(0x82);  // map(2)
  put_str("timestamps");
  m.push_back(0x80 | (uint8_t)n_imgs);
  put_str("ego_view");
  put_f64(ts);
  if (n_imgs == 2) {
    put_str("head_view");
    put_f64(head_ts);
  }
  put_str("images");
  m.push_back(0x80 | (uint8_t)n_imgs);
  put_str("ego_view");
  put_bin(jpeg, len);
  if (n_imgs == 2) {
    put_str("head_view");
    put_bin(head.data(), head.size());
  }
  zmq_send(g_zmq_pub, m.data(), m.size(), ZMQ_DONTWAIT);
}

// Standalone head-recorder: consumes a SECOND copy of the RTP stream (the g1
// push uses multiudpsink to port and port+1) so recording can never
// backpressure the live video pipeline (an in-branch tee stalled it).
GstElement *g_head_pipeline = nullptr;

GstFlowReturn on_new_head_sample(GstAppSink *sink, gpointer user_data);

static void start_head_recorder(int rtp_port) {
  std::string desc =
      "udpsrc port=" + std::to_string(rtp_port) +
      " caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26 ! "
      "rtpjitterbuffer latency=80 ! rtpjpegdepay ! "
      "appsink name=headsink emit-signals=true sync=false max-buffers=2 drop=true";
  GError *err = nullptr;
  g_head_pipeline = gst_parse_launch(desc.c_str(), &err);
  if (!g_head_pipeline || err) {
    std::cerr << "head recorder pipeline failed: "
              << (err ? err->message : "unknown") << std::endl;
    if (err)
      g_clear_error(&err);
    return;
  }
  GstElement *sink = gst_bin_get_by_name(GST_BIN(g_head_pipeline), "headsink");
  g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_head_sample), nullptr);
  gst_object_unref(sink);
  gst_element_set_state(g_head_pipeline, GST_STATE_PLAYING);
  std::cout << "Head recorder listening on udp:" << rtp_port
            << " (head_view in ZMQ frames)" << std::endl;
}

GstFlowReturn on_new_head_sample(GstAppSink *sink, gpointer user_data) {
  (void)user_data;
  GstSample *sample = gst_app_sink_pull_sample(sink);
  if (!sample)
    return GST_FLOW_ERROR;
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    std::lock_guard<std::mutex> lk(g_head_mutex);
    if (g_head_jpeg.empty())
      std::cout << "[head] first frame received (" << map.size << " bytes)"
                << std::endl;
    g_head_jpeg.assign(map.data, map.data + map.size);
    g_head_ts = tv.tv_sec + tv.tv_usec / 1e6;
    gst_buffer_unmap(buffer, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

GstFlowReturn on_new_data_sample(GstAppSink *sink, gpointer user_data) {
  (void)user_data;
  GstSample *sample = gst_app_sink_pull_sample(sink);
  if (!sample)
    return GST_FLOW_ERROR;
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    publish_data_frame(map.data, map.size);
    gst_buffer_unmap(buffer, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Control command dispatch (verbatim from main_zed_tcp.cpp)
// ---------------------------------------------------------------------------
void onDataCallback(const std::string &command) {
  std::vector<uint8_t> binaryData(command.begin(), command.end());
  if (binaryData.size() < 4) {
    std::cerr << "Data too small to contain length header" << std::endl;
    return;
  }
  uint32_t bodyLength = (static_cast<uint32_t>(binaryData[0]) << 24) |
                        (static_cast<uint32_t>(binaryData[1]) << 16) |
                        (static_cast<uint32_t>(binaryData[2]) << 8) |
                        static_cast<uint32_t>(binaryData[3]);
  if (4 + bodyLength > binaryData.size()) {
    std::cerr << "Data too small for declared body length. Expected: "
              << (4 + bodyLength) << ", got: " << binaryData.size() << std::endl;
    return;
  }
  std::vector<uint8_t> protocolData(binaryData.begin() + 4,
                                    binaryData.begin() + 4 + bodyLength);
  try {
    NetworkDataProtocol protocol =
        NetworkDataProtocolDeserializer::deserialize(protocolData);
    std::cout << "Received protocol command: '" << protocol.command << "'"
              << std::endl;
    if (protocol.command == "OPEN_CAMERA") {
      handleOpenCamera(protocol.data);
    } else if (protocol.command == "CLOSE_CAMERA") {
      handleCloseCamera(protocol.data);
    } else {
      std::cout << "Unknown protocol command: " << protocol.command << std::endl;
    }
  } catch (const std::exception &e) {
    std::cout << "Failed to parse as NetworkDataProtocol: " << e.what()
              << std::endl;
  }
}

void onDisconnectCallback() {
  std::cout << "Client disconnected, stopping streaming" << std::endl;
  stopStreamingThread();
}

void listenThreadFunction(const std::string &listen_address) {
  std::cout << "Listen thread started on " << listen_address << std::endl;
  while (!stop_requested.load()) {
    try {
      server_ptr = make_unique_helper<TCPServer>(listen_address);
      server_ptr->setDataCallback(onDataCallback);
      server_ptr->setDisconnectCallback(onDisconnectCallback);
      server_ptr->start();
      std::cout << "TCPServer is listening on " << listen_address << std::endl;
      while (!stop_requested.load() && server_ptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (server_ptr) {
        server_ptr->stop();
        server_ptr = nullptr;
      }
      if (!stop_requested.load()) {
        std::cout << "Waiting for new connection..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    } catch (const std::exception &e) {
      std::cerr << "Listen thread error: " << e.what() << std::endl;
      if (!stop_requested.load())
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
  std::cout << "Listen thread stopped" << std::endl;
}

void handle_sigint(int) {
  std::cout << "\nSIGINT received. Stopping all threads..." << std::endl;
  stop_requested.store(true);
  stopStreamingThread();
  if (server_ptr) {
    server_ptr->stop();
    server_ptr = nullptr;
  }
  streaming_cv.notify_all();
}

// The 2026 XRoboToolkit client will not start rendering until it receives an
// OPEN_CAMERA_ACK on the control connection carrying the audio/video session
// config (schema g1_wuji_audio_ports_v2). Audio is disabled (ports 0); the
// request id only needs to be a 16-128 char token.
static void send_open_camera_ack() {
  if (!server_ptr)
    return;
  std::string json =
      "{\"schema\":\"g1_wuji_audio_ports_v2\","
      "\"audio_request_id\":\"orinvideosender-0000-video-only-ack\","
      "\"audio_stream_port\":0,"
      "\"microphone_upload_port\":0,"
      "\"video_projection\":\"flat\","
      "\"video_stereo_layout\":\"mono\"}";
  const std::string cmd = "OPEN_CAMERA_ACK";
  std::vector<uint8_t> body;
  auto put_le32 = [&](uint32_t v) {
    body.push_back(v & 0xFF);
    body.push_back((v >> 8) & 0xFF);
    body.push_back((v >> 16) & 0xFF);
    body.push_back((v >> 24) & 0xFF);
  };
  put_le32((uint32_t)cmd.size());
  body.insert(body.end(), cmd.begin(), cmd.end());
  put_le32((uint32_t)json.size());
  body.insert(body.end(), json.begin(), json.end());
  std::vector<uint8_t> frame;
  frame.push_back((body.size() >> 24) & 0xFF);
  frame.push_back((body.size() >> 16) & 0xFF);
  frame.push_back((body.size() >> 8) & 0xFF);
  frame.push_back(body.size() & 0xFF);
  frame.insert(frame.end(), body.begin(), body.end());
  if (server_ptr->replyToClient(frame))
    std::cout << "Sent OPEN_CAMERA_ACK" << std::endl;
  else
    std::cerr << "Failed to send OPEN_CAMERA_ACK" << std::endl;
}

void handleOpenCamera(const std::vector<uint8_t> &data) {
  std::cout << "Handling OPEN_CAMERA command" << std::endl;
  try {
    CameraRequestData cameraConfig =
        CameraRequestDeserializer::deserialize(data);
    std::cout << "Camera config - Width: " << cameraConfig.width
              << ", Height: " << cameraConfig.height
              << ", FPS: " << cameraConfig.fps
              << ", Bitrate: " << cameraConfig.bitrate
              << ", IP: " << cameraConfig.ip << ", Port: " << cameraConfig.port
              << ", type: " << cameraConfig.camera << std::endl;
    // NOTE: unlike main_zed_tcp.cpp we accept ANY camera type (no ZED rejection)
    {
      std::lock_guard<std::mutex> lock(config_mutex);
      current_camera_config = cameraConfig;
    }
    send_to_server = g_video_via.empty() ? cameraConfig.ip : g_video_via;
    send_to_port = cameraConfig.port;
    if (!g_video_via.empty())
      std::cout << "Video routed via relay " << g_video_via << std::endl;
    // Newer XRoboToolkit clients fire OPEN_CAMERA twice per panel-open; the
    // second accept is refused by the headset, so restarting the live stream
    // for a duplicate kills a working session. Ignore exact duplicates while
    // the stream is healthy.
    if (streaming_active.load() && sender_ptr && sender_ptr->isConnected() &&
        cameraConfig.ip == send_to_server && cameraConfig.port == send_to_port) {
      std::lock_guard<std::mutex> lk(sender2_mutex);
      if (sender2_ptr && sender2_ptr->isConnected()) {
        std::cout << "Duplicate OPEN_CAMERA - both streams already up" << std::endl;
        send_open_camera_ack();
        return;
      }
      std::cout << "Duplicate OPEN_CAMERA - trying a second video connection" << std::endl;
      for (int i = 0; i < 5; ++i) {
        try {
          auto c = std::unique_ptr<TCPClient>(
              new TCPClient(send_to_server, send_to_port));
          c->connect();
          sender2_ptr = std::move(c);
          std::cout << "SECOND video connection ESTABLISHED - mirroring stream"
                    << std::endl;
          send_open_camera_ack();
          return;
        } catch (const TCPException &e) {
          std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
      }
      std::cout << "second connection refused - keeping single stream" << std::endl;
      return;
    }
    std::cout << "Updated sender target to " << send_to_server << ":"
              << send_to_port << std::endl;
    send_open_camera_ack();
    startStreamingThread();
  } catch (const std::exception &e) {
    std::cerr << "Failed to parse camera config: " << e.what() << std::endl;
    if (!send_to_server.empty() && send_to_port > 0) {
      startStreamingThread();
    } else {
      std::cerr << "No valid server configuration available, cannot start "
                   "streaming"
                << std::endl;
    }
  }
}

void handleCloseCamera(const std::vector<uint8_t> &data) {
  (void)data;
  std::cout << "Handling CLOSE_CAMERA command" << std::endl;
  stopStreamingThread();
}

void startStreamingThread() {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  if (streaming_thread && streaming_thread->joinable()) {
    if (streaming_active.load()) {
      // Genuinely still streaming: tear the old session down so a reconnecting
      // headset (fresh OPEN_CAMERA) takes over instead of being refused.
      std::cout << "Streaming thread running - restarting for new client" << std::endl;
      streaming_active.store(false);
    } else {
      // Thread ended on its own (e.g. dead video socket) but was never joined.
      std::cout << "Reaping finished streaming thread" << std::endl;
    }
    if (sender_ptr && sender_ptr->isConnected())
      sender_ptr->disconnect();
    sender_ptr = nullptr;
    streaming_thread->join();
    streaming_thread = nullptr;
  }
  streaming_active.store(true);
  streaming_thread = make_unique_helper<std::thread>(streamingThreadFunction);
  std::cout << "Started streaming thread" << std::endl;
}

void stopStreamingThread() {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  streaming_active.store(false);
  encoding_enabled.store(false);
  send_enabled.store(false);
  if (sender_ptr && sender_ptr->isConnected())
    sender_ptr->disconnect();
  sender_ptr = nullptr;
  {
    std::lock_guard<std::mutex> lk(sender2_mutex);
    if (sender2_ptr && sender2_ptr->isConnected())
      sender2_ptr->disconnect();
    sender2_ptr = nullptr;
  }
  if (streaming_thread && streaming_thread->joinable()) {
    streaming_cv.notify_all();
    streaming_thread->join();
    streaming_thread = nullptr;
    std::cout << "Stopped streaming thread" << std::endl;
  }
}

// ---------------------------------------------------------------------------
// Streaming thread: direct v4l2 IR capture -> HW H.264 -> appsink -> TCP.
// (Replaces the ZED-SDK + appsrc capture in main_zed_tcp.cpp.)
// ---------------------------------------------------------------------------
void streamingThreadFunction() {
  std::cout << "Streaming thread started" << std::endl;
  try {
    if (!initialize_sender()) {
      std::cerr << "Failed to initialize sender, streaming thread stopping"
                << std::endl;
      return;
    }
    encoding_enabled.store(true);
    send_enabled.store(true);

    CameraRequestData config;
    {
      std::lock_guard<std::mutex> lock(config_mutex);
      config = current_camera_config;
    }
    // Scale the captured feed to whatever resolution the headset requested
    // (falls back to native size); honor requested bitrate if provided.
    int outW = config.width > 0 ? config.width : g_cap_width;
    int outH = config.height > 0 ? config.height : g_cap_height;
    int bitrate = config.bitrate > 0 ? config.bitrate : 4000000;
    bitrate = std::min(bitrate, g_max_bitrate);

    // Effective source dims after rotation (90-degree flips swap W/H)
    int effW = (g_flip == 1 || g_flip == 3) ? g_cap_height : g_cap_width;
    int effH = (g_flip == 1 || g_flip == 3) ? g_cap_width : g_cap_height;
    static const char *kFlip[] = {"", "counterclockwise", "rotate-180", "clockwise"};

    std::string encode_tail =
        "videoconvert ! video/x-raw,format=NV12 ! "
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=" +
        std::to_string(outW) + ",height=" + std::to_string(outH) +
        " ! "
        "nvv4l2h264enc maxperf-enable=1 insert-sps-pps=true idrinterval=15 "
        "bitrate=" +
        std::to_string(bitrate) +
        " ! "
        "h264parse ! appsink name=mysink emit-signals=true sync=false";

    // Optional data tee: 20 Hz letterboxed 640x480 JPEGs -> ZMQ (episode recording)
    std::string data_branch;
    if (g_zmq_pub_port > 0) {
      double dsc = std::min((double)DATA_W / effW, (double)DATA_H / effH);
      int dw = ((int)(effW * dsc) / 2) * 2, dh = ((int)(effH * dsc) / 2) * 2;
      int dl = (DATA_W - dw) / 2, dr = DATA_W - dw - dl;
      int dt = (DATA_H - dh) / 2, db = DATA_H - dh - dt;
      data_branch =
          "tee name=datatee ! queue leaky=downstream max-size-buffers=2 ! "
          "videoscale ! video/x-raw,width=" + std::to_string(dw) +
          ",height=" + std::to_string(dh) + " ! "
          "videobox fill=black left=-" + std::to_string(dl) +
          " right=-" + std::to_string(dr) +
          " top=-" + std::to_string(dt) +
          " bottom=-" + std::to_string(db) + " ! "
          "jpegenc quality=85 ! appsink name=datasink emit-signals=true sync=false "
          "datatee. ! queue ! ";
    }

    std::string pipeline_str;
    if (!g_device2.empty()) {
      // Dual-camera composite: primary letterboxed into the LEFT half of the
      // canvas, secondary into the RIGHT half (compositor sink pads scale).
      int eff2W = (g_flip2 == 1 || g_flip2 == 3) ? g_cap2_height : g_cap2_width;
      int eff2H = (g_flip2 == 1 || g_flip2 == 3) ? g_cap2_width : g_cap2_height;
      int halfW = outW / 2;
      auto fit = [&](int ew, int eh, int xoff, int &w, int &h, int &x, int &y) {
        double sc = std::min((double)halfW / ew, (double)outH / eh);
        w = ((int)(ew * sc) / 2) * 2;
        h = ((int)(eh * sc) / 2) * 2;
        x = xoff + (halfW - w) / 2;
        y = (outH - h) / 2;
      };
      int w0, h0, x0, y0, w1, h1, x1, y1;
      fit(effW, effH, 0, w0, h0, x0, y0);
      fit(eff2W, eff2H, halfW, w1, h1, x1, y1);

      std::string flip1 = (g_flip >= 1 && g_flip <= 3)
          ? std::string("videoflip method=") + kFlip[g_flip] + " ! " : "";
      std::string flip2 = (g_flip2 >= 1 && g_flip2 <= 3)
          ? std::string("videoflip method=") + kFlip[g_flip2] + " ! " : "";

      // Second source may be a network stream instead of a local V4L2 node:
      // --second-device udp:PORT expects RTP/MJPEG (rtpjpegpay) on that port,
      // e.g. pushed from another host with:
      //   gst-launch-1.0 v4l2src ! ... ! jpegenc ! rtpjpegpay ! udpsink host=<this> port=PORT
      std::string src2;
      if (g_device2.rfind("udp:", 0) == 0) {
        std::string port2 = g_device2.substr(4);
        src2 = "udpsrc port=" + port2 +
               " caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26 ! "
               "rtpjitterbuffer latency=80 ! "
               "rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,format=I420 ! "
               "queue leaky=downstream max-size-buffers=3 ! ";
      } else {
        src2 = "v4l2src device=" + g_device2 + " ! "
               "video/x-raw,format=" + g_pixfmt2 +
               ",width=" + std::to_string(g_cap2_width) +
               ",height=" + std::to_string(g_cap2_height) +
               ",framerate=" + std::to_string(g_cap2_fps) + "/1 ! "
               "videoconvert ! video/x-raw,format=I420 ! ";
      }

      pipeline_str =
          // latency + min-upstream-latency make the mixer LIVE: it emits on a
          // deadline with whichever inputs arrived, so a stalled/silent branch
          // (e.g. the udp: source) blanks its half instead of freezing both.
          "compositor name=comp background=black latency=150000000 "
          "min-upstream-latency=150000000 "
          "sink_0::xpos=" + std::to_string(x0) + " sink_0::ypos=" + std::to_string(y0) +
          " sink_0::width=" + std::to_string(w0) + " sink_0::height=" + std::to_string(h0) +
          " sink_1::xpos=" + std::to_string(x1) + " sink_1::ypos=" + std::to_string(y1) +
          " sink_1::width=" + std::to_string(w1) + " sink_1::height=" + std::to_string(h1) +
          " ! video/x-raw,format=I420,width=" + std::to_string(outW) +
          ",height=" + std::to_string(outH) + " ! " + encode_tail +
          "  v4l2src device=" + g_device + " ! "
          "video/x-raw,format=" + g_pixfmt +
          ",width=" + std::to_string(g_cap_width) +
          ",height=" + std::to_string(g_cap_height) +
          ",framerate=" + std::to_string(g_cap_fps) + "/1 ! "
          "videoconvert ! video/x-raw,format=I420 ! " + flip1 + data_branch + "comp.sink_0"
          "  " + src2 + flip2 + "comp.sink_1";
    } else {
      std::string mid;
      if (g_flip >= 1 && g_flip <= 3)
        mid += std::string("videoflip method=") + kFlip[g_flip] + " ! ";
      if (g_letterbox && (outW * effH != outH * effW)) {
        // Scale to fit inside outW x outH, pad the rest with black bars.
        double sc = std::min((double)outW / effW, (double)outH / effH);
        int sw = ((int)(effW * sc) / 2) * 2, sh = ((int)(effH * sc) / 2) * 2;
        int padL = (outW - sw) / 2, padR = outW - sw - padL;
        int padT = (outH - sh) / 2, padB = outH - sh - padT;
        mid += "videoscale ! video/x-raw,width=" + std::to_string(sw) +
               ",height=" + std::to_string(sh) + " ! "
               "videobox fill=black left=-" + std::to_string(padL) +
               " right=-" + std::to_string(padR) +
               " top=-" + std::to_string(padT) +
               " bottom=-" + std::to_string(padB) + " ! ";
      }
      pipeline_str =
          "v4l2src device=" + g_device + " ! "
          "video/x-raw,format=" + g_pixfmt +
          ",width=" + std::to_string(g_cap_width) +
          ",height=" + std::to_string(g_cap_height) +
          ",framerate=" + std::to_string(g_cap_fps) + "/1 ! "
          "videoconvert ! video/x-raw,format=I420 ! " + data_branch + mid + encode_tail;
    }
    std::cout << "Pipeline: " << pipeline_str << std::endl;

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline || error) {
      // Newer JetPacks (e.g. Thor R38) drop the maxperf-enable property from
      // nvv4l2h264enc; retry without it so one binary runs on Orin and Thor.
      if (error)
        g_clear_error(&error);
      error = nullptr;
      std::string alt = pipeline_str;
      size_t pos = alt.find("maxperf-enable=1 ");
      if (pos != std::string::npos) {
        alt.erase(pos, strlen("maxperf-enable=1 "));
        std::cout << "Retrying pipeline without maxperf-enable" << std::endl;
        if (pipeline)
          gst_object_unref(pipeline);
        pipeline = gst_parse_launch(alt.c_str(), &error);
      }
    }
    if (!pipeline) {
      std::cerr << "Failed to create pipeline: "
                << (error ? error->message : "unknown") << std::endl;
      if (error)
        g_clear_error(&error);
      return;
    }
    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    if (!appsink) {
      std::cerr << "Failed to get appsink" << std::endl;
      gst_object_unref(pipeline);
      return;
    }
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), nullptr);
    GstElement *datasink = gst_bin_get_by_name(GST_BIN(pipeline), "datasink");
    if (datasink) {
      g_signal_connect(datasink, "new-sample", G_CALLBACK(on_new_data_sample), nullptr);
      gst_object_unref(datasink);
    }
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    std::cout << "Streaming IR feed to " << send_to_server << ":"
              << send_to_port << " ..." << std::endl;
    while (streaming_active.load() && !stop_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Streaming loop ended, cleaning up..." << std::endl;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
  } catch (const std::exception &e) {
    std::cerr << "Streaming thread error: " << e.what() << std::endl;
  }
  std::cout << "Streaming thread finished" << std::endl;
}

// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);
  signal(SIGINT, handle_sigint);

  bool listen_enabled = false;
  std::string listen_address = "";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--listen" && i + 1 < argc) {
      listen_enabled = true;
      listen_address = argv[++i];
    } else if (arg == "--device" && i + 1 < argc) {
      g_device = argv[++i];
    } else if (arg == "--pixfmt" && i + 1 < argc) {
      g_pixfmt = argv[++i];
    } else if (arg == "--width" && i + 1 < argc) {
      g_cap_width = std::stoi(argv[++i]);
    } else if (arg == "--height" && i + 1 < argc) {
      g_cap_height = std::stoi(argv[++i]);
    } else if (arg == "--fps" && i + 1 < argc) {
      g_cap_fps = std::stoi(argv[++i]);
    } else if (arg == "--flip" && i + 1 < argc) {
      g_flip = std::stoi(argv[++i]);
    } else if (arg == "--zmq-pub" && i + 1 < argc) {
      g_zmq_pub_port = std::stoi(argv[++i]);
    } else if (arg == "--max-bitrate" && i + 1 < argc) {
      g_max_bitrate = std::stoi(argv[++i]);
    } else if (arg == "--video-via" && i + 1 < argc) {
      g_video_via = argv[++i];
    } else if (arg == "--fit" && i + 1 < argc) {
      g_letterbox = std::string(argv[++i]) != "stretch";
    } else if (arg == "--second-device" && i + 1 < argc) {
      g_device2 = argv[++i];
    } else if (arg == "--second-pixfmt" && i + 1 < argc) {
      g_pixfmt2 = argv[++i];
    } else if (arg == "--second-width" && i + 1 < argc) {
      g_cap2_width = std::stoi(argv[++i]);
    } else if (arg == "--second-height" && i + 1 < argc) {
      g_cap2_height = std::stoi(argv[++i]);
    } else if (arg == "--second-fps" && i + 1 < argc) {
      g_cap2_fps = std::stoi(argv[++i]);
    } else if (arg == "--second-flip" && i + 1 < argc) {
      g_flip2 = std::stoi(argv[++i]);
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0] << " --listen IP:PORT [options]\n"
                << "  Serves a V4L2 camera to XRoboToolkit Remote Vision.\n"
                << "  The headset connects to IP:PORT, sends OPEN_CAMERA with "
                   "its\n"
                << "  callback ip:port, and we stream H.264 back to it.\n"
                << "Options (defaults = D430i left-IR head cam):\n"
                << "  --device /dev/video2   V4L2 node\n"
                << "  --pixfmt GRAY8         GStreamer format (GRAY8, YUY2, UYVY)\n"
                << "  --width 640 --height 480 --fps 30\n"
                << "  --flip 0               nvvidconv flip-method (1=90ccw, 2=180, 3=90cw)\n"
                << "Examples:\n"
                << "  " << argv[0] << " --listen 0.0.0.0:13579\n"
                << "  " << argv[0] << " --listen 0.0.0.0:13580 --device /dev/video0"
                   " --pixfmt YUY2 --width 1280 --height 704 --fps 15\n";
      return 0;
    }
  }
  if (!listen_enabled) {
    std::cerr << "Error: --listen IP:PORT is required "
                 "(e.g. --listen 0.0.0.0:13579)\n";
    std::cerr << "Use --help to see usage options" << std::endl;
    return -1;
  }

  if (g_zmq_pub_port > 0) {
    g_zmq_ctx = zmq_ctx_new();
    g_zmq_pub = zmq_socket(g_zmq_ctx, ZMQ_PUB);
    std::string ep = "tcp://*:" + std::to_string(g_zmq_pub_port);
    if (zmq_bind(g_zmq_pub, ep.c_str()) != 0) {
      std::cerr << "zmq bind failed on " << ep << std::endl;
      return -1;
    }
    std::cout << "Publishing ego_view JPEGs (composed-camera protocol) on " << ep
              << std::endl;
    if (g_device2.rfind("udp:", 0) == 0)
      start_head_recorder(std::stoi(g_device2.substr(4)) + 1);
  }

  std::cout << "Starting IR video streaming server (listen " << listen_address
            << ")..." << std::endl;
  listen_thread =
      make_unique_helper<std::thread>(listenThreadFunction, listen_address);
  std::cout << "Server started. Press Ctrl+C to stop." << std::endl;
  while (!stop_requested.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (listen_thread && listen_thread->joinable())
    listen_thread->join();

  std::cout << "Shutting down..." << std::endl;
  stopStreamingThread();
  std::cout << "All threads stopped. Exiting." << std::endl;
  return 0;
}
