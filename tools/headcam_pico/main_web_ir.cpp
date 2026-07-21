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
#include <thread>
#include <vector>

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
std::unique_ptr<TCPServer> server_ptr;
std::string send_to_server = "";
int send_to_port = 0;

template <typename T, typename... Args>
std::unique_ptr<T> make_unique_helper(Args &&...args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

bool initialize_sender() {
  int retry = 10;
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
    send_to_server = cameraConfig.ip;
    send_to_port = cameraConfig.port;
    std::cout << "Updated sender target to " << send_to_server << ":"
              << send_to_port << std::endl;
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
    std::cout << "Streaming thread already running" << std::endl;
    return;
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
    // Scale the 640x480 IR feed to whatever resolution the headset requested
    // (falls back to native 640x480); honor requested bitrate if provided.
    int outW = config.width > 0 ? config.width : 640;
    int outH = config.height > 0 ? config.height : 480;
    int bitrate = config.bitrate > 0 ? config.bitrate : 4000000;

    std::string pipeline_str =
        "v4l2src device=/dev/video2 ! "
        "video/x-raw,format=GRAY8,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! video/x-raw,format=NV12 ! "
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=" +
        std::to_string(outW) + ",height=" + std::to_string(outH) +
        " ! "
        "nvv4l2h264enc maxperf-enable=1 insert-sps-pps=true idrinterval=15 "
        "bitrate=" +
        std::to_string(bitrate) +
        " ! "
        "h264parse ! appsink name=mysink emit-signals=true sync=false";
    std::cout << "Pipeline: " << pipeline_str << std::endl;

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
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
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0] << " --listen IP:PORT\n"
                << "  Serves the G1 head-camera IR feed to XRoboToolkit Remote "
                   "Vision.\n"
                << "  The headset connects to IP:PORT, sends OPEN_CAMERA with "
                   "its\n"
                << "  callback ip:port, and we stream H.264 back to it.\n"
                << "  Example: " << argv[0] << " --listen 0.0.0.0:13579\n";
      return 0;
    }
  }
  if (!listen_enabled) {
    std::cerr << "Error: --listen IP:PORT is required "
                 "(e.g. --listen 0.0.0.0:13579)\n";
    std::cerr << "Use --help to see usage options" << std::endl;
    return -1;
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
