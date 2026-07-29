#!/usr/bin/env python3
"""Generic TCP relay: listen on a port, pipe each client to a target host:port.

Used on mjolnir to carry the camera H.264 stream from g1 to the PICO on the
hotspot: forwarded (routed) traffic g1->PICO gets firewall-rejected by the
Docker/NM chain soup, but mjolnir's own connections to both sides work fine.
The camera sender runs with --video-via <mjolnir>, so it connects here and we
pipe to the headset.

Usage: tcp_relay.py <listen_port> <target_ip> <target_port>
"""
import socket
import sys
import threading


def pump(src, dst):
    try:
        while True:
            d = src.recv(65536)
            if not d:
                break
            dst.sendall(d)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def main():
    lport, tip, tport = int(sys.argv[1]), sys.argv[2], int(sys.argv[3])
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", lport))
    srv.listen(2)
    print(f"[relay] 0.0.0.0:{lport} -> {tip}:{tport}", flush=True)
    while True:
        c, peer = srv.accept()
        print(f"[relay] client {peer[0]}", flush=True)
        try:
            t = socket.create_connection((tip, tport), timeout=5)
        except OSError as e:
            print(f"[relay] target unreachable: {e}", flush=True)
            c.close()
            continue
        threading.Thread(target=pump, args=(c, t), daemon=True).start()
        threading.Thread(target=pump, args=(t, c), daemon=True).start()


if __name__ == "__main__":
    main()
