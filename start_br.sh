#!/bin/sh
python3 ws_control_bridge_webrtc.py --ws-host 0.0.0.0 --ws-port 8765   --udp-host 127.0.0.1 --udp-port 9999   --signal-url ws://127.0.0.1:8770
