#!/bin/sh
python3 car_server.py --port 9999   --signal-host 0.0.0.0 --signal-port 8770   --camera /dev/video0 --w 320 --h 240 --fps 20
