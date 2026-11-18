#!/usr/bin/env bash
# Start the Go2 LiDAR + camera viewer, then open http://localhost:8080 in a browser.
#   ./run.sh                     -> AP mode: Pi is on the Go2's Wi-Fi hotspot
#   ./run.sh --ip 192.168.1.181  -> STA mode: robot is on the same Wi-Fi as the Pi
# Press Ctrl+C to stop.
cd "$(dirname "$0")"
exec .venv/bin/python go2lidar.py --port 8080 "$@"
