#!/usr/bin/env bash
# Start the Go2 LiDAR + camera viewer and open it full-screen.
#   ./run.sh                     -> AP mode: Pi is on the Go2's Wi-Fi hotspot
#   ./run.sh --ip 192.168.1.181  -> STA mode: robot is on the same Wi-Fi as the Pi
# Press Alt+F4 (or Ctrl+W) to close the full-screen browser; the viewer stops with it.
cd "$(dirname "$0")"
PORT=8080

.venv/bin/python go2lidar.py --port "$PORT" "$@" &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT

# wait for the server to come up
for _ in $(seq 1 30); do
  curl -s -o /dev/null "http://localhost:$PORT/status" && break
  sleep 0.5
done

# separate profile so kiosk mode works even if Firefox is already open
PROFILE="$HOME/snap/firefox/common/go2-kiosk"
mkdir -p "$PROFILE"
firefox --no-remote --profile "$PROFILE" --kiosk "http://localhost:$PORT/"
