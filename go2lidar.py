#!/usr/bin/env python3
"""
Go2 LiDAR viewer for STEM Night.

Connects to a Unitree Go2 over WebRTC (AP mode: the Pi joins the dog's Wi-Fi),
decodes the LiDAR voxel map, and serves a full-screen 3D viewer at
http://localhost:8080 .

  python go2lidar.py               # connect to the robot (AP mode, 192.168.12.1)
  python go2lidar.py --key <hex>   # firmware 1.1.15+ needs the per-robot AES key

The AES key can also be put in a file named aes_key.txt next to this script.
"""
import argparse
import asyncio
import json
import logging
import math
import os
import struct
import sys
import threading
import time
import types
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WEB_DIR = HERE / "web"
MAX_POINTS = 200_000

log = logging.getLogger("go2lidar")


# ---------------------------------------------------------------- shared state
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = b""          # latest binary frame for the browser
        self.seq = 0
        self.pose = (0.0, 0.0, 0.0, 0.0)  # x, y, z, yaw
        self.last_pose_time = 0.0
        self.battery = None           # state of charge, %
        self.battery_time = 0.0
        self.status = "starting"
        self.message = "Starting up..."
        self.last_frame_time = 0.0
        self.fps = 0.0

    def set_status(self, status, message):
        with self.lock:
            self.status, self.message = status, message
        log.info("[%s] %s", status, message)

    def set_pose(self, x, y, z, yaw):
        with self.lock:
            self.pose = (x, y, z, yaw)
            self.last_pose_time = time.time()

    def set_points(self, pts):
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        if len(pts) > MAX_POINTS:
            pts = pts[:: math.ceil(len(pts) / MAX_POINTS)]
        with self.lock:
            # header: count, robot x, y, z, yaw (5 x float32), then xyz points
            header = struct.pack("<5f", float(len(pts)), *self.pose)
            self.frame = header + pts.tobytes()
            self.seq += 1
            now = time.time()
            if self.last_frame_time:
                dt = now - self.last_frame_time
                self.fps = 0.8 * self.fps + 0.2 * (1.0 / dt if dt > 0 else 0)
            self.last_frame_time = now


STATE = State()


# ---------------------------------------------------------------- camera
class Camera:
    """Holds the newest video frame; a worker thread JPEG-encodes it for
    browsers (served as an MJPEG stream at /camera.mjpg)."""

    def __init__(self, max_fps=15, width=960):
        self.cond = threading.Condition()
        self.raw = None           # newest PIL image (unencoded)
        self.jpeg = b""
        self.jseq = 0
        self.last_time = 0.0
        self.max_fps, self.width = max_fps, width
        threading.Thread(target=self._encoder, daemon=True).start()

    def put_image(self, img):
        with self.cond:
            self.raw = img
            self.cond.notify_all()

    def _encoder(self):
        import io
        while True:
            with self.cond:
                while self.raw is None:
                    self.cond.wait()
                img, self.raw = self.raw, None
            if img.width > self.width:
                img = img.resize((self.width, round(img.height * self.width / img.width)))
            buf = io.BytesIO()
            img.convert("RGB").save(buf, "JPEG", quality=75)
            with self.cond:
                self.jpeg, self.jseq, self.last_time = buf.getvalue(), self.jseq + 1, time.time()
                self.cond.notify_all()
            time.sleep(1.0 / self.max_fps)

    def wait_jpeg(self, after_seq, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.jseq != after_seq, timeout=timeout)
            return self.jpeg, self.jseq


CAMERA = Camera()


# ---------------------------------------------------------------- tricks
# Gentle tricks only (no flips/jumps around kids). These api_ids are the same
# in the classic and the newer MCF sport API, so they work on any firmware.
TRICKS = [  # (key, button label, api_id)
    ("hello",   "Hello / Shake", 1016),   # raises a front paw (the app's wave / handshake)
    ("heart",   "Heart",     1036),       # "FingerHeart" in the classic API, "Heart" in MCF
    ("stretch", "Stretch",   1017),
    ("dance1",  "Dance 1",   1022),
    ("dance2",  "Dance 2",   1023),
    ("happy",   "Happy",     1020),
    ("scrape",  "Scrape",    1029),
    ("sit",     "Sit",       1009),
    ("getup",   "Get up",    1010),
    ("stand",   "Stand up",  1004),
    ("lie",     "Lie down",  1005),
    ("recover", "Recover",   1006),
    ("stop",    "Stop",      1003),
]
TRICK_IDS = {k: api for k, _, api in TRICKS}
SPORT_TOPIC = "rt/api/sport/request"


class Robot:
    """The live connection, so the web server can send commands to it."""
    def __init__(self):
        self.conn = None
        self.loop = None
        self.busy = threading.Lock()   # one trick at a time
        self.enabled = True

    def send_trick(self, key, timeout=6.0):
        if not self.enabled:
            return 403, {"ok": False, "error": "Tricks are turned off"}
        if key not in TRICK_IDS:
            return 400, {"ok": False, "error": "Unknown trick"}
        conn, loop = self.conn, self.loop
        if conn is None or loop is None:
            return 503, {"ok": False, "error": "Robot not connected"}
        # Stop always goes through, even while another trick is running.
        if key != "stop" and not self.busy.acquire(blocking=False):
            return 429, {"ok": False, "error": "Still doing the last trick"}
        try:
            fut = asyncio.run_coroutine_threadsafe(
                conn.datachannel.pub_sub.publish_request_new(
                    SPORT_TOPIC, {"api_id": TRICK_IDS[key]}), loop)
            try:
                resp = fut.result(timeout=timeout)
            except TimeoutError:
                return 200, {"ok": True, "note": "sent"}   # long tricks reply late
            code = (resp or {}).get("data", {}).get("header", {}).get("status", {}).get("code", 0)
            if code != 0:
                return 502, {"ok": False, "error": f"Robot said no (code {code})"}
            return 200, {"ok": True}
        except Exception as e:
            log.warning("trick %s failed: %r", key, e)
            return 500, {"ok": False, "error": str(e)[:80]}
        finally:
            if key != "stop":
                self.busy.release()


ROBOT = Robot()


# ---------------------------------------------------------------- robot link
def _stub_audio():
    """The robot library imports sounddevice for audio we don't use. If the
    PortAudio system library is missing, substitute an empty module."""
    try:
        import sounddevice  # noqa: F401
    except Exception:
        sys.modules["sounddevice"] = types.ModuleType("sounddevice")


def _quiet_library_prints():
    """The library print()s connection-state lines on every attempt; send them
    to the debug log instead (shown with -v)."""
    import importlib
    def quiet(status_type, status_message):
        log.debug("%s: %s", status_type, status_message)
    for name in ("util", "webrtc_driver", "webrtc_datachannel", "unitree_auth"):
        try:
            mod = importlib.import_module("unitree_webrtc_connect." + name)
            if hasattr(mod, "print_status"):
                mod.print_status = quiet
        except Exception:
            pass


def _yaw_from_quat(q):
    x, y, z, w = (q.get(k, 0.0) for k in ("x", "y", "z", "w"))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _read_key_file():
    f = HERE / "aes_key.txt"
    try:
        return f.read_text().strip() or None
    except OSError:
        return None


async def robot_loop(aes_key, ip=None):
    _stub_audio()
    from unitree_webrtc_connect.webrtc_driver import (
        UnitreeWebRTCConnection, WebRTCConnectionMethod)
    _quiet_library_prints()

    attempt = 0
    while True:
        attempt += 1
        conn = None
        try:
            STATE.set_status("connecting",
                             f"Looking for the robot dog... (try {attempt})")
            key = aes_key or _read_key_file()
            kwargs = {"aes_128_key": key} if key else {}
            if ip:   # robot on a shared Wi-Fi network (STA mode)
                conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip, **kwargs)
            else:    # Pi joined the robot's own hotspot (AP mode)
                conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, **kwargs)
            await asyncio.wait_for(conn.connect(), timeout=30)

            await conn.datachannel.disableTrafficSaving(True)
            conn.datachannel.set_decoder(decoder_type="native")
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")

            def on_lidar(msg):
                try:
                    pts = msg["data"]["data"]["points"]
                    STATE.set_points(pts)
                except Exception as e:  # malformed frame: skip it
                    log.debug("bad lidar frame: %s", e)

            def on_pose(msg):
                try:
                    p = msg["data"]["pose"]
                    pos, ori = p["position"], p["orientation"]
                    STATE.set_pose(pos["x"], pos["y"], pos["z"], _yaw_from_quat(ori))
                except Exception as e:
                    log.debug("bad pose: %s", e)

            conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", on_lidar)
            conn.datachannel.pub_sub.subscribe("rt/utlidar/robot_pose", on_pose)

            seen_lowstate = []

            def on_lowstate(msg):
                if not seen_lowstate:   # log the first message's shape once, to debug the battery tile
                    seen_lowstate.append(1)
                    d = msg.get("data") if isinstance(msg, dict) else None
                    log.info("battery data: first lowstate message, keys=%s bms=%s",
                             list(d.keys())[:20] if isinstance(d, dict) else type(d).__name__,
                             d.get("bms_state") if isinstance(d, dict) else None)
                try:
                    soc = msg["data"]["bms_state"]["soc"]
                    with STATE.lock:
                        STATE.battery, STATE.battery_time = int(soc), time.time()
                except Exception as e:
                    log.debug("bad lowstate: %s", e)

            conn.datachannel.pub_sub.subscribe("rt/lf/lowstate", on_lowstate)

            # Front camera: grab frames as they arrive, hand them to the encoder.
            async def on_video(track):
                while True:
                    frame = await track.recv()
                    CAMERA.put_image(frame.to_image())

            conn.video.switchVideoChannel(True)
            conn.video.add_track_callback(on_video)

            ROBOT.conn, ROBOT.loop = conn, asyncio.get_running_loop()
            STATE.set_status("connected", "Connected to the robot dog!")
            attempt = 0

            # Watchdog: if no LiDAR frames for 15 s, reconnect.
            STATE.last_frame_time = time.time()
            connected_at = time.time()
            while True:
                await asyncio.sleep(2)
                if not seen_lowstate and time.time() - connected_at > 15:
                    seen_lowstate.append(1)
                    log.info("battery data: no lowstate messages after 15 s")
                if time.time() - STATE.last_frame_time > 15:
                    STATE.fps = 0.0
                    raise RuntimeError("no LiDAR data for 15 seconds")
        except Exception as e:
            if type(e).__name__ == "AesKeyRequiredError":
                STATE.set_status("connecting", "Robot needs its AES key: save it in aes_key.txt")
            else:
                STATE.set_status("connecting", f"Robot not connected ({str(e)[:80]}). Retrying...")
            log.debug("connection problem: %r", e)   # status line above already says why
        finally:
            ROBOT.conn = None
            if conn is not None:
                try:
                    await asyncio.wait_for(conn.disconnect(), timeout=5)
                except Exception:
                    pass
        await asyncio.sleep(3)


# ---------------------------------------------------------------- web server
class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(WEB_DIR), **kw)

    def log_message(self, *a):
        pass

    def _send(self, body, ctype, extra=None):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/tricks"):
            body = json.dumps({"enabled": ROBOT.enabled,
                               "tricks": [{"key": k, "label": l} for k, l, _ in TRICKS]}).encode()
            return self._send(body, "application/json")
        if self.path.startswith("/frame"):
            with STATE.lock:
                body, seq = STATE.frame, STATE.seq
            return self._send(body, "application/octet-stream", {"X-Seq": str(seq)})
        if self.path.startswith("/status"):
            with STATE.lock:
                stale = time.time() - STATE.last_frame_time > 5 if STATE.last_frame_time else True
                body = json.dumps({
                    "status": STATE.status, "message": STATE.message,
                    "seq": STATE.seq, "fps": round(STATE.fps, 1),
                    "stale": stale,
                    "camera": time.time() - CAMERA.last_time < 3,
                    "pose": time.time() - STATE.last_pose_time < 3,
                    "battery": STATE.battery if time.time() - STATE.battery_time < 10 else None,
                }).encode()
            return self._send(body, "application/json")
        if self.path.startswith("/camera.mjpg"):
            return self._mjpeg()
        return super().do_GET()

    def do_POST(self):
        if not self.path.startswith("/trick"):
            self.send_error(404); return
        try:
            n = int(self.headers.get("Content-Length", 0))
            key = json.loads(self.rfile.read(min(n, 1024)) or b"{}").get("trick", "")
        except (ValueError, AttributeError):
            key = ""
        status, result = ROBOT.send_trick(str(key))
        log.info("trick %r -> %s %s", key, status, result)
        body = json.dumps(result).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = -1
        try:
            while True:
                jpeg, seq = CAMERA.wait_jpeg(seq)
                if not jpeg:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                 + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=os.environ.get("GO2_IP"),
                    help="robot IP when it is on the same Wi-Fi (STA mode); omit for AP mode")
    ap.add_argument("--key", default=os.environ.get("GO2_AES_KEY"),
                    help="AES-128 key (needed for Go2 firmware 1.1.15+)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0",
                    help="0.0.0.0 lets other devices on the network watch too")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--no-tricks", action="store_true", help="hide and disable the tricks panel")
    ap.add_argument("--fetch-key", nargs=argparse.REMAINDER,
                    help="get the AES key from your Unitree account (needs internet); "
                         "extra args go to unitree-fetch-aes-key")
    args = ap.parse_args()

    if args.fetch_key is not None:
        _stub_audio()
        from unitree_webrtc_connect._cli import main as fetch_main
        sys.argv = ["unitree-fetch-aes-key", *args.fetch_key]
        sys.exit(fetch_main())

    # The robot library logs every data-channel message on the root logger,
    # so the root stays at WARNING and only this app's own logger shows INFO.
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING,
                        format="%(asctime)s %(levelname)s %(message)s")
    log.setLevel(logging.DEBUG if args.verbose else logging.INFO)
    if not args.verbose:
        logging.getLogger("aioice").setLevel(logging.ERROR)
        logging.getLogger("aiortc").setLevel(logging.ERROR)   # hides H264 "failed to decode" noise

    key = args.key or _read_key_file()

    ROBOT.enabled = not args.no_tricks
    threading.Thread(target=lambda: asyncio.run(robot_loop(key, args.ip)), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"\n  Go2 LiDAR viewer running:  http://localhost:{args.port}\n"
          f"  ({('STA mode, robot at ' + args.ip) if args.ip else 'AP mode, robot at 192.168.12.1'}"
          f"{', AES key set' if key else ''})  Ctrl+C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
