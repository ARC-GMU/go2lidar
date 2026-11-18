# Go2 LiDAR Viewer

![Python](https://img.shields.io/badge/python-3.12-3776AB?style=flat-square&logo=python&logoColor=white)
![Raspberry Pi 5](https://img.shields.io/badge/Raspberry%20Pi-5-A22846?style=flat-square&logo=raspberrypi&logoColor=white)
![Unitree Go2](https://img.shields.io/badge/robot-Unitree%20Go2-ff6b35?style=flat-square)
![three.js](https://img.shields.io/badge/three.js-r160-000000?style=flat-square&logo=threedotjs&logoColor=white)
![WebRTC](https://img.shields.io/badge/link-WebRTC-333333?style=flat-square&logo=webrtc&logoColor=white)
![Works offline](https://img.shields.io/badge/internet-not%20required-2ec27e?style=flat-square)

A full-screen, kid-friendly display of what a Unitree Go2 robot dog "sees",
built for an elementary school STEM Night. It runs on a Raspberry Pi 5 and
shows:

- **Live LiDAR point cloud** in 3D, colored by height in six flat bands
  (floor, ankle, knee, waist, chest, head)
- **Two views:** an orbiting overview with a robot-dog marker, and a
  **Dog's eyes** POV that looks out from the robot's head
- **Front camera feed** from the robot, swappable with the 3D view
- **Find yourself:** dots that just appeared (someone moving) turn white
  while everything else dims, so kids can wave and spot themselves
- **Distance ruler:** click any dot to see how far it is from the dog, in
  meters, school rulers and big steps, plus how high it is
- **Paw print trail** of where the dog walked, and **map memory** that keeps
  everything the dog has seen in its height colours, so the room builds up as
  it walks. The map stays current: spots in the area being scanned that are
  no longer there are removed, and people walking past don't leave ghosts
- **Minimap:** top-down map (north up) with the dog's heading, paw trail and
  measured spots; zoom it, or enlarge it
- **Battery gauge**, dot count and scan rate
- **Display panel** to show or hide any part of the screen (saved between
  visits), and **H** to hide everything except the 3D view
- Rotating "Did you know?" facts

Everything (including three.js) is served from the Pi, so it works on the
robot's own Wi-Fi hotspot with no internet.

## Quick start

```bash
./run.sh                       # AP mode: Pi is on the Go2's Wi-Fi hotspot
./run.sh --ip 192.168.1.181    # STA mode: robot is on the same Wi-Fi as the Pi
```

`run.sh` starts the server. Open `http://localhost:8080` in a browser
(press **F** on the page for full screen). Stop the server with **Ctrl+C**.
Other devices on the same network can watch at `http://<pi-ip>:8080`.

### Controls

| Input | Action |
|---|---|
| Drag / scroll | Spin / zoom the overview |
| **V** or top-bar buttons | Switch Overview / Dog's eyes |
| **C** or click camera | Swap big camera / big 3D view |
| **R** | Reset to the overview |
| **T** or click "Tricks" | Open / close the tricks panel |
| **Y** or "Find yourself" | Highlight things that just moved |
| Click a dot | Measure its distance from the dog (**Esc** clears) |
| **H** | Hide / show all on-screen panels |
| **M** | Show / hide the minimap |
| Click / scroll the map | Zoom the minimap |
| Double-click map or "Enlarge" | Enlarge / shrink the map (**Esc** shrinks) |
| **F** | Browser full screen |

### Tricks

A collapsible **Tricks** panel (top right, closed by default) sends gentle
tricks: Hello / Shake, Heart, Stretch, Dance 1/2, Happy, Scrape, Sit, Get up, Stand up,
Lie down, Recover, plus a **Stop** button that always goes through. Flips,
jumps and handstands are deliberately left out. The server runs one trick at
a time and only accepts tricks from this list. Start with `--no-tricks` to
hide the panel and disable the endpoint.

## Connecting to the robot

**AP mode (used at the event):** turn on the Go2, join its hotspot from the Pi
(named like `Go2_XXX`), then run `./run.sh`. The robot is at `192.168.12.1`.

**STA mode:** if the robot and Pi are on the same Wi-Fi, pass the robot's IP
with `--ip`.

Only one client can hold the robot's WebRTC connection, so close the Unitree
phone app first and drive the dog with the handheld controller.

### AES key (Go2 firmware 1.1.15 and newer)

Newer firmware requires a per-robot key from your Unitree account. Fetch it
once while the Pi has internet:

```bash
.venv/bin/python go2lidar.py --fetch-key --device-type Go2 --email YOUR_UNITREE_EMAIL
```

Save the 32-character key in `aes_key.txt` in this folder (it is git-ignored).
The server re-reads the file on every connection attempt, so no restart is
needed.

## Setup

Uses [uv](https://github.com/astral-sh/uv) and
[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect).

```bash
uv venv .venv --python /usr/bin/python3
uv pip install --python .venv/bin/python --no-deps unitree_webrtc_connect
uv pip install --python .venv/bin/python aiortc pycryptodome requests curl_cffi wasmtime lz4 numpy packaging pillow
```

The library's audio extras (pyaudio, sounddevice) are skipped on purpose; the
viewer stubs them out because LiDAR and video don't need audio.

**Key-fetch fix:** Unitree's login server now sits behind bot protection that
blocks library v2.2.0. Until a fixed release is out, apply upstream
[PR #69](https://github.com/legion1581/unitree_webrtc_connect/pull/69): in
`.venv/lib/python3.12/site-packages/unitree_webrtc_connect/unitree_cloud.py`
change `impersonate="chrome120"` to `impersonate="safari17_0"`.

## How it works

```
Go2 ──WebRTC──> go2lidar.py ──HTTP──> browser (web/index.html)
                 │ decodes LiDAR voxel map        │ three.js point cloud
                 │ JPEG-encodes camera frames     │ polls /frame, /status
                 └ serves /frame /status /camera.mjpg
```

| File | Purpose |
|---|---|
| `go2lidar.py` | Connects to the robot, decodes LiDAR and pose, encodes video, serves the page |
| `web/index.html` | The viewer: three.js scene, overview and POV modes, UI |
| `web/vendor/` | Bundled three.js r160 and OrbitControls (MIT) |
| `run.sh` | Starts the server |

**Firefox on the Pi:** Firefox renders WebGL but never puts a WebGL canvas on
screen on this setup, so three.js draws off-screen and each frame is copied
to a visible 2D canvas.

### Troubleshooting

- Status stays yellow: check the Pi is on the robot's Wi-Fi and the phone app
  is closed. Run `.venv/bin/python go2lidar.py -v` for detailed logs.
- "Robot needs its AES key": see the AES key section above.
- Script errors are shown in a red box on the page.
