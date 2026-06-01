# 🌱 SAQI Backend

A **FastAPI**-powered backend for a Raspberry Pi agricultural robot. Controls
motors, a water pump, a live AXIS PTZ camera feed, AI-based plant detection
with **ByteTrack** tracking + appearance **Re-ID**, and a fully autonomous
"find-and-water" loop — all exposed as a REST API with open CORS so a browser
frontend can call it directly.

---

## 📁 Project Structure

```
Backend/
├── components/
│   ├── camera.py          # MJPEG camera capture (AXIS 213, background thread)
│   ├── camera_control.py  # PTZ HTTP control (AXIS VAPIX)
│   ├── ai.py              # YOLO + ByteTrack detection & annotation
│   ├── reid.py            # MobileNet-V3 multi-view appearance Re-ID
│   ├── motor.py           # Differential-drive motor control (2× BTS7960, PWM)
│   ├── waterpump.py       # Relay-driven water pump
│   ├── buzzer.py          # Passive-buzzer audio cues (GPIO PWM)
│   ├── automatic.py       # Autonomous navigation + inference loop
│   └── .env               # AI config (TARGET_CLASS, tracking toggle)
├── ai_models/             # YOLO weights (.pt / .onnx / ncnn)
│   └── yolo26n.onnx       # ← default model loaded by ai.py
├── main.py                # FastAPI application (entry point)
├── requirements.txt       # Python dependencies
├── start.sh               # One-command startup script
└── README.md
```

> The default model is **`ai_models/yolo26n.onnx`** (set in
> `components/ai.py` → `_DEFAULT_MODEL_PATH`). Other weights in `ai_models/`
> (`best.pt`, `yolo11n.*`, ncnn exports) are alternatives — point
> `_DEFAULT_MODEL_PATH` at whichever you want.

---

## ⚙️ Hardware / GPIO Pin Map

All pins are **BCM GPIO numbers**. They are defined in **one place** —
`components/config.py` (single source of truth). Edit pin numbers there;
`motor.py` / `waterpump.py` import from it. The tables below are reference
only.

### Motors — 2× BTS7960 drivers via gpiozero `Robot` (hardware PWM @ 1 kHz)

| Function                  | GPIO | Notes                                  |
|---------------------------|------|----------------------------------------|
| Left motor — forward PWM  | 12   | `RPWM` left                            |
| Left motor — backward PWM | 13   | `LPWM` left                            |
| Left motor — enable A     | 5    | BTS7960 enable                         |
| Left motor — enable B     | 17   | BTS7960 enable (driven on at startup)  |
| Right motor — forward PWM | 22   | `RPWM` right                           |
| Right motor — backward PWM| 23   | `LPWM` right                           |
| Right motor — enable A    | 6    | BTS7960 enable                         |
| Right motor — enable B    | 27   | BTS7960 enable (driven on at startup)  |

> Software trim (`left_trim` / `right_trim` in `motor.py`) lets you correct
> drift so the robot drives straight. `left()` / `right()` internally double
> the requested speed (capped at 1.0) for a snappier in-place turn.

### Water pump

| Function   | GPIO | Notes                                            |
|------------|------|--------------------------------------------------|
| Pump relay | 25   | **Active-LOW** relay, starts OFF (`initial_value=False`) |

### Passive buzzer

| Function      | GPIO | Notes                                      |
|---------------|------|--------------------------------------------|
| Buzzer signal | 26   | PWM output for a passive buzzer; other lead to GND |

The buzzer plays distinct pump-on and pump-off cues with a watering melody,
rising or falling chirps when automatic mode is enabled or disabled, and
faster, higher-pitched proximity beeps below 35 cm. At 17 cm or closer it
sounds a steady high alarm for two seconds, then stays quiet until the reading
exceeds 20 cm so close-range testing does not keep making noise.

`Ode to Joy` is the default watering song. The settings UI at `/settings/ui`
can switch live between `Ode to Joy`, `Twinkle, Twinkle, Little Star`,
`Mary Had a Little Lamb`, `Happy Birthday`, and `Jingle Bells`. Turning
watering music off restores the simpler interval beep. Edit the
`BUZZER_WATER_*` defaults and melody catalog in `components/config.py`.

### Status LEDs (WS2812 / NeoPixel, 8 LEDs)

| Function     | GPIO | Notes                                            |
|--------------|------|--------------------------------------------------|
| LED data in  | 18   | PWM. `rpi_ws281x` usually needs **root** — without it the panel self-disables (system still runs) |

8-LED meaning (auto-driven by system events): **1** Manual/Auto · **2** Wifi ·
**3** Camera · **4** Ultrasonic · **5** Moving · **6** Detect · **7** Watering
· **8** Error. Manual mode = red, Auto = green, init = orange blink,
ready = green, error = red/blink.

### Ultrasonic sensor head (ESP32 + HC-SR04 + servo)

| Function | Connection | Notes                                          |
|----------|------------|------------------------------------------------|
| ESP32 sensor head | USB serial | Configure `ULTRASONIC_SERIAL_PORT` in `components/config.py` |
Manual mode logs front/left/right readings and drives the Ultrasonic LED.
Automatic mode also uses the sensor for obstacle avoidance when distance is
below `ULTRASONIC_OBSTACLE_CM`. For watering, a centered close-up YOLO plant
gets a bounded ultrasonic-only final approach so the robot can still water
after the camera loses sight of an oversized nearby plant.

### Camera (no GPIO — network device)

| Function          | Connection                                            |
|-------------------|-------------------------------------------------------|
| Video (MJPEG)     | `http://169.254.138.53/axis-cgi/mjpg/video.cgi`       |
| PTZ control       | `http://169.254.138.53/axis-cgi/com/ptz.cgi` (VAPIX)  |

---

## 📋 System Prerequisites

Install once on a fresh Raspberry Pi OS:

```bash
sudo apt update
sudo apt install -y swig liblgpio-dev python3-dev python3-venv
```

| Package        | Why it's needed                               |
|----------------|-----------------------------------------------|
| `swig`         | Builds the `lgpio` Python wheel               |
| `liblgpio-dev` | C library for GPIO access                     |
| `python3-dev`  | Headers for native extensions                 |
| `python3-venv` | Virtual environments                          |

---

## 🔧 Configuration

### `components/.env` (AI settings — read by `components/ai.py`)

```ini
TARGET_CLASS="potted plant"   # YOLO class the bot waters
USE_YOLO_WITH_TRACK=True      # ByteTrack tracking (code currently forces ON)
```

### Hardcoded settings (edit the module directly)

| Setting           | File                           | Default                                          |
|-------------------|--------------------------------|--------------------------------------------------|
| PTZ host/user/pass| `components/camera_control.py` | `169.254.138.53` / `root` / `root`               |
| MJPEG source URL  | `components/camera.py`         | `http://169.254.138.53/axis-cgi/mjpg/video.cgi`  |
| Default YOLO model| `components/ai.py`             | `ai_models/yolo26n.onnx`                         |
| ReID similarity   | `components/reid.py`           | `0.82` cosine (top-2 mean)                       |
| Nav tuning consts | `components/automatic.py`      | speeds, scan steps, settle times (top of file)   |
| Buzzer melodies   | `components/config.py`         | `Ode to Joy` while watering                      |

If the PTZ camera is unreachable at startup, PTZ disables itself (logs a
warning) and the navigator degrades to motor-only scanning — the rest of the
bot keeps working.

---

## 🚀 Quick Start

```bash
# 1. Copy to the Pi
scp -r Backend/ pi@<pi-ip>:~/Desktop/

# 2. System deps (first time only)
sudo apt update && sudo apt install -y swig liblgpio-dev python3-dev python3-venv

# 3. Run
cd ~/Desktop/Backend
chmod +x start.sh
./start.sh
```

`start.sh` creates/activates the venv, installs `requirements.txt` (first run
only), starts uvicorn on `http://0.0.0.0:8000`, and optionally opens a
Cloudflare tunnel.

**Manual start:**
```bash
cd ~/Desktop/Backend
source venv/bin/activate
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Interactive API docs: `http://<pi-ip>:8000/docs` (Swagger) ·
`http://<pi-ip>:8000/redoc`.

---

## 🤖 Autonomous Mode — how it behaves

Autonomous mode is engaged by requesting `/camera/feed?mode=automatic`
**once**. Important behaviour for frontend devs:

- **It is decoupled from the stream.** YOLO inference + navigation run in
  their own background threads. Once started, **the robot keeps detecting and
  driving even if the browser closes the video stream / loses connection.**
- **To stop autonomous mode**, request `/camera/feed?mode=manual` (or shut
  the server down). Manual mode stops the navigator and streams the raw
  camera. There is intentionally **no separate start/stop endpoint** — the
  `mode` query param is the switch.
- In automatic mode **YOLO + ByteTrack run continuously on every frame**
  (not on a timer). Navigation only *acts* on detections at settle points
  (when the robot/camera is stopped), so motion-blurred frames don't cause
  bad moves.

State machine:

1. **Track** → pick the highest-confidence *unwatered* plant; turn
   (LEFT/RIGHT zone) or drive forward (CENTER).
2. **Water handoff** → once a centered YOLO box reaches
   `YOLO_WATER_HANDOFF_AREA_RATIO`, ultrasonic owns the bounded final approach
   even if YOLO loses the oversized close-up plant. Once front distance is
   within `ULTRASONIC_WATER_DISTANCE_CM`, the pump runs 5 s. If ultrasonic is
   unavailable, centered YOLO area falls back to `ARRIVAL_AREA_RATIO`.
3. **Watered-plant memory** →
   - ByteTrack IDs of watered plants → drawn blue / `WATERED`.
   - **Multi-view appearance Re-ID**: several embeddings are captured per
     plant at watering and stored as one cluster; a later detection matches
     if the top-2 mean cosine similarity ≥ 0.82. This survives ByteTrack ID
     loss after occlusion, a 180° turn, or a PTZ sweep. Falls back to
     ID-only memory if torch/torchvision is missing.
4. **Post-water reverse** → always reverse 2 s after watering, then look for
   the next unwatered plant or the base QR when the watering quota is complete.
5. **Scan when lost** (biased toward the side the plant was last pursued):
   - **Stepped camera sweep** — pan in **22.5° hops up to ±90°**, biased
     side first; the camera fully stops and settles before each detection
     read, so YOLO never sees a blurred frame.
   - **Stepped motor scan** — if the sweep finds nothing, rotate the robot
     in **4 discrete ~45° steps (~180° total)**, checking for plants between
     every step and bailing early the moment one appears.
   - No PTZ camera → camera sweep is skipped; motor scan only.

Obstacle avoidance runs only when there is no active watering handoff. The
ESP32 scans left and right, then the robot reverses, swings toward the clearer
side, drives forward, and undoes the swing before continuing.

Watering threshold tuning:

- `YOLO_CENTER_SIDE_MARGIN`: bigger value means a narrower, stricter centered
  zone; smaller value makes centering easier.
- `YOLO_WATER_HANDOFF_AREA_RATIO`: bigger value means YOLO must see a larger
  box, so the robot hands off closer to the plant.
- `ULTRASONIC_WATER_DISTANCE_CM`: bigger value means water farther away;
  smaller value means drive closer before watering.
- `WATER_HANDOFF_FORWARD_SPEED`, `WATER_HANDOFF_FORWARD_SECONDS`, and
  `WATER_HANDOFF_TIMEOUT_SECONDS`: tune the slower ultrasonic-only approach.
- `POST_WATER_REVERSE_SECONDS`: mandatory reverse after watering before
  searching for another plant or acting on the base QR.

---

## 📡 API Reference (for frontend developers)

Base URL: `http://<pi-ip>:8000`. CORS is fully open
(`Access-Control-Allow-Origin: *`), so you can `fetch()` these directly from
a browser app on any origin. All bodies are JSON; control endpoints are
`POST` with query-string params (no request body needed).

### Root

| Method | Endpoint | Response |
|--------|----------|----------|
| GET | `/` | `{"message": "...", "endpoints": {...}}` |

### Motor

| Method | Endpoint | Params | Success response |
|--------|----------|--------|------------------|
| POST | `/motor/move` | `direction`=forward\|backward\|left\|right, `speed`=0.0–1.0 (def 0.8) | `{"status":"moving","direction":"forward","speed":0.8}` |
| POST | `/motor/stop` | — | `{"status":"stopped"}` |

Invalid direction → `400 {"status":"error","message":"Invalid direction: x"}`.

```js
// Drive forward at 80%
await fetch(`${BASE}/motor/move?direction=forward&speed=0.8`, { method: 'POST' });
// Stop
await fetch(`${BASE}/motor/stop`, { method: 'POST' });
```

### Camera — video

| Method | Endpoint | Params | Description |
|--------|----------|--------|-------------|
| GET | `/camera/feed` | `mode`=manual\|automatic (def manual) | Live MJPEG stream (`multipart/x-mixed-replace`) |
| GET | `/camera/snapshot` | — | Single JPEG (or `503` JSON if no frame) |

Render the stream straight into an `<img>` — no JS decoding needed:

```html
<!-- Plain live view -->
<img id="cam" src="http://<pi-ip>:8000/camera/feed" />

<!-- Engage autonomous mode (boxes + WATERED labels overlaid).
     The robot keeps running even if this <img> is later removed. -->
<img src="http://<pi-ip>:8000/camera/feed?mode=automatic" />
```

```js
// Toggle autonomous mode by swapping the stream URL.
const cam = document.getElementById('cam');
function setAuto(on) {
  cam.src = `${BASE}/camera/feed?mode=${on ? 'automatic' : 'manual'}`;
}
// setAuto(true)  → start find-and-water (persists past disconnect)
// setAuto(false) → stop navigator, back to raw video
```

> Tip: add a cache-busting `&t=${Date.now()}` when re-assigning `src` if the
> browser refuses to restart the stream.

### Camera — PTZ (pan / tilt / zoom)

Every PTZ endpoint returns `503 {"status":"error","message":"PTZ camera
control not available"}` if the AXIS camera is unreachable. Absolute moves
(not relative).

| Method | Endpoint | Params | Success response |
|--------|----------|--------|------------------|
| POST | `/camera/ptz/pan` | `angle` -180..180 | `{"status":"ok","pan":45}` |
| POST | `/camera/ptz/tilt` | `angle` -90..90 | `{"status":"ok","tilt":-10}` |
| POST | `/camera/ptz/zoom` | `level` 1..9999 | `{"status":"ok","zoom":2000}` |
| POST | `/camera/ptz/move` | `direction`=left\|right\|up\|down, `speed` 1..100 (def 50) | `{"status":"ok","direction":"left","speed":50}` |
| POST | `/camera/ptz/stop` | — | `{"status":"ok"}` |
| POST | `/camera/ptz/look` | `direction`=left\|right\|up\|down\|center, `degrees` 0..180 (def 90) | `{"status":"ok","direction":"right","degrees":90}` |
| POST | `/camera/ptz/center` | — | `{"status":"ok"}` |
| POST | `/camera/ptz/home` | — | `{"status":"ok"}` |
| POST | `/camera/ptz/preset` | `name` **or** `number` (≥1) | `{"status":"ok","name":"Row_A","number":null}` |
| GET  | `/camera/ptz/position` | — | `{"status":"ok","position":{"pan":45.0,"tilt":0.0,"zoom":2000.0}}` |

`move` with an unknown direction → `400`. `degrees`: positive = right/up,
negative handled by direction. `pan`<0 = left. `look` with an unknown
direction → `400`. `preset` with neither name nor number → `400`.
`position` with no data from camera → `502`.

> **Absolute vs. continuous — important.** `pan`, `tilt`, `zoom`, `look`,
> `center`, `home`, `preset` are **absolute**: they send the camera to a
> fixed position. Sending the *same value again is a no-op* — if the camera
> is already there it (correctly) does nothing. Only **`/camera/ptz/move`**
> is **continuous/velocity** (move until `stop`), so it always responds to a
> repeated press. Use `move` for the arrow controls.

---

### 👉 What the frontend actually needs for live camera control

The camera-control UI is **4 arrows + speed + stop + zoom + center** — these
endpoints, nothing else:

- **`POST /camera/ptz/move?direction=left|right|up|down&speed=1..100`** —
  *continuous* (hold-to-move) pan/tilt. One arrow = one `direction`; `speed`
  (default 50) is how fast. The camera **keeps moving until you `stop`** — it
  does not stop on its own. Velocity-based, so holding/repeating an arrow
  always works (no absolute "already there" no-op).
- **`POST /camera/ptz/stop`** — halts the motion. **Every `move` must be
  paired with a `stop`** on the arrow's release (mouse-up / touch-end /
  mouse-leave), or the camera pans forever.
- **`POST /camera/ptz/zoom?level=1..9999`** — absolute zoom (1 = widest …
  9999 = telephoto); wire to a slider.
- **`POST /camera/ptz/center`** — recenter pan to 0° ("reset view" button).
- The UI also embeds the video **`GET /camera/feed`** to *see* the camera.

> Everything else in the table (`pan`, `tilt`, `look`, `home`, `preset`,
> `position`) is optional convenience — not needed for this UI.

**Arrow control — press to move, release to stop** (bind all four arrows;
this is the reference implementation):

```js
const SPEED = 50;
function bindArrow(el, direction) {
  const start = () =>
    fetch(`${BASE}/camera/ptz/move?direction=${direction}&speed=${SPEED}`,
          { method: 'POST' });
  const stop  = () =>
    fetch(`${BASE}/camera/ptz/stop`, { method: 'POST' });
  el.addEventListener('mousedown',  start);
  el.addEventListener('touchstart', start);
  el.addEventListener('mouseup',    stop);
  el.addEventListener('mouseleave', stop);   // pointer slips off the button
  el.addEventListener('touchend',   stop);
}
bindArrow(document.getElementById('arrowLeft'),  'left');
bindArrow(document.getElementById('arrowRight'), 'right');
bindArrow(document.getElementById('arrowUp'),    'up');
bindArrow(document.getElementById('arrowDown'),  'down');

// Zoom slider + center button
zoomSlider.addEventListener('change', e =>
  fetch(`${BASE}/camera/ptz/zoom?level=${e.target.value}`, { method: 'POST' }));
centerBtn.addEventListener('click', () =>
  fetch(`${BASE}/camera/ptz/center`, { method: 'POST' }));
```

**Snap-look buttons & position readout:**

```js
// "Look Left 90°" button
await fetch(`${BASE}/camera/ptz/look?direction=left&degrees=90`, { method:'POST' });

// Poll the live PTZ position (e.g. every 1s) to drive a UI indicator
const r = await fetch(`${BASE}/camera/ptz/position`);
const { position } = await r.json();   // { pan, tilt, zoom, ... }
```

### Water pump

| Method | Endpoint | Success response |
|--------|----------|------------------|
| POST | `/pump/on` | `{"status":"pump_on"}` |
| POST | `/pump/off` | `{"status":"pump_off"}` |
| GET  | `/pump/status` | `{"is_on": true}` |

> Ultrasonic has **no HTTP endpoint**. It is read internally from the ESP32
> over USB serial and used by autonomous obstacle avoidance.

### AI detection (one-shot)

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/ai/detect` | Runs YOLO on the latest frame once |

```json
{
  "status": "ok",
  "count": 2,
  "detections": [
    { "class": "potted plant", "confidence": 0.87,
      "box": [120.5, 45.2, 300.1, 280.7], "id": 3 }
  ]
}
```

`box` is `[x1, y1, x2, y2]` in pixels. `id` is the ByteTrack track ID, or
`null` if the track is too young / tracking disabled. Returns
`503` if the model failed to load or no camera frame is available.

> For continuous detection in a UI, **don't poll `/ai/detect`** — use
> `/camera/feed?mode=automatic`, which streams frames with boxes already
> drawn and runs the full navigator.

---

## 🛑 Shutdown

`Ctrl+C` triggers a graceful shutdown: stop navigator (joins the nav +
inference threads) → stop motors → pump off → stop camera thread → re-center
& close PTZ → release GPIO → kill Cloudflare tunnel (if running).

---

## 📦 Dependencies

- **fastapi** / **uvicorn** – web framework + ASGI server
- **opencv-python-headless** – camera capture & image processing
- **ultralytics** – YOLO detection + ByteTrack
- **torch** / **torchvision** – MobileNet-V3 backbone for appearance Re-ID
  (transitive via ultralytics; Re-ID self-disables if missing)
- **gpiozero** / **lgpio** – Raspberry Pi GPIO
- **requests** – HTTP client for AXIS VAPIX PTZ
- **python-dotenv** – loads `components/.env`

---

## 🌐 Cloudflare Tunnel (optional remote access)

`./start.sh` prompts (15 s, defaults to No) to launch a
[Cloudflare Tunnel](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/)
and prints a shareable public URL.

```bash
# Install cloudflared on the Pi (arm64)
curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 \
  -o /usr/local/bin/cloudflared
chmod +x /usr/local/bin/cloudflared
```
 
