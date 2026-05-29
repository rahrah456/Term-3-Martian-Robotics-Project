#!/usr/bin/env python3
"""
dashboard.py  —  MQTT subscriber + HTTP server for robot dashboard
Listens to robot data via MQTT and serves a live HTML page.

Usage:
    pip install paho-mqtt (install the Paho MQTT client library)
    python3 dashboard.py
    Then open http://localhost:8081 in your browser.
"""

import json
import threading
import time
import http.server
import socketserver
import urllib.parse
import paho.mqtt.client as mqtt

# ── Config ──────────────────────────────────────────────────────────────────────
MQTT_HOST = "192.168.0.74"
MQTT_PORT = 1883
GROUP_ID = "3"
ROBOT_ID = "Haunter"
DASHBOARD_ID = "dash3"          # Must match DASHBOARD_ID in secrets.h
HTTP_PORT = 8081

# ── State ───────────────────────────────────────────────────────────────────────
robot_state = {
    "pose": {"x": 0, "y": 0, "heading": 0},
    "state": "unknown",
    "ir": [0] * 9,
    "centroid": -1,
    "uds": [0, 0, 0],
    "heading": 0.0,
    "holes": {},
    "log": []
}

sse_clients = []      # list of queue.Queue for SSE push
sse_lock = threading.Lock()

# ── MQTT Callbacks ──────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, reason_code, properties):
    pass
    # Subscribe to all messages for our group (MiniMessenger topic format)
    client.subscribe(f"lab/g/{GROUP_ID}/from/{ROBOT_ID}/to/{DASHBOARD_ID}")

def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8")

    # Parse our message format
    if payload.startswith("POSE:"):
        parts = payload[5:].split(",")
        if len(parts) >= 3:
            robot_state["pose"]["x"] = float(parts[0])
            robot_state["pose"]["y"] = float(parts[1])
            robot_state["pose"]["heading"] = float(parts[2])

    elif payload.startswith("STATE:"):
        robot_state["state"] = payload[6:]

    elif payload.startswith("SENSOR:"):
        parts = payload[7:].split(",")
        if len(parts) >= 15:
            robot_state["centroid"] = int(parts[0])
            for i in range(9):
                robot_state["ir"][i] = int(parts[1 + i])
            robot_state["uds"] = [int(parts[10]), int(parts[11]), int(parts[12])]
            robot_state["heading"] = float(parts[13])

    elif payload.startswith("HOLE:"):
        parts = payload[5:].split(",")
        if len(parts) >= 4:
            key = f"{parts[0]},{parts[1]}"
            robot_state["holes"][key] = {
                "planted": parts[2] == "1",
                "fertile": parts[3] == "1"
            }

    elif payload.startswith("LOG:"):
        msg_text = payload[4:]
        robot_state["log"].append(msg_text)
        if len(robot_state["log"]) > 50:
            robot_state["log"] = robot_state["log"][-50:]

    # Push update to all SSE clients
    notify_clients()

def on_disconnect(client, userdata, reason_code, properties=None):
    pass

# ── MQTT Client Setup ───────────────────────────────────────────────────────────
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.on_disconnect = on_disconnect

def mqtt_thread():
    while True:
        try:
            mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
            mqtt_client.loop_forever()
        except Exception:
            time.sleep(5)

# ── SSE ─────────────────────────────────────────────────────────────────────────
def notify_clients():
    data = json.dumps(robot_state)
    with sse_lock:
        for q in sse_clients[:]:
            try:
                q.put_nowait(data)
            except:
                sse_clients.remove(q)

# ── HTTP Server ─────────────────────────────────────────────────────────────────
class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def safe_write(self, data):
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.safe_write(HTML_PAGE.encode("utf-8"))

        elif path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            import queue
            q = queue.Queue()
            with sse_lock:
                sse_clients.append(q)
            try:
                while True:
                    data = q.get()
                    self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with sse_lock:
                    if q in sse_clients:
                        sse_clients.remove(q)

        elif path == "/command":
            msg = query.get("msg", [""])[0]
            if msg:
                topic = f"lab/g/{GROUP_ID}/from/{DASHBOARD_ID}/to/{ROBOT_ID}"
                mqtt_client.publish(topic, msg)
                topic2 = f"group/{GROUP_ID}/board/{ROBOT_ID}"
                mqtt_client.publish(topic2, msg)
                # Local feedback
                robot_state["log"].append(f"CMD: {msg}")
                if len(robot_state["log"]) > 50:
                    robot_state["log"] = robot_state["log"][-50:]
                notify_clients()
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.safe_write(b"ok")

        elif path == "/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.safe_write(json.dumps(robot_state).encode("utf-8"))

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

# ── HTML Page ───────────────────────────────────────────────────────────────────
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Dashboard - Team 3</title>
<style>
  :root {
    color-scheme: light;
    --bg: #f6f7f9;
    --surface: #ffffff;
    --surface-strong: #f0f3f5;
    --ink: #172026;
    --muted: #63707a;
    --line: #d7dde2;
    --blue: #246b9f;
    --green: #317456;
    --green-soft: #dcefe5;
    --amber: #a35c00;
    --amber-soft: #fff0d6;
    --red: #b42318;
    --red-soft: #ffe3df;
    --shadow: 0 12px 30px rgba(23, 32, 38, 0.08);
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: "Segoe UI", system-ui, -apple-system, BlinkMacSystemFont, sans-serif; background: var(--bg); color: var(--ink); padding: 20px; }
  .app-header { display: flex; align-items: center; justify-content: space-between; gap: 20px; margin: -20px -20px 18px; padding: 22px 28px; border-bottom: 1px solid var(--line); background: var(--surface); }
  .app-header h1 { font-size: 28px; line-height: 1.1; margin: 0; }
  .app-header .eyebrow { margin: 0 0 4px; color: var(--muted); font-size: 12px; font-weight: 700; letter-spacing: 0; text-transform: uppercase; }
  .status-pill { display: inline-block; min-width: 124px; border: 1px solid var(--line); border-radius: 999px; padding: 8px 12px; background: var(--surface-strong); color: var(--muted); font-size: 14px; font-weight: 600; text-align: center; }
  .status-pill.online { border-color: #9bd0b2; background: var(--green-soft); color: var(--green); }
  .status-pill.offline, .status-pill.emergency { border-color: #ffb6ad; background: var(--red-soft); color: var(--red); }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 1480px; }
  .card { background: var(--surface); border: 1px solid var(--line); border-radius: 8px; padding: 18px; box-shadow: var(--shadow); }
  .card h2 { font-size: 14px; font-weight: 600; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 0.3px; color: var(--muted); }
  .card table { width: 100%; border-collapse: collapse; font-size: 14px; }
  .card td, .card th { padding: 6px 8px; text-align: left; border-bottom: 1px solid var(--line); }
  .card th { color: var(--muted); font-weight: 500; width: 80px; }
  .badge { display: inline-block; padding: 3px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; border: 1px solid transparent; }
  .badge-green { background: var(--green-soft); color: var(--green); border-color: #9bd0b2; }
  .badge-red { background: var(--red-soft); color: var(--red); border-color: #ffb6ad; }
  .badge-yellow { background: var(--amber-soft); color: var(--amber); border-color: #efc36f; }
  .btn { display: inline-flex; align-items: center; justify-content: center; min-height: 36px; border: 1px solid transparent; border-radius: 6px; padding: 0 14px; font: inherit; font-weight: 600; cursor: pointer; font-size: 13px; }
  .btn-primary { background: var(--blue); color: #fff; }
  .btn-secondary { border-color: var(--line); background: var(--surface); color: var(--ink); }
  .btn-danger { background: var(--red); color: #fff; }
  .btn-small { min-height: 28px; padding: 0 10px; font-size: 12px; }
  .map-box { position: relative; width: 100%; aspect-ratio: 1; background: #fbfcfd; border: 1px solid var(--line); border-radius: 6px; overflow: hidden; }
  .ir-bar { display: flex; gap: 2px; height: 48px; align-items: flex-end; margin: 8px 0; }
  .ir-seg { flex: 1; background: var(--blue); min-height: 2px; border-radius: 2px 2px 0 0; transition: height 0.15s; opacity: 0.7; }
  .ir-seg.active { background: var(--green); opacity: 1; }
  .log-box { height: 150px; overflow-y: auto; font-family: "Cascadia Code", "Fira Code", "Consolas", monospace; font-size: 12px; color: var(--muted); background: #fbfcfd; border: 1px solid var(--line); border-radius: 6px; padding: 8px; }
  .ctrl-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin: 8px 0; }
  .ctrl-row input[type="text"] { flex: 1; min-width: 120px; border: 1px solid var(--line); border-radius: 6px; padding: 8px 12px; background: var(--surface-strong); font: inherit; font-size: 13px; color: var(--ink); }
  .ctrl-row input[type="text"]:focus { outline: none; border-color: var(--blue); }
  .ctrl-row input.num { width: 60px; border: 1px solid var(--line); border-radius: 4px; padding: 4px 6px; background: var(--surface-strong); font: inherit; font-size: 12px; color: var(--ink); }
  .ctrl-row label { font-size: 12px; color: var(--muted); font-weight: 500; }
  .full { grid-column: 1 / -1; }
  .hole-grid { display: grid; grid-template-columns: repeat(9, 1fr); gap: 1px; border: 1px solid var(--line); background: var(--line); max-width: 320px; }
  .hole-cell { display: flex; align-items: center; justify-content: center; min-height: 28px; background: #fbfcfd; font-size: 13px; font-family: "Segoe UI", sans-serif; color: var(--muted); }
  .hole-cell.planted { background: var(--green-soft); color: var(--green); }
  .hole-cell.fertile { background: var(--amber-soft); color: var(--amber); }
  .hole-cell.empty { background: #eef1f3; color: #bbb; }
  .cheatsheet { display: none; background: #fbfcfd; border: 1px solid var(--line); border-radius: 6px; padding: 10px 14px; font-size: 12px; color: var(--muted); line-height: 1.8; margin-bottom: 8px; }
  .cheatsheet b { color: var(--ink); }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } .app-header { flex-direction: column; align-items: stretch; gap: 12px; padding: 18px; } }
</style>
</head>
<body>
<div class="app-header">
  <div>
    <div class="eyebrow">Team 3 &middot; martian robotics</div>
    <h1>Robot Dashboard</h1>
  </div>
  <div class="status-pill" id="statePill">Disconnected</div>
</div>
<div class="grid" id="app">
  <!-- State -->
  <div class="card">
    <h2>Status</h2>
    <table>
      <tr><th>State</th><td><span id="state" class="badge badge-yellow">--</span></td></tr>
      <tr><th>Position</th><td id="pos">--, --</td></tr>
      <tr><th>Heading</th><td id="heading">--&deg;</td></tr>
      <tr><th>Centroid</th><td id="centroid">--</td></tr>
      <tr><th>UDS</th><td id="uds">-- / -- / --</td></tr>
    </table>
    <div class="ctrl-row">
      <button class="btn btn-primary" onclick="sendCmd('ENABLE')">ENABLE</button>
      <button class="btn btn-secondary" onclick="sendCmd('DISABLE')">DISABLE</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('HEADING:0')" style="margin-top:4px;">Reset Heading</button>
    </div>
  </div>

  <!-- Map -->
  <div class="card">
    <h2>Map</h2>
    <div class="map-box" id="map">
      <canvas id="mapCanvas" style="width:100%;height:100%"></canvas>
    </div>
  </div>

  <!-- IR Sensors -->
  <div class="card">
    <h2>IR Array</h2>
    <div class="ir-wrapper" style="position:relative;">
      <div class="ir-bar" id="irBar"></div>
    </div>
    <div style="font-size:12px;color:var(--muted);">centroid: <span id="irLabel">--</span></div>
  </div>

  <!-- Hole Status -->
  <div class="card">
    <h2>Holes (9&times;9)</h2>
    <div id="holeGrid" class="hole-grid"></div>
  </div>

  <!-- PID Tuning -->
  <div class="card full">
    <h2>PID Tuning &amp; Tests</h2>
    <div class="ctrl-row">
      <input id="consoleInput" type="text" placeholder="Send any command&hellip;"
             onkeydown="if(event.key==='Enter'){sendCmd(this.value);this.value=''}">
      <button class="btn btn-small btn-secondary" onclick="sendCmd(consoleInput.value);consoleInput.value=''">Send</button>
      <span style="font-size:14px;color:var(--muted);cursor:pointer;font-weight:600;" onclick="toggleCheat()">?</span>
    </div>
    <div class="cheatsheet" id="cheat">
      <b>Console commands:</b><br>
      ENABLE &nbsp; DISABLE &nbsp; DEPOSIT &nbsp; EXIT_BASE<br>
      TEST:FOLLOW_LINE:&lt;base&gt;,&lt;kp&gt;,&lt;ki&gt;,&lt;kd&gt;,&lt;md&gt;<br>
      TEST:FOLLOW_WALL:&lt;base&gt;,&lt;side&gt;,&lt;targetCm&gt;,&lt;kp&gt;,&lt;ki&gt;,&lt;kd&gt;,&lt;md&gt;<br>
      &lt;key&gt;:&lt;val&gt; &nbsp; (kp, ki, kd, md)
    </div>
    <div class="ctrl-row">
      <label>kp:</label><input id="kp" value="0.5" class="num" onchange="sendPid('kp',this.value)">
      <label>ki:</label><input id="ki" value="0.0" class="num" onchange="sendPid('ki',this.value)">
      <label>kd:</label><input id="kd" value="0.0" class="num" onchange="sendPid('kd',this.value)">
      <label>md:</label><input id="md" value="40" class="num" onchange="sendPid('md',this.value)">
    </div>
    <div class="ctrl-row">
      <span style="font-size:12px;color:var(--muted);font-weight:500;">Tests:</span>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:FOLLOW_LINE:500,'+getPidStr())">Follow Line</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:FOLLOW_WALL:500,1,8.0,'+getWallPidStr())">Follow Wall (R, 8cm)</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:DEPOSIT')">Deposit</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:EXIT_BASE')">Exit Base</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:REVIVE')">Revive</button>
    </div>
  </div>

  <!-- Log -->
  <div class="card full">
    <h2>Log</h2>
    <div class="log-box" id="logBox"></div>
  </div>
</div>

<script>
function getPidStr() {
  return document.getElementById('kp').value + ',' +
         document.getElementById('ki').value + ',' +
         document.getElementById('kd').value + ',' +
         document.getElementById('md').value;
}
function getWallPidStr() {
  return document.getElementById('kp').value + ',0,' +
         document.getElementById('kd').value + ',' +
         document.getElementById('md').value;
}
function toggleCheat() {
  const e = document.getElementById('cheat');
  e.style.display = e.style.display === 'none' ? 'block' : 'none';
}

function sendCmd(msg) {
  console.log('sendCmd:', msg);
  fetch('/command?msg=' + encodeURIComponent(msg))
    .then(function(r) { console.log('sendCmd response:', r.status); })
    .catch(function(e) { console.error('sendCmd error:', e); });
}

function sendPid(key, val) {
  console.log('sendPid:', key, val);
  fetch('/command?msg=' + key + ':' + val);
}

// SSE connection
const evtSource = new EventSource('/events');
evtSource.onmessage = function(e) {
  try {
    const d = JSON.parse(e.data);
    update(d);
  } catch(_) {}
};

let lastState = null;
function update(d) {
  lastState = d;

  // State badge
  const st = document.getElementById('state');
  st.textContent = d.state || '--';
  st.className = 'badge ' + ((d.state === 'NAVIGATE' || d.state === 'DEPOSIT') ? 'badge-green' :
                             d.state === 'INIT' || d.state === 'IDLE' ? 'badge-yellow' : 'badge-red');

  // Header state pill
  const pill = document.getElementById('statePill');
  if (d.state && d.state !== 'unknown') {
    pill.textContent = d.state;
    pill.className = 'status-pill online';
  } else {
    pill.textContent = 'Disconnected';
    pill.className = 'status-pill offline';
  }

  document.getElementById('pos').textContent = (d.pose.x|0) + ', ' + (d.pose.y|0);
  document.getElementById('heading').textContent = d.pose.heading.toFixed(1) + '\u00b0';
  document.getElementById('centroid').textContent = d.centroid;
  document.getElementById('uds').textContent = d.uds.join(' / ') + ' cm';

  // IR bar
  const bar = document.getElementById('irBar');
  const wrapper = bar.parentElement;
  bar.innerHTML = '';
  const vals = d.ir || [];
  let maxV = -1, maxI = -1;
  for (let i = 0; i < 9; i++) {
    let v = vals[i] || 0;
    if (v > maxV) { maxV = v; maxI = i; }
    const seg = document.createElement('div');
    seg.className = 'ir-seg';
    seg.style.height = Math.max(4, (v / 1000) * 100) + '%';
    seg.title = 'S' + i + ': ' + v;
    bar.appendChild(seg);
  }
  // Highlight the strongest sensor, or centroid-nearest if centroid is valid
  let activeIdx = maxI;
  if (d.centroid >= 0) {
    activeIdx = Math.round(d.centroid / 1000);
    if (activeIdx < 0) activeIdx = 0;
    if (activeIdx > 8) activeIdx = 8;
  }
  if (activeIdx >= 0) {
    bar.children[activeIdx].classList.add('active');
  }
  // Centroid caret (▼) at the interpolated position inside the wrapper
  let caret = document.getElementById('irCaret');
  if (!caret) {
    caret = document.createElement('div');
    caret.id = 'irCaret';
    caret.style.cssText = 'position:absolute;top:-16px;left:0;font-size:14px;line-height:1;pointer-events:none;';
    wrapper.appendChild(caret);
  }
  if (d.centroid >= 0) {
    const pct = (d.centroid / 8000) * 100;
    caret.textContent = '\u25bc';
    caret.style.left = pct + '%';
  } else {
    caret.textContent = '';
  }
  document.getElementById('irLabel').textContent = d.centroid >= 0 ? d.centroid : '--';

  // Hole grid
  const hg = document.getElementById('holeGrid');
  hg.innerHTML = '';
  for (let r = 0; r < 9; r++) {
    for (let c = 0; c < 9; c++) {
      const cell = document.createElement('div');
      cell.className = 'hole-cell';
      const key = r + ',' + c;
      const h = d.holes[key];
      if (h) {
        if (h.planted) { cell.textContent = '\u2714'; cell.className = 'hole-cell planted'; }
        else if (h.fertile) { cell.textContent = '\u25CB'; cell.className = 'hole-cell fertile'; }
        else { cell.textContent = '\u2718'; cell.className = 'hole-cell empty'; }
      } else {
        cell.textContent = '\u00B7';
        cell.className = 'hole-cell empty';
      }
      cell.title = 'Row ' + r + ', Col ' + c;
      hg.appendChild(cell);
    }
  }

  // Map canvas
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * 2;
  canvas.height = rect.height * 2;
  ctx.scale(2, 2);
  const w = rect.width, h = rect.height;
  ctx.fillStyle = '#fbfcfd';
  ctx.fillRect(0, 0, w, h);

  // Map scale: 0.15 px per mm
  const S = 0.15;
  const cx = w * 0.5 + d.pose.x * S;
  const cy = h * 0.5 - d.pose.y * S;
  const headRad = d.pose.heading * Math.PI / 180;

  // ── Hole grid (250mm spacing) ──────────────────────────────
  const holeSpacing = 250 * S;
  ctx.strokeStyle = '#e8ecf0';
  ctx.lineWidth = 0.5;
  for (let r = 0; r < 9; r++) {
    for (let c = 0; c < 9; c++) {
      const gx = w * 0.5 + (c - 4) * holeSpacing;
      const gy = h * 0.5 - (r - 4) * holeSpacing;
      const key = r + ',' + c;
      const hole = d.holes[key];
      ctx.fillStyle = hole ? (hole.planted ? '#317456' : hole.fertile ? '#a35c00' : '#63707a') : '#d7dde2';
      ctx.beginPath();
      ctx.arc(gx, gy, 4, 0, Math.PI*2);
      ctx.fill();
    }
  }

  // ── Draw robot (rectangle) ─────────────────────────────────
  const robW = 115 * S, robH = 170 * S;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(headRad);
  // Chassis
  ctx.fillStyle = '#246b9f';
  ctx.fillRect(-robW / 2, -robH / 2, robW, robH);
  ctx.strokeStyle = '#172026';
  ctx.lineWidth = 1;
  ctx.strokeRect(-robW / 2, -robH / 2, robW, robH);
  // Heading indicator (arrow)
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.moveTo(0, -robH / 2 - 4);
  ctx.lineTo(-4, -robH / 2 + 2);
  ctx.lineTo(4, -robH / 2 + 2);
  ctx.fill();
  ctx.restore();

  // ── UDS positions and cones ──────────────────────────────
  // Side UDS: 50mm behind, 66mm sideways, 32° yaw
  // Front (mid) UDS: 85mm forward, centre
  const udsPos = [
    { sx: -66 * S, sy: -50 * S, yaw: -32 },
    { sx:   0,      sy:  85 * S, yaw: 0 },
    { sx:  66 * S,  sy: -50 * S, yaw: 32 }
  ];
  const udsColors = ['#317456', '#317456', '#317456'];
  const udsRanges = [d.uds[0], d.uds[1], d.uds[2]];
  for (let i = 0; i < 3; i++) {
    const ux = cx + udsPos[i].sx * Math.cos(headRad) + udsPos[i].sy * Math.sin(headRad);
    const uy = cy + udsPos[i].sx * Math.sin(headRad) - udsPos[i].sy * Math.cos(headRad);
    const range = Math.min(udsRanges[i], 100) * 0.8;
    ctx.save();
    ctx.translate(ux, uy);
    ctx.rotate(headRad + udsPos[i].yaw * Math.PI / 180 - Math.PI / 2);
    ctx.fillStyle = udsColors[i] + '44';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, range, -0.17, 0.17);
    ctx.fill();
    ctx.restore();
    // UDS dot
    ctx.fillStyle = '#317456';
    ctx.beginPath();
    ctx.arc(ux, uy, 2, 0, Math.PI * 2);
    ctx.fill();
  }

  // Log
  const logBox = document.getElementById('logBox');
  if (d.log && d.log.length) {
    logBox.innerHTML = d.log.slice(-20).join('<br>');
    logBox.scrollTop = logBox.scrollHeight;
  }
}

// Initial load
fetch('/state').then(r => r.json()).then(d => update(d));
</script>
</body>
</html>
"""

# ── Main ────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=mqtt_thread, daemon=True).start()
    time.sleep(0.5)

    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    server = ThreadedHTTPServer(("0.0.0.0", HTTP_PORT), DashboardHandler)
    server.serve_forever()
