#!/usr/bin/env python3
"""
dashboard.py  —  MQTT subscriber + HTTP server for robot dashboard
Listens to robot data via MQTT and serves a live HTML page.

Usage:
    python3 dashboard.py
    Then open http://localhost:8080 in your browser.
"""

import json
import threading
import time
import http.server
import urllib.parse
import paho.mqtt.client as mqtt

# ── Config ──────────────────────────────────────────────────────────────────────
MQTT_HOST = "192.168.0.74"
MQTT_PORT = 1883
GROUP_ID = "3"
ROBOT_ID = "Terminator"       # Must match BoardId in final_code.ino
HTTP_PORT = 8080

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
    print(f"[MQTT] Connected (rc={reason_code})")
    # Subscribe to all messages for our group
    client.subscribe(f"group/{GROUP_ID}/broadcast")
    client.subscribe(f"group/{GROUP_ID}/board/{ROBOT_ID}")

def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    topic = msg.topic
    # print(f"[MQTT] {topic}: {payload}")

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
        if len(parts) >= 7:
            robot_state["centroid"] = int(parts[0])
            robot_state["ir"][0] = int(parts[1])
            robot_state["ir"][8] = int(parts[2])
            robot_state["uds"] = [int(parts[3]), int(parts[4]), int(parts[5])]
            robot_state["heading"] = float(parts[6])

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
        print(f"[LOG] {msg_text}")
        robot_state["log"].append(msg_text)
        if len(robot_state["log"]) > 50:
            robot_state["log"] = robot_state["log"][-50:]

    # Push update to all SSE clients
    notify_clients()

def on_disconnect(client, userdata, reason_code, properties=None):
    print("[MQTT] Disconnected — will reconnect")

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
        except Exception as e:
            print(f"[MQTT] Error: {e}, reconnecting in 5s")
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
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

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
                # Send via MQTT to robot
                mqtt_client.publish(f"group/{GROUP_ID}/board/{ROBOT_ID}", msg)
                print(f"[CMD] Sent: {msg}")
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b"ok")

        elif path == "/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(robot_state).encode("utf-8"))

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass    # suppress HTTP log spam

# ── HTML Page ───────────────────────────────────────────────────────────────────
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Robot Dashboard</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
  h1 { color: #e94560; margin-bottom: 16px; font-size: 20px; }
  h2 { color: #0f3460; font-size: 14px; margin: 10px 0 6px; border-bottom: 1px solid #333; padding-bottom: 4px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 1200px; }
  .card { background: #16213e; border-radius: 8px; padding: 14px; border: 1px solid #0f3460; }
  .card table { width: 100%; border-collapse: collapse; font-size: 13px; }
  .card td, .card th { padding: 3px 6px; text-align: left; border-bottom: 1px solid #222; }
  .card th { color: #aaa; font-weight: 400; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 11px; font-weight: 600; }
  .badge-green { background: #1b8a3f; color: #fff; }
  .badge-red { background: #a83232; color: #fff; }
  .badge-yellow { background: #a88c32; color: #fff; }
  .btn { padding: 6px 14px; border: none; border-radius: 4px; cursor: pointer; font-size: 12px; margin: 2px; }
  .btn-primary { background: #e94560; color: #fff; }
  .btn-secondary { background: #0f3460; color: #eee; }
  .btn-small { padding: 3px 8px; font-size: 11px; }
  .map-box { position: relative; width: 100%; aspect-ratio: 1; background: #0d1b2a; border: 1px solid #333; border-radius: 4px; overflow: hidden; }
  .ir-bar { display: flex; gap: 2px; height: 40px; align-items: flex-end; margin: 6px 0; }
  .ir-seg { flex: 1; background: #e94560; min-height: 2px; border-radius: 2px 2px 0 0; transition: height 0.15s; }
  .ir-seg.active { background: #16c79a; }
  .log-box { height: 120px; overflow-y: auto; font-family: monospace; font-size: 11px; color: #aaa; }
  .ctrl-row { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; margin: 6px 0; }
  .ctrl-row input { width: 60px; background: #0d1b2a; border: 1px solid #333; color: #eee; padding: 3px 6px; border-radius: 3px; font-size: 12px; }
  .ctrl-row label { font-size: 12px; color: #aaa; }
  .full { grid-column: 1 / -1; }
  @media (max-width: 800px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<h1>&#x1F916; Robot Dashboard — Team 3</h1>
<div class="grid" id="app">
  <!-- State -->
  <div class="card">
    <h2>Status</h2>
    <table>
      <tr><th>State</th><td><span id="state" class="badge badge-yellow">--</span></td></tr>
      <tr><th>Position</th><td id="pos">--,--</td></tr>
      <tr><th>Heading</th><td id="heading">--&deg;</td></tr>
      <tr><th>Centroid</th><td id="centroid">--</td></tr>
      <tr><th>UDS</th><td id="uds">-- / -- / --</td></tr>
    </table>
    <div class="ctrl-row" style="margin-top:10px">
      <button class="btn btn-primary" onclick="sendCmd('ENABLE')">ENABLE</button>
      <button class="btn btn-secondary" onclick="sendCmd('DISABLE')">DISABLE</button>
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
    <h2>IR Array <span id="irLabel" style="font-weight:400;font-size:11px;color:#aaa;"></span></h2>
    <div class="ir-bar" id="irBar"></div>
  </div>

  <!-- Hole Status -->
  <div class="card">
    <h2>Holes (9&times;9)</h2>
    <div id="holeGrid" style="font-size:10px;font-family:monospace;line-height:1.4;color:#aaa;"></div>
  </div>

  <!-- PID Tuning -->
  <div class="card full">
    <h2>PID Tuning &amp; Tests</h2>
    <div class="ctrl-row">
      <label>kp:</label><input id="kp" value="0.5" onchange="sendPid('kp',this.value)">
      <label>ki:</label><input id="ki" value="0.0" onchange="sendPid('ki',this.value)">
      <label>kd:</label><input id="kd" value="0.0" onchange="sendPid('kd',this.value)">
      <label>md:</label><input id="md" value="40" onchange="sendPid('md',this.value)">
    </div>
    <div class="ctrl-row">
      <span style="font-size:12px;color:#aaa;">Tests:</span>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:FOLLOW_LINE:500,'+getPidStr())">Follow Line</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:FOLLOW_WALL:500,1,'+getWallPidStr())">Follow Wall</button>
      <button class="btn btn-small btn-secondary" onclick="sendCmd('TEST:DEPOSIT')">Deposit</button>
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

function sendCmd(msg) {
  fetch('/command?msg=' + encodeURIComponent(msg));
}

function sendPid(key, val) {
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

  document.getElementById('pos').textContent = (d.pose.x|0) + ',' + (d.pose.y|0);
  document.getElementById('heading').textContent = d.pose.heading.toFixed(1) + '\u00b0';
  document.getElementById('centroid').textContent = d.centroid;
  document.getElementById('uds').textContent = d.uds.join(' / ') + ' cm';

  // IR bar
  const bar = document.getElementById('irBar');
  // Only have first and last IR val from snapshot — draw what we have
  bar.innerHTML = '';
  const vals = d.ir || [];
  for (let i = 0; i < 9; i++) {
    const seg = document.createElement('div');
    seg.className = 'ir-seg' + (d.centroid >= 0 && Math.abs(i*1000 - d.centroid) < 500 ? ' active' : '');
    // Estimate intermediate values
    let v = vals[i] || 0;
    seg.style.height = Math.max(4, (v / 1000) * 100) + '%';
    seg.title = 'S' + i + ': ' + v;
    bar.appendChild(seg);
  }
  document.getElementById('irLabel').textContent = 'centroid=' + d.centroid;

  // Hole grid
  const hg = document.getElementById('holeGrid');
  let html = '';
  for (let r = 0; r < 9; r++) {
    for (let c = 0; c < 9; c++) {
      const key = r + ',' + c;
      const h = d.holes[key];
      const ch = h ? (h.planted ? '\u2714' : h.fertile ? '\u25CB' : '\u2718') : '\u00B7';
      html += ch + ' ';
    }
    html += '<br>';
  }
  hg.innerHTML = html;

  // Map canvas
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * 2;
  canvas.height = rect.height * 2;
  ctx.scale(2, 2);
  const w = rect.width, h = rect.height;
  ctx.fillStyle = '#0d1b2a';
  ctx.fillRect(0, 0, w, h);

  // Draw robot
  const cx = w * 0.5 + d.pose.x * 0.05;
  const cy = h * 0.5 - d.pose.y * 0.05;
  const headRad = d.pose.heading * Math.PI / 180;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-headRad + Math.PI/2);
  ctx.fillStyle = '#e94560';
  ctx.beginPath();
  ctx.arc(0, 0, 8, 0, Math.PI*2);
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, -12);
  ctx.lineTo(0, 12);
  ctx.stroke();
  ctx.restore();

  // Draw UDS cones
  const udsAngles = [-32, 0, 32];
  const udsColors = ['#16c79a', '#16c79a', '#16c79a'];
  const udsRanges = [d.uds[0], d.uds[1], d.uds[2]];
  for (let i = 0; i < 3; i++) {
    const ang = (d.pose.heading + udsAngles[i]) * Math.PI / 180;
    const range = Math.min(udsRanges[i] * 0.8, 60);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(Math.PI/2 - ang);
    ctx.fillStyle = udsColors[i] + '44';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, range, -0.17, 0.17);
    ctx.fill();
    ctx.restore();
  }

  // Hole grid
  for (let r = 0; r < 9; r++) {
    for (let c = 0; c < 9; c++) {
      const hx = w * 0.5 + (c - 4) * 14;
      const hy = h * 0.5 - (r - 4) * 14;
      const key = r + ',' + c;
      const hole = d.holes[key];
      ctx.fillStyle = hole ? (hole.planted ? '#1b8a3f' : hole.fertile ? '#a88c32' : '#555') : '#333';
      ctx.beginPath();
      ctx.arc(hx, hy, 3, 0, Math.PI*2);
      ctx.fill();
    }
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
    print(f"Dashboard starting — MQTT: {MQTT_HOST}:{MQTT_PORT}  HTTP: :{HTTP_PORT}")
    threading.Thread(target=mqtt_thread, daemon=True).start()
    time.sleep(0.5)

    port = HTTP_PORT
    while port < HTTP_PORT + 10:
        try:
            server = http.server.HTTPServer(("0.0.0.0", port), DashboardHandler)
            break
        except OSError:
            port += 1
    if port != HTTP_PORT:
        print(f"Port {HTTP_PORT} in use — using port {port}")
    print(f"Open http://localhost:{port} in your browser")
    server.serve_forever()
