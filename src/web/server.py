"""
Web dashboard for drone-vio-bridge.
Opens on http://<rpi-ip>:5000

Shows live:
  - VIO position (X/Y/Z)
  - Attitude (roll/pitch/yaw)
  - Quality indicator
  - Bridge connection status
  - CPU + memory usage
  - Log tail
"""

import json
import logging
import os
import threading
import time
from collections import deque
from typing import Optional

log = logging.getLogger(__name__)

try:
    from flask import Flask, jsonify, render_template_string
    _FLASK_AVAILABLE = True
except ImportError:
    _FLASK_AVAILABLE = False


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>VIO Bridge</title>
<style>
  :root { --bg:#0d1117; --card:#161b22; --border:#30363d;
          --text:#c9d1d9; --accent:#58a6ff; --ok:#3fb950;
          --warn:#d29922; --err:#f85149; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text); font:14px/1.5 'Segoe UI',monospace; }
  h1 { padding:14px 20px; font-size:18px; border-bottom:1px solid var(--border);
       color:var(--accent); display:flex; align-items:center; gap:10px; }
  h1 span { font-size:12px; color:var(--text); opacity:.6; }
  .grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(220px,1fr));
          gap:12px; padding:16px; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:8px; padding:14px; }
  .card-title { font-size:11px; text-transform:uppercase; letter-spacing:.8px;
                color:var(--accent); margin-bottom:8px; }
  .val { font-size:22px; font-weight:700; font-family:monospace; }
  .unit { font-size:11px; opacity:.5; margin-left:4px; }
  .row { display:flex; justify-content:space-between; margin-top:6px; font-size:13px; }
  .dot { width:9px; height:9px; border-radius:50%; display:inline-block; margin-right:5px; }
  .ok  { background:var(--ok); }
  .warn{ background:var(--warn); }
  .err { background:var(--err); }
  .log { background:#0d1117; border:1px solid var(--border); border-radius:6px;
         padding:10px; font-family:monospace; font-size:12px; height:160px;
         overflow-y:auto; white-space:pre; color:#8b949e; margin:0 16px 16px; }
  #quality-bar { height:6px; border-radius:3px; background:var(--border); margin-top:6px; }
  #quality-fill { height:100%; border-radius:3px; background:var(--ok);
                  transition:width .3s, background .3s; }
</style>
</head>
<body>
<h1>🛸 drone-vio-bridge <span id="uptime">–</span></h1>
<div class="grid">
  <div class="card">
    <div class="card-title">Position (NED)</div>
    <div class="row"><span>North</span><span><b id="px">–</b> m</span></div>
    <div class="row"><span>East</span> <span><b id="py">–</b> m</span></div>
    <div class="row"><span>Down</span> <span><b id="pz">–</b> m</span></div>
  </div>
  <div class="card">
    <div class="card-title">Attitude</div>
    <div class="row"><span>Roll</span> <span><b id="roll">–</b>°</span></div>
    <div class="row"><span>Pitch</span><span><b id="pitch">–</b>°</span></div>
    <div class="row"><span>Yaw</span>  <span><b id="yaw">–</b>°</span></div>
  </div>
  <div class="card">
    <div class="card-title">Velocity</div>
    <div class="row"><span>Vn</span><span><b id="vx">–</b> m/s</span></div>
    <div class="row"><span>Ve</span><span><b id="vy">–</b> m/s</span></div>
    <div class="row"><span>Vd</span><span><b id="vz">–</b> m/s</span></div>
  </div>
  <div class="card">
    <div class="card-title">VIO Quality</div>
    <div class="val"><span id="quality">–</span><span class="unit">%</span></div>
    <div id="quality-bar"><div id="quality-fill" style="width:0%"></div></div>
    <div class="row" style="margin-top:10px"><span>Algorithm</span><span id="algo">–</span></div>
    <div class="row"><span>Rate</span><span><b id="fps">–</b> Hz</span></div>
  </div>
  <div class="card">
    <div class="card-title">Bridges</div>
    <div class="row"><span><span class="dot" id="dot-ap"></span>ArduPilot</span><span id="st-ap">–</span></div>
    <div class="row"><span><span class="dot" id="dot-in"></span>iNav</span>      <span id="st-in">–</span></div>
    <div class="row"><span><span class="dot" id="dot-bf"></span>Betaflight</span><span id="st-bf">–</span></div>
  </div>
  <div class="card">
    <div class="card-title">System</div>
    <div class="row"><span>CPU</span>  <span><b id="cpu">–</b>%</span></div>
    <div class="row"><span>RAM</span>  <span><b id="ram">–</b>%</span></div>
    <div class="row"><span>Temp</span> <span><b id="temp">–</b>°C</span></div>
  </div>
</div>
<div class="log" id="log">Waiting for log…</div>
<script>
const r = n => n.toFixed(2);
const deg = r => (r * 180 / Math.PI).toFixed(1);
function dotClass(ok) { return ok ? 'dot ok' : 'dot err'; }
async function refresh() {
  try {
    const d = await fetch('/api/status').then(r=>r.json());
    const p = d.pose || {};
    document.getElementById('px').textContent   = p.x != null ? r(p.x) : '–';
    document.getElementById('py').textContent   = p.y != null ? r(p.y) : '–';
    document.getElementById('pz').textContent   = p.z != null ? r(p.z) : '–';
    document.getElementById('vx').textContent   = p.vx != null ? r(p.vx) : '–';
    document.getElementById('vy').textContent   = p.vy != null ? r(p.vy) : '–';
    document.getElementById('vz').textContent   = p.vz != null ? r(p.vz) : '–';
    document.getElementById('roll').textContent  = p.roll  != null ? (p.roll*180/Math.PI).toFixed(1) : '–';
    document.getElementById('pitch').textContent = p.pitch != null ? (p.pitch*180/Math.PI).toFixed(1) : '–';
    document.getElementById('yaw').textContent   = p.yaw   != null ? (p.yaw*180/Math.PI).toFixed(1) : '–';
    const q = p.quality != null ? Math.round(p.quality*100) : 0;
    document.getElementById('quality').textContent = q;
    const qf = document.getElementById('quality-fill');
    qf.style.width = q + '%';
    qf.style.background = q > 60 ? '#3fb950' : q > 30 ? '#d29922' : '#f85149';
    document.getElementById('fps').textContent  = d.fps != null ? d.fps.toFixed(1) : '–';
    document.getElementById('algo').textContent = d.algorithm || '–';
    // Bridges
    ['ap','in','bf'].forEach((k,i) => {
      const ok = d.bridges && d.bridges[k];
      document.getElementById('dot-'+k).className = ok ? 'dot ok' : 'dot err';
      document.getElementById('st-'+k).textContent = ok ? 'OK' : 'off';
    });
    // System
    const s = d.system || {};
    document.getElementById('cpu').textContent  = s.cpu  != null ? s.cpu.toFixed(0) : '–';
    document.getElementById('ram').textContent  = s.ram  != null ? s.ram.toFixed(0) : '–';
    document.getElementById('temp').textContent = s.temp != null ? s.temp.toFixed(1) : '–';
    // Uptime
    if (d.uptime != null) {
      const u = Math.floor(d.uptime);
      document.getElementById('uptime').textContent =
        `${Math.floor(u/3600)}h ${Math.floor((u%3600)/60)}m ${u%60}s`;
    }
    // Log
    if (d.log) { const el=document.getElementById('log'); el.textContent=d.log; el.scrollTop=el.scrollHeight; }
  } catch(e) {}
}
refresh();
setInterval(refresh, 200);
</script>
</body>
</html>
"""


class WebDashboard:
    def __init__(self, host: str = "0.0.0.0", port: int = 5000):
        if not _FLASK_AVAILABLE:
            raise RuntimeError("flask not installed. Run: pip install flask")

        self._host = host
        self._port = port
        self._app = Flask(__name__)
        self._state = {
            "pose": None, "fps": 0.0, "algorithm": "–",
            "bridges": {"ap": False, "in": False, "bf": False},
            "system": {}, "uptime": 0.0, "log": "",
        }
        self._lock = threading.Lock()
        self._log_buf: deque = deque(maxlen=30)
        self._start_t = time.time()

        self._app.add_url_rule("/", "index", self._index)
        self._app.add_url_rule("/api/status", "status", self._status)

    def update(self, pose=None, fps=None, bridges=None, algorithm=None):
        with self._lock:
            if pose is not None:
                self._state["pose"] = {
                    "x": pose.x, "y": pose.y, "z": pose.z,
                    "vx": pose.vx, "vy": pose.vy, "vz": pose.vz,
                    "roll": pose.roll, "pitch": pose.pitch, "yaw": pose.yaw,
                    "quality": pose.quality,
                }
            if fps is not None:
                self._state["fps"] = fps
            if bridges is not None:
                self._state["bridges"].update(bridges)
            if algorithm is not None:
                self._state["algorithm"] = algorithm
            self._state["uptime"] = time.time() - self._start_t
            self._state["system"] = self._system_stats()
            self._state["log"] = "\n".join(self._log_buf)

    def push_log(self, line: str):
        self._log_buf.append(line)

    def start(self):
        t = threading.Thread(
            target=lambda: self._app.run(host=self._host, port=self._port,
                                          debug=False, use_reloader=False),
            daemon=True, name="web-dashboard"
        )
        t.start()
        log.info("Web dashboard: http://%s:%d", self._host, self._port)

    def _index(self):
        return render_template_string(_HTML)

    def _status(self):
        with self._lock:
            return jsonify(dict(self._state))

    @staticmethod
    def _system_stats() -> dict:
        result = {}
        try:
            import psutil
            result["cpu"]  = psutil.cpu_percent()
            result["ram"]  = psutil.virtual_memory().percent
        except ImportError:
            pass
        # RPi CPU temperature
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                result["temp"] = int(f.read().strip()) / 1000.0
        except OSError:
            pass
        return result
