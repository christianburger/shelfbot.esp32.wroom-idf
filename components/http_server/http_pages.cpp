// http_pages.cpp
//
// All page content lives here as string literals placed in flash (rodata).
// No heap allocations are performed.  The server sends these directly.
//
// Editing guide
// -------------
// • Root page  : edit kPageRoot[]
// • Motor page : edit kPageMotorPrefix[] (everything before {{FW_VER}}) and
//                      kPageMotorSuffix[] (everything after {{FW_VER}})
// • lidar.html / lidar_viz.js are embedded as binary via CMakeLists.txt
//   target_add_binary_data() — no changes needed here for those.
//
// Memory note (no-PSRAM ESP32-D0WD-V3, 4 MB flash)
// --------------------------------------------------
// These strings are placed in flash (RODATA segment) by the linker.
// They are NOT copied to DRAM unless explicitly accessed via memcpy.
// httpd_resp_send / httpd_resp_sendstr read them directly through the
// flash cache, which is safe for read-only access.

#include "http_pages.hpp"

// ---------------------------------------------------------------------------
// Root / dashboard page
// ---------------------------------------------------------------------------
// Sent as one chunk — no version injection needed on this page; the firmware
// version is displayed as a static string baked in at compile time via the
// FIRMWARE_VERSION_STRING macro.  If you want a runtime value, split it like
// the motor page below.

const char kPageRoot[] =
R"RAW(<!DOCTYPE html>
<html>
<head>
  <title>Shelfbot ESP32</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>
    *{box-sizing:border-box}
    body{font-family:Inter,Arial,sans-serif;margin:0;background:#0f172a;color:#e2e8f0}
    .container{max-width:1100px;margin:0 auto;padding:28px}
    h1{margin:0 0 6px}
    .subtitle{color:#94a3b8;margin-bottom:24px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:16px}
    .card{background:#1e293b;border:1px solid #334155;padding:16px;border-radius:12px}
    .card h3{margin-top:0}
    code{background:#0b1220;padding:2px 6px;border-radius:4px;color:#93c5fd}
    a{color:#93c5fd;text-decoration:none;font-weight:600}
    a:hover{text-decoration:underline}
    .ver{font-size:12px;color:#475569;margin-top:4px}
  </style>
</head>
<body>
<div class="container">
  <h1>&#x1F916; Shelfbot Dashboard</h1>
  <p class="subtitle">ESP32 Firmware Control Interface</p>
  <div class="grid">
    <div class="card">
      <h3>&#x1F5FA;&#xFE0F; LiDAR Viewer</h3>
      <p><a href="/lidar.html">Open live LiDAR visualisation</a></p>
      <code>/lidar.html</code>
    </div>
    <div class="card">
      <h3>&#x1F6F0;&#xFE0F; LiDAR API</h3>
      <p><a href="/api/lidar">Open LiDAR JSON</a></p>
      <code>GET /api/lidar</code>
    </div>
    <div class="card">
      <h3>&#x2699;&#xFE0F; Motor Control</h3>
      <p><a href="/motor.html">Open motor control page</a></p>
      <code>/motor.html</code>
    </div>
    <div class="card">
      <h3>&#x2699;&#xFE0F; Motor API</h3>
      <p><a href="/api/motor/status">Open motor status JSON</a></p>
      <code>GET /api/motor/status</code>
    </div>
  </div>
</div>
</body>
</html>
)RAW";

const size_t kPageRootLen = sizeof(kPageRoot) - 1u; // exclude NUL

// ---------------------------------------------------------------------------
// Motor-control dashboard
//
// The firmware version is injected between the prefix and suffix at runtime.
// Using chunked send means no heap allocation is required:
//   httpd_resp_send_chunk(req, kPageMotorPrefix, kPageMotorPrefixLen);
//   httpd_resp_send_chunk(req, ver, strlen(ver));
//   httpd_resp_send_chunk(req, kPageMotorSuffix, kPageMotorSuffixLen);
//   httpd_resp_send_chunk(req, nullptr, 0);  // end of response
// ---------------------------------------------------------------------------

const char kPageMotorPrefix[] =
R"RAW(<!doctype html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Shelfbot Motor Control</title>
  <style>
    *{box-sizing:border-box}
    body{font-family:Inter,Arial;background:#0b1020;color:#e2e8f0;margin:0;padding:20px}
    .top{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-top:16px}
    .card{background:#141b34;border:1px solid #334155;border-radius:14px;padding:14px}
    .row{display:flex;gap:8px;align-items:center;margin:8px 0;flex-wrap:wrap}
    label{font-size:13px;color:#94a3b8;width:100%}
    input[type=number]{flex:1;min-width:80px;padding:8px;border-radius:8px;border:1px solid #475569;background:#0f172a;color:#e2e8f0}
    button{padding:8px 12px;border:0;border-radius:10px;background:#22c55e;color:#04110a;font-weight:700;cursor:pointer;white-space:nowrap}
    button.stop{background:#ef4444;color:#fff}
    .json{background:#020617;border-radius:10px;padding:10px;font-family:monospace;font-size:11px;min-height:90px;white-space:pre-wrap;overflow:auto;max-height:180px}
    .pill{display:inline-block;padding:2px 8px;border-radius:999px;background:#1e293b;font-size:12px}
    .pill.run{background:#14532d}
    a{color:#93c5fd;font-weight:600;text-decoration:none}
    a:hover{text-decoration:underline}
    h2{margin:0}
  </style>
</head>
<body>
<div class="top">
  <h2>&#x2699;&#xFE0F; Motor Control Dashboard</h2>
  <a href="/">&#x2190; Main Dashboard</a>
</div>
<p class="pill">Firmware: )RAW";

const size_t kPageMotorPrefixLen = sizeof(kPageMotorPrefix) - 1u;

const char kPageMotorSuffix[] =
R"RAW(</p>
<p style="font-size:13px;color:#94a3b8">
  Per-motor independent set/get — units: <b>position_rad</b> and <b>velocity_rad_s</b>.
</p>
<div class="grid" id="motors"></div>
<script>
(function(){
  'use strict';
  var N = 5;

  function makeCard(i) {
    return '<div class="card" id="card'+i+'">'
      +'<div class="row"><h3 style="margin:0">Motor '+i+'</h3>'
      +'<span class="pill" id="run'+i+'">--</span></div>'
      +'<div class="row"><label>Position (rad) — signed, 0 = continuous</label>'
      +'<input id="pos'+i+'" type="number" step="0.01" value="0"></div>'
      +'<div class="row"><label>Velocity (rad/s) — signed, 0 = stop</label>'
      +'<input id="vel'+i+'" type="number" step="0.01" value="0"></div>'
      +'<div class="row">'
      +'<button onclick="applyMotor('+i+')">Apply</button>'
      +'<button class="stop" onclick="stopMotor('+i+')">Stop</button>'
      +'</div>'
      +'<div class="row" style="font-size:12px;color:#94a3b8">'
      +'pos: <span id="apos'+i+'">--</span> rad &nbsp;|&nbsp;'
      +'vel: <span id="avel'+i+'">--</span> rad/s</div>'
      +'<div class="json" id="json'+i+'">Loading...</div>'
      +'</div>';
  }

  var grid = document.getElementById('motors');
  var html = '';
  for (var i = 0; i < N; i++) html += makeCard(i);
  grid.innerHTML = html;

  function postMotor(motor, pos, vel) {
    return fetch('/api/motor/set', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({motor: motor, position_rad: pos, velocity_rad_s: vel})
    });
  }

  window.applyMotor = function(i) {
    var pos = parseFloat(document.getElementById('pos'+i).value) || 0;
    var vel = parseFloat(document.getElementById('vel'+i).value) || 0;
    postMotor(i, pos, vel).then(pollStatus).catch(function(e){ console.error('apply failed',e); });
  };

  window.stopMotor = function(i) {
    postMotor(i, 0, 0).then(pollStatus).catch(function(e){ console.error('stop failed',e); });
  };

  function pollStatus() {
    fetch('/api/motor/status')
      .then(function(r){ return r.json(); })
      .then(function(data){
        data.motors.forEach(function(m){
          var runEl  = document.getElementById('run'+m.motor);
          var aposEl = document.getElementById('apos'+m.motor);
          var avelEl = document.getElementById('avel'+m.motor);
          var jsonEl = document.getElementById('json'+m.motor);
          if (!runEl) return;
          runEl.textContent = m.running ? '\u25B6 RUNNING' : '\u25A0 IDLE';
          runEl.className   = 'pill' + (m.running ? ' run' : '');
          aposEl.textContent = m.position_rad.toFixed(4);
          var v = m.velocity_rad_s;
          avelEl.textContent = (v >= 0 ? '+' : '') + v.toFixed(4);
          avelEl.style.color = v > 0 ? '#4ade80' : v < 0 ? '#f87171' : '#94a3b8';
          jsonEl.textContent = JSON.stringify(m, null, 2);
        });
      })
      .catch(function(e){ console.error('status poll failed', e); });
  }

  pollStatus();
  setInterval(pollStatus, 500);
})();
</script>
</body>
</html>
)RAW";

const size_t kPageMotorSuffixLen = sizeof(kPageMotorSuffix) - 1u;
