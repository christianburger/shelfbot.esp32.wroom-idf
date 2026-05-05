(function(){
  const svg = document.getElementById('lidar');
  const meta = document.getElementById('meta');
  const size = 600;
  const cx = size/2;
  const cy = size/2;
  const scale = 0.12; // mm to px
  const startLogBtn = document.getElementById('startLogBtn');
  const stopLogBtn = document.getElementById('stopLogBtn');
  const saveLogBtn = document.getElementById('saveLogBtn');
  const persistBtn = document.getElementById('persistBtn');
  const clearPersistBtn = document.getElementById('clearPersistBtn');
  const logStatus = document.getElementById('logStatus');
  let fileHandle = null;
  let writable = null;
  let memoryLog = [];
  let loggingActive = false;
  let persistMode = false;
  let persistedPoints = [];
  const persistLimit = 3000;

  function el(name, attrs) {
    const node = document.createElementNS('http://www.w3.org/2000/svg', name);
    Object.keys(attrs).forEach(k => node.setAttribute(k, attrs[k]));
    return node;
  }

  function drawGrid() {
    svg.innerHTML = '';
    [500, 1000, 1500, 2000].forEach(mm => {
      svg.appendChild(el('circle', {cx, cy, r: mm*scale, fill:'none', stroke:'#1e293b', 'stroke-width':'1'}));
    });
    svg.appendChild(el('line', {x1:0,y1:cy,x2:size,y2:cy,stroke:'#1e293b'}));
    svg.appendChild(el('line', {x1:cx,y1:0,x2:cx,y2:size,stroke:'#1e293b'}));
    svg.appendChild(el('circle', {cx, cy, r:4, fill:'#22d3ee'}));
    const labels = [
      { txt: '0°', x: cx, y: 18 },
      { txt: '90°', x: size - 30, y: cy - 8 },
      { txt: '180°', x: cx - 16, y: size - 10 },
      { txt: '270°', x: 8, y: cy - 8 }
    ];
    labels.forEach(l => {
      const t = el('text', { x: l.x, y: l.y, fill: '#94a3b8', 'font-size': '12px' });
      t.textContent = l.txt;
      svg.appendChild(t);
    });
  }

  function pointToXY(distanceMm, angleDeg) {
    const rad = (angleDeg - 90) * Math.PI / 180;
    return {
      x: cx + Math.cos(rad) * distanceMm * scale,
      y: cy + Math.sin(rad) * distanceMm * scale
    };
  }

  function normalizePoints(data) {
    if (Array.isArray(data.points) && data.points.length > 0 && data.points[0].d !== undefined) {
      const start = Number(data.start_angle_deg || 0);
      const end = Number(data.end_angle_deg || start);
      let span = end - start;
      if (span < 0) span += 360;
      return data.points.map((p, i) => ({
        distance_mm: Number(p.d || 0),
        intensity: Number(p.c || 0),
        angle_deg: (start + (span * i / Math.max(1, data.points.length - 1))) % 360,
        valid: Number(p.d || 0) > 0
      }));
    }
    return (data.points || []).map(p => ({
      distance_mm: Number(p.distance_mm || 0),
      intensity: Number(p.intensity || 0),
      angle_deg: Number(p.angle_deg || 0),
      valid: Boolean(p.valid) && Number(p.distance_mm || 0) > 0
    }));
  }

  function drawScan(data) {
    drawGrid();
    const points = normalizePoints(data).filter(p => p.valid);
    if (persistMode && points.length > 0) {
      for (const p of points) {
        persistedPoints.push({ distance_mm: p.distance_mm, angle_deg: p.angle_deg, intensity: p.intensity });
      }
      if (persistedPoints.length > persistLimit) {
        persistedPoints.splice(0, persistedPoints.length - persistLimit);
      }
    }
    if (persistMode && persistedPoints.length > 0) {
      persistedPoints.forEach((p) => {
        const pt = pointToXY(p.distance_mm, p.angle_deg);
        svg.appendChild(el('circle', {cx:pt.x, cy:pt.y, r:1.5, fill:'#60a5fa', opacity:'0.5'}));
      });
    }
    points.forEach((p) => {
      const pt = pointToXY(p.distance_mm, p.angle_deg);
      const dot = el('circle', {cx:pt.x, cy:pt.y, r:2.5, fill:'#4ade80', cursor:'pointer'});
      dot.addEventListener('click', () => {
        meta.textContent = JSON.stringify({
          clicked_point: {
            angle_deg: p.angle_deg,
            distance_mm: p.distance_mm,
            intensity: p.intensity
          },
          start_angle_deg: data.start_angle_deg,
          end_angle_deg: data.end_angle_deg,
          min_distance_angle_deg: data.min_distance_angle_deg,
          point_count: points.length,
          timestamp_ms: data.timestamp_ms
        }, null, 2);
      });
      svg.appendChild(dot);
    });

    if (points.length === 0) {
      const fallback = Number(data.distance_mm || 0);
      const fallbackAngle = Number(data.min_distance_angle_deg || 0);
      if (fallback > 0) {
        const fp = pointToXY(fallback, fallbackAngle);
        svg.appendChild(el('circle', {cx:fp.x, cy:fp.y, r:4, fill:'#f59e0b'}));
        meta.textContent = JSON.stringify({
          info: 'No per-point packet in response; showing min-distance fallback.',
          distance_mm: fallback,
          min_distance_angle_deg: fallbackAngle,
          timestamp_us: data.timestamp_us
        }, null, 2);
      } else {
        meta.textContent = 'No valid point data in current frame.';
      }
    } else {
      meta.textContent = JSON.stringify({
        start_angle_deg: data.start_angle_deg,
        end_angle_deg: data.end_angle_deg,
        min_distance_angle_deg: data.min_distance_angle_deg,
        point_count: points.length,
        timestamp_ms: data.timestamp_ms
      }, null, 2);
    }
  }

  function togglePersistMode() {
    persistMode = !persistMode;
    persistBtn.textContent = `Persist Points: ${persistMode ? 'ON' : 'OFF'}`;
    if (!persistMode) {
      logStatus.textContent = `Persistence off (${persistedPoints.length} buffered points kept).`;
    } else {
      logStatus.textContent = `Persistence on (buffer limit ${persistLimit} points).`;
    }
  }

  function clearPersistedPoints() {
    persistedPoints = [];
    logStatus.textContent = 'Persisted point buffer cleared.';
  }

  async function writeLogLine(data) {
    const line = JSON.stringify({
      captured_at_iso: new Date().toISOString(),
      lidar: data
    }) + '\n';
    memoryLog.push(line);
    if (writable) {
      await writable.write(line);
    }
  }

  async function startLocalLog() {
    if (!window.showSaveFilePicker) {
      logStatus.textContent = 'File System Access API unavailable; using in-memory backup only.';
      loggingActive = true;
      return;
    }
    try {
      fileHandle = await window.showSaveFilePicker({
        suggestedName: `lidar-raw-${Date.now()}.ndjson`,
        types: [{ description: 'NDJSON', accept: { 'application/x-ndjson': ['.ndjson'] } }]
      });
      writable = await fileHandle.createWritable();
      loggingActive = true;
      logStatus.textContent = 'Local logging: ON (writing to selected file)';
    } catch (e) {
      logStatus.textContent = 'Log start canceled or failed: ' + e;
    }
  }

  async function stopLocalLog() {
    loggingActive = false;
    if (writable) {
      await writable.close();
      writable = null;
      logStatus.textContent = 'Local logging: stopped and file closed.';
    } else {
      logStatus.textContent = 'Local logging: stopped.';
    }
  }

  function downloadBackupLog() {
    if (memoryLog.length === 0) {
      logStatus.textContent = 'No captured samples yet.';
      return;
    }
    const blob = new Blob(memoryLog, { type: 'application/x-ndjson' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `lidar-raw-backup-${Date.now()}.ndjson`;
    a.click();
    URL.revokeObjectURL(url);
    logStatus.textContent = `Downloaded backup with ${memoryLog.length} frames.`;
  }

  async function refresh() {
    try {
      const res = await fetch('/api/lidar');
      const data = await res.json();
      if (loggingActive) {
        await writeLogLine(data);
      }
      drawScan(data);
    } catch (e) {
      meta.textContent = 'Failed to fetch /api/lidar: ' + e;
    }
  }

  refresh();
  setInterval(refresh, 400);
  startLogBtn.addEventListener('click', startLocalLog);
  stopLogBtn.addEventListener('click', stopLocalLog);
  saveLogBtn.addEventListener('click', downloadBackupLog);
  persistBtn.addEventListener('click', togglePersistMode);
  clearPersistBtn.addEventListener('click', clearPersistedPoints);
})();
