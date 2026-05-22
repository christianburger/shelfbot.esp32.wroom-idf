(function(){
  const svg = document.getElementById('lidar');
  const meta = document.getElementById('meta');
  const size = 600;
  const cx = size/2;
  const cy = size/2;
  const scale = 0.12; // mm → px  (1px = 8.33 mm)
  const startLogBtn    = document.getElementById('startLogBtn');
  const stopLogBtn     = document.getElementById('stopLogBtn');
  const saveLogBtn     = document.getElementById('saveLogBtn');
  const persistBtn     = document.getElementById('persistBtn');
  const clearPersistBtn= document.getElementById('clearPersistBtn');
  const logStatus      = document.getElementById('logStatus');

  let fileHandle = null, writable = null;
  let memoryLog = [], loggingActive = false;
  let persistMode = false, persistedPoints = [];
  const persistLimit = 3000;

  // Distance rings config
  const RINGS = [
    { mm: 250,  label: '250mm', minor: true  },
    { mm: 500,  label: '500mm', minor: false },
    { mm: 1000, label: '1m',    minor: false },
    { mm: 1500, label: '1.5m',  minor: false },
    { mm: 2000, label: '2m',    minor: false },
  ];

  // ── Cursor overlay (built once, re-appended after every drawGrid) ──────────
  let cursorGroup, cursorHLine, cursorVLine, cursorDot,
      cursorBg, cursorLine1, cursorLine2;

  function el(name, attrs) {
    const node = document.createElementNS('http://www.w3.org/2000/svg', name);
    Object.keys(attrs).forEach(k => node.setAttribute(k, attrs[k]));
    return node;
  }

  function buildCursorOverlay() {
    cursorGroup = el('g', { visibility: 'hidden' });

    cursorHLine = el('line', {
      x1: 0, y1: 0, x2: size, y2: 0,
      stroke: '#22d3ee', 'stroke-width': '0.5', 'stroke-dasharray': '3 4', opacity: '0.45'
    });
    cursorVLine = el('line', {
      x1: 0, y1: 0, x2: 0, y2: size,
      stroke: '#22d3ee', 'stroke-width': '0.5', 'stroke-dasharray': '3 4', opacity: '0.45'
    });
    cursorDot = el('circle', {
      cx: 0, cy: 0, r: 3,
      fill: 'none', stroke: '#22d3ee', 'stroke-width': '1', opacity: '0.8'
    });
    cursorBg = el('rect', {
      x: 0, y: 0, width: 115, height: 34, rx: 4,
      fill: '#020617', opacity: '0.9', stroke: '#334155', 'stroke-width': '0.5'
    });
    cursorLine1 = el('text', {
      x: 0, y: 0, fill: '#22d3ee',
      'font-size': '11px', 'font-family': 'monospace'
    });
    cursorLine2 = el('text', {
      x: 0, y: 0, fill: '#94a3b8',
      'font-size': '10px', 'font-family': 'monospace'
    });

    [cursorHLine, cursorVLine, cursorDot, cursorBg, cursorLine1, cursorLine2]
      .forEach(n => cursorGroup.appendChild(n));
  }

  // ── Grid ──────────────────────────────────────────────────────────────────
  function addAxisLabel(x, y, txt, anchor, dimmed) {
    const t = el('text', {
      x, y,
      fill: dimmed ? '#374151' : '#4ade80',
      'font-size': dimmed ? '9px' : '10px',
      'font-family': 'monospace',
      'text-anchor': anchor,
      'paint-order': 'stroke',
      stroke: '#020617',
      'stroke-width': '3',
      'stroke-linejoin': 'round'
    });
    t.textContent = txt;
    svg.appendChild(t);
  }

  function drawGrid() {
    svg.innerHTML = '';

    // Diagonal 45° guide lines
    [45, 135, 225, 315].forEach(deg => {
      const rad = (deg - 90) * Math.PI / 180;
      svg.appendChild(el('line', {
        x1: cx, y1: cy,
        x2: cx + Math.cos(rad) * cx * 1.42,
        y2: cy + Math.sin(rad) * cy * 1.42,
        stroke: '#172032', 'stroke-width': '0.75', 'stroke-dasharray': '4 4'
      }));
    });

    // Rings + labels on all 4 axes
    RINGS.forEach(({ mm, label, minor }) => {
      const r = mm * scale;

      // Ring circle
      svg.appendChild(el('circle', {
        cx, cy, r,
        fill: 'none',
        stroke: minor ? '#172032' : '#1e3a5f',
        'stroke-width': minor ? '0.75' : '1',
        'stroke-dasharray': minor ? '3 5' : ''
      }));

      // Top axis  — label just right of axis, just inside the ring
      addAxisLabel(cx + 4, cy - r + 11, label, 'start', minor);
      // Right axis — label just right of ring, just above axis
      addAxisLabel(cx + r + 4, cy - 3, label, 'start', minor);
      // Bottom axis — label just right of axis, just above ring bottom
      addAxisLabel(cx + 4, cy + r - 3, label, 'start', minor);
      // Left axis  — label just left of ring, just above axis
      addAxisLabel(cx - r - 4, cy - 3, label, 'end', minor);
    });

    // Cardinal axes
    svg.appendChild(el('line', { x1:0, y1:cy, x2:size, y2:cy, stroke:'#1e3a5f', 'stroke-width':'1' }));
    svg.appendChild(el('line', { x1:cx, y1:0, x2:cx, y2:size, stroke:'#1e3a5f', 'stroke-width':'1' }));

    // Origin dot
    svg.appendChild(el('circle', { cx, cy, r:4, fill:'#22d3ee' }));

    // Cardinal angle labels
    [
      { txt:'0°',   x: cx+6,    y: 15           },
      { txt:'90°',  x: size-34, y: cy-6         },
      { txt:'180°', x: cx+6,    y: size-6       },
      { txt:'270°', x: 6,       y: cy-6         },
    ].forEach(l => {
      const t = el('text', { x:l.x, y:l.y, fill:'#94a3b8', 'font-size':'12px', 'font-family':'monospace' });
      t.textContent = l.txt;
      svg.appendChild(t);
    });

    // 45° angle labels just outside the outermost ring
    const outerR = RINGS[RINGS.length - 1].mm * scale + 18;
    [45, 135, 225, 315].forEach(deg => {
      const rad = (deg - 90) * Math.PI / 180;
      const t = el('text', {
        x: cx + Math.cos(rad) * outerR,
        y: cy + Math.sin(rad) * outerR,
        fill: '#475569', 'font-size': '10px',
        'font-family': 'monospace', 'text-anchor': 'middle'
      });
      t.textContent = deg + '°';
      svg.appendChild(t);
    });

    // Always keep cursor overlay on top
    if (cursorGroup) svg.appendChild(cursorGroup);
  }

  // ── Mouse hover: live coordinate readout ─────────────────────────────────
  function getSVGPoint(e) {
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    return pt.matrixTransform(svg.getScreenCTM().inverse());
  }

  svg.addEventListener('mousemove', (e) => {
    if (!cursorGroup) return;
    const p = getSVGPoint(e);

    const dx = p.x - cx, dy = p.y - cy;
    const distMm = Math.round(Math.sqrt(dx * dx + dy * dy) / scale);
    let angleDeg = Math.atan2(dy, dx) * 180 / Math.PI + 90;
    if (angleDeg < 0)   angleDeg += 360;
    if (angleDeg >= 360) angleDeg -= 360;

    // Update crosshairs and dot
    cursorHLine.setAttribute('y1', p.y); cursorHLine.setAttribute('y2', p.y);
    cursorVLine.setAttribute('x1', p.x); cursorVLine.setAttribute('x2', p.x);
    cursorDot.setAttribute('cx', p.x);   cursorDot.setAttribute('cy', p.y);

    // Flip tooltip away from edges
    const flipX = p.x > size * 0.72;
    const flipY = p.y < 44;
    const tx = flipX ? p.x - 123 : p.x + 8;
    const ty = flipY ? p.y + 8   : p.y - 40;

    cursorBg.setAttribute('x', tx);   cursorBg.setAttribute('y', ty);
    cursorLine1.setAttribute('x', tx + 7); cursorLine1.setAttribute('y', ty + 14);
    cursorLine2.setAttribute('x', tx + 7); cursorLine2.setAttribute('y', ty + 27);
    cursorLine1.textContent = distMm + ' mm  (' + (distMm / 10).toFixed(1) + ' cm)';
    cursorLine2.textContent = angleDeg.toFixed(1) + '°';

    cursorGroup.setAttribute('visibility', 'visible');
  });

  svg.addEventListener('mouseleave', () => {
    if (cursorGroup) cursorGroup.setAttribute('visibility', 'hidden');
  });

  // ── Scan rendering ────────────────────────────────────────────────────────
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
      const end   = Number(data.end_angle_deg || start);
      let span = end - start;
      if (span < 0) span += 360;
      return data.points.map((p, i) => ({
        distance_mm: Number(p.d || 0),
        intensity:   Number(p.c || 0),
        angle_deg:   (start + (span * i / Math.max(1, data.points.length - 1))) % 360,
        valid:       Number(p.d || 0) > 0
      }));
    }
    return (data.points || []).map(p => ({
      distance_mm: Number(p.distance_mm || 0),
      intensity:   Number(p.intensity   || 0),
      angle_deg:   Number(p.angle_deg   || 0),
      valid:       Boolean(p.valid) && Number(p.distance_mm || 0) > 0
    }));
  }

  function drawScan(data) {
    drawGrid();
    const points = normalizePoints(data).filter(p => p.valid);

    if (persistMode && points.length > 0) {
      for (const p of points) persistedPoints.push(p);
      if (persistedPoints.length > persistLimit)
        persistedPoints.splice(0, persistedPoints.length - persistLimit);
    }

    if (persistMode && persistedPoints.length > 0) {
      persistedPoints.forEach(p => {
        const pt = pointToXY(p.distance_mm, p.angle_deg);
        svg.appendChild(el('circle', { cx:pt.x, cy:pt.y, r:1.5, fill:'#60a5fa', opacity:'0.5' }));
      });
    }

    points.forEach(p => {
      const pt = pointToXY(p.distance_mm, p.angle_deg);
      const dot = el('circle', { cx:pt.x, cy:pt.y, r:2.5, fill:'#4ade80', cursor:'pointer' });
      dot.addEventListener('click', () => {
        meta.textContent = JSON.stringify({
          clicked_point: {
            angle_deg:   p.angle_deg,
            distance_mm: p.distance_mm,
            distance_cm: +(p.distance_mm / 10).toFixed(1),
            intensity:   p.intensity
          },
          start_angle_deg:       data.start_angle_deg,
          end_angle_deg:         data.end_angle_deg,
          min_distance_angle_deg:data.min_distance_angle_deg,
          point_count:           points.length,
          timestamp_ms:          data.timestamp_ms
        }, null, 2);
      });
      svg.appendChild(dot);
    });

    // Cursor overlay stays on top of scan points
    if (cursorGroup) svg.appendChild(cursorGroup);

    if (points.length === 0) {
      const fallback      = Number(data.distance_mm || 0);
      const fallbackAngle = Number(data.min_distance_angle_deg || 0);
      if (fallback > 0) {
        const fp = pointToXY(fallback, fallbackAngle);
        svg.appendChild(el('circle', { cx:fp.x, cy:fp.y, r:4, fill:'#f59e0b' }));
        meta.textContent = JSON.stringify({
          info: 'No per-point packet; showing min-distance fallback.',
          distance_mm:           fallback,
          distance_cm:           +(fallback / 10).toFixed(1),
          min_distance_angle_deg:fallbackAngle,
          timestamp_us:          data.timestamp_us
        }, null, 2);
      } else {
        meta.textContent = 'No valid point data in current frame.';
      }
    } else {
      meta.textContent = JSON.stringify({
        start_angle_deg:        data.start_angle_deg,
        end_angle_deg:          data.end_angle_deg,
        min_distance_angle_deg: data.min_distance_angle_deg,
        min_distance_mm:        data.distance_mm,
        min_distance_cm:        data.distance_mm ? +(data.distance_mm / 10).toFixed(1) : null,
        point_count:            points.length,
        timestamp_ms:           data.timestamp_ms
      }, null, 2);
    }
  }

  // ── Controls ──────────────────────────────────────────────────────────────
  function togglePersistMode() {
    persistMode = !persistMode;
    persistBtn.textContent = `Persist Points: ${persistMode ? 'ON' : 'OFF'}`;
    logStatus.textContent  = persistMode
      ? `Persistence on (buffer limit ${persistLimit} points).`
      : `Persistence off (${persistedPoints.length} buffered points kept).`;
  }

  function clearPersistedPoints() {
    persistedPoints = [];
    logStatus.textContent = 'Persisted point buffer cleared.';
  }

  async function writeLogLine(data) {
    const line = JSON.stringify({ captured_at_iso: new Date().toISOString(), lidar: data }) + '\n';
    memoryLog.push(line);
    if (writable) await writable.write(line);
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
      writable      = await fileHandle.createWritable();
      loggingActive = true;
      logStatus.textContent = 'Local logging: ON (writing to selected file)';
    } catch (e) {
      logStatus.textContent = 'Log start canceled or failed: ' + e;
    }
  }

  async function stopLocalLog() {
    loggingActive = false;
    if (writable) { await writable.close(); writable = null; }
    logStatus.textContent = 'Local logging: stopped.';
  }

  function downloadBackupLog() {
    if (memoryLog.length === 0) { logStatus.textContent = 'No captured samples yet.'; return; }
    const blob = new Blob(memoryLog, { type: 'application/x-ndjson' });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href = url; a.download = `lidar-raw-backup-${Date.now()}.ndjson`;
    a.click(); URL.revokeObjectURL(url);
    logStatus.textContent = `Downloaded backup with ${memoryLog.length} frames.`;
  }

  async function refresh() {
    try {
      const res  = await fetch('/api/lidar');
      const data = await res.json();
      if (loggingActive) await writeLogLine(data);
      drawScan(data);
    } catch (e) {
      meta.textContent = 'Failed to fetch /api/lidar: ' + e;
    }
  }

  // ── Init ──────────────────────────────────────────────────────────────────
  buildCursorOverlay();
  refresh();
  setInterval(refresh, 400);
  startLogBtn.addEventListener('click',    startLocalLog);
  stopLogBtn.addEventListener('click',     stopLocalLog);
  saveLogBtn.addEventListener('click',     downloadBackupLog);
  persistBtn.addEventListener('click',     togglePersistMode);
  clearPersistBtn.addEventListener('click',clearPersistedPoints);
})();
