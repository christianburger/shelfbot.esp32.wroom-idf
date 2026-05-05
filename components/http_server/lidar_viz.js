(function(){
  const svg = document.getElementById('lidar');
  const meta = document.getElementById('meta');
  const size = 600;
  const cx = size/2;
  const cy = size/2;
  const scale = 0.12; // mm to px

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
  }

  function pointToXY(distanceMm, angleDeg) {
    const rad = (angleDeg - 90) * Math.PI / 180;
    return {
      x: cx + Math.cos(rad) * distanceMm * scale,
      y: cy + Math.sin(rad) * distanceMm * scale
    };
  }

  function drawScan(data) {
    drawGrid();
    const points = (data.points || []).filter(p => p && p.valid && p.distance_mm > 0);
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
      meta.textContent = 'No valid points in current frame.';
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

  async function refresh() {
    try {
      const res = await fetch('/api/lidar');
      const data = await res.json();
      drawScan(data);
    } catch (e) {
      meta.textContent = 'Failed to fetch /api/lidar: ' + e;
    }
  }

  refresh();
  setInterval(refresh, 400);
})();
