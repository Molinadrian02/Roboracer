'use strict';
/* websim F1TENTH — browser client */

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v == null || !isFinite(v) ? '–' : v.toFixed(d).replace('.', ','));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const store = {
  get(k, d) { try { const v = localStorage.getItem('websim.' + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('websim.' + k, JSON.stringify(v)); } catch { /* storage unavailable */ } },
};

const COLORS = {
  concrete: [214, 209, 196], erased: [226, 222, 211], wall: [44, 49, 55], painted: [58, 95, 168],
  outside: '#2c3137', yellow: '#f5c518', teal: '#12a4a4', kerb: '#e0442e', text: '#e8e6e1', asphalt: '#20252b',
};
const SERIES_COLORS = ['#12a4a4', '#5b8def', '#e0442e', '#46b471', '#c77dff', '#ff9f43'];
const HISTORY_S = 12;

const S = {
  ws: null, hello: null, map: null, st: null,
  mapCanvas: document.createElement('canvas'), mapVersion: -1, mapLoading: false,
  trail: [], epoch: -1,
  view: { scale: 40, cx: 0, cy: 0 }, follow: true, lidarMode: store.get('lidar', 0), showTrail: true,
  tool: 'pan', pointer: null, mouseW: null,
  painting: null, paintHoldUntil: 0,
  hist: { speed: [], topics: {} },
  plotKeys: store.get('plotKeys', []),
  topicsSig: '',
  keys: new Set(), steer: 0, kbdV: store.get('kbdV', 2), lastSentActive: false,
  logIds: new Set(),
};

/* ================================================================ socket */
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  S.ws = ws;
  ws.onopen = () => { setPill('pillConn', 'ok', 'Conectado'); $('offline').hidden = true; };
  ws.onclose = () => {
    setPill('pillConn', 'bad', 'Sin conexión');
    $('offline').hidden = false;
    setTimeout(connect, 1200);
  };
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.t === 's') onState(m);
    else if (m.t === 'hello') onHello(m);
    else if (m.t === 'map') onMap(m.map, true);
  };
}
function send(obj) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(obj)); }

function setPill(id, cls, text) {
  const el = $(id); el.className = 'pill ' + (cls || ''); el.textContent = text;
}

/* ================================================================ hello / map */
function onHello(h) {
  S.hello = h;
  const sel = $('mapSelect');
  sel.innerHTML = '';
  for (const m of h.maps) {
    const o = document.createElement('option');
    o.value = m.name; o.textContent = m.name; o.title = m.description || '';
    sel.appendChild(o);
  }
  onMap(h.map, false);
  S.trail = h.trail.map((p) => p.slice());
  $('log').innerHTML = ''; S.logIds.clear();
  addEvents(h.events);
  setPill('pillRos', h.ros ? 'ok' : 'bad', h.ros ? 'ROS 2' : 'ROS 2 apagado');
  $('rosBlock').classList.toggle('off', !h.ros);
  $('driveTopic').textContent = h.topics.drive;
  $('odomTopic').textContent = h.topics.odom;
  if (!h.ros) {
    $('drivePubs').textContent = 'ROS 2 no está activo (se lanzó con --no-ros o falta rclpy).';
    $('topicHint').textContent = 'Sin ROS 2 solo puedes graficar las distancias del LiDAR simulado.';
  }
  rebuildTopicOptions([]);
  sendPlotKeys();
}

function onMap(meta, fromSwitch) {
  const changedName = !S.map || S.map.name !== meta.name;
  S.map = meta;
  $('mapSelect').value = meta.name;
  $('mapSelect').title = meta.description || '';
  if (changedName || fromSwitch) { S.trail = []; fitView(); }
  loadGrid(meta.version);
}

function loadGrid(version) {
  if (S.mapLoading) return;
  S.mapLoading = true;
  const img = new Image();
  img.onload = () => {
    const c = S.mapCanvas; c.width = img.width; c.height = img.height;
    const ctx = c.getContext('2d', { willReadFrequently: true });
    ctx.drawImage(img, 0, 0);
    const d = ctx.getImageData(0, 0, c.width, c.height);
    const px = d.data;
    for (let i = 0; i < px.length; i += 4) {
      const g = px[i];
      const col = g === 255 ? COLORS.concrete : g === 230 ? COLORS.erased : g === 60 ? COLORS.painted : COLORS.wall;
      px[i] = col[0]; px[i + 1] = col[1]; px[i + 2] = col[2]; px[i + 3] = 255;
    }
    ctx.putImageData(d, 0, 0);
    S.mapVersion = version;
    S.mapLoading = false;
  };
  img.onerror = () => { S.mapLoading = false; };
  img.src = `/api/grid.png?v=${version}&t=${Date.now()}`;
}

/* ================================================================ state */
function onState(m) {
  const prev = S.st;
  S.st = m;
  if (m.epoch !== S.epoch) { if (S.epoch !== -1) S.trail = []; S.epoch = m.epoch; }
  if (m.map_version !== S.mapVersion && !S.painting && performance.now() > S.paintHoldUntil) loadGrid(m.map_version);

  const last = S.trail[S.trail.length - 1];
  if (!last || Math.hypot(m.x - last[0], m.y - last[1]) > 0.05) {
    S.trail.push([m.x, m.y, m.v]);
    if (S.trail.length > 4000) S.trail.splice(0, 500);
  }

  // histories (keyed by simulation time)
  if (!prev || m.simt !== prev.simt) {
    if (prev && m.simt < prev.simt) { S.hist.speed = []; S.hist.topics = {}; }
    S.hist.speed.push([m.simt, m.v, m.cmd[0]]);
    const sc = scanStats(m);
    pushTopic('sim|front', m.simt, sc.front);
    pushTopic('sim|min', m.simt, sc.min);
  }
  if (m.samples) for (const [k, arr] of Object.entries(m.samples)) for (const [t, v] of arr) pushTopic(k, t, v);
  trimHistory(m.simt);
  if (m.events) addEvents(m.events);
  if (m.ros) updateRos(m.ros);
  updatePanel(m);
}

function pushTopic(key, t, v) {
  if (!S.plotKeys.includes(key)) return;
  (S.hist.topics[key] = S.hist.topics[key] || []).push([t, v]);
}
function trimHistory(now) {
  const lim = now - HISTORY_S - 1;
  const cut = (a) => { let i = 0; while (i < a.length && a[i][0] < lim) i++; if (i) a.splice(0, i); };
  cut(S.hist.speed);
  for (const a of Object.values(S.hist.topics)) cut(a);
}

function scanStats(m) {
  const h = S.hello; if (!h || !m.scan) return { front: null, min: null };
  const mid = Math.round(-h.scan.angle_min / h.scan.inc);
  const span = Math.round((3 * Math.PI / 180) / h.scan.inc);
  let front = Infinity, min = Infinity;
  for (let i = 0; i < m.scan.length; i++) {
    const r = m.scan[i] / 100;
    if (r < min) min = r;
    if (Math.abs(i - mid) <= span && r < front) front = r;
  }
  return { front, min };
}

/* ================================================================ panel */
const MODE_HELP = {
  ros: 'Solo conduce lo que llegue por /drive. El teclado no hace nada.',
  mix: 'Conduces con el teclado, pero un mensaje en /drive manda sobre él durante 0,25 s. Así se prueba el frenado de emergencia.',
  kbd: 'Solo el teclado. Lo que llegue por /drive se ignora.',
};
const SRC_NAME = { ros: 'ROS', kbd: 'Teclado', estop: 'Freno', estop_ros: 'Freno (ROS)', none: 'nadie' };

function updatePanel(m) {
  const h = S.hello; if (!h) return;
  $('vNow').textContent = fmt(m.v);
  $('vCmd').textContent = fmt(m.cmd[0]);
  const lo = -5, hi = 10;
  const pos = (v) => ((clamp(v, lo, hi) - lo) / (hi - lo)) * 100;
  const z = pos(0), p = pos(m.v);
  $('stripFill').style.left = Math.min(z, p) + '%';
  $('stripFill').style.width = Math.abs(p - z) + '%';
  $('stripCmd').style.left = pos(m.cmd[0]) + '%';
  document.querySelector('.strip-zero').style.left = z + '%';

  const smax = h.car.s_max;
  setNeedle('steerNeedle', m.steer / smax);
  setNeedle('steerCmdNeedle', m.cmd[1] / smax);
  $('steerNow').textContent = fmt(m.steer * 180 / Math.PI, 1) + '°';

  const plate = $('sourcePlate');
  plate.className = 'source src-' + m.src;
  $('srcName').textContent = SRC_NAME[m.src] || m.src;

  for (const b of document.querySelectorAll('.modes button')) b.setAttribute('aria-checked', String(b.dataset.mode === m.mode));
  $('modeHelp').textContent = MODE_HELP[m.mode];

  $('btnEstop').classList.toggle('on', m.estop);
  const pb = $('btnPause');
  pb.classList.toggle('is-paused', m.paused);
  pb.querySelector('use').setAttribute('href', m.paused ? '#i-play' : '#i-pause');
  pb.querySelector('span').textContent = m.paused ? 'Reanudar' : 'Pausar';
  $('btnStep').disabled = !m.paused;
  $('rtfSelect').value = String(m.rtf);
  $('chkAutoReset').checked = m.auto_reset;

  const sc = scanStats(m);
  $('stFront').textContent = sc.front == null ? '–' : fmt(sc.front) + ' m';
  $('stMin').textContent = sc.min == null ? '–' : fmt(sc.min) + ' m';
  $('stColl').textContent = m.ncoll;
  $('stColl').classList.toggle('warn', m.ncoll > 0);
  $('stLap').textContent = fmt(m.lap.cur, 1) + ' s';
  $('stBest').textContent = m.lap.best == null ? '–' : fmt(m.lap.best, 2) + ' s';
  $('stDist').textContent = Math.round(m.dist) + ' m';

  const banner = $('banner');
  if (m.coll) { banner.hidden = false; banner.className = 'banner'; banner.textContent = 'Colisión'; }
  else if (m.estop) { banner.hidden = false; banner.className = 'banner'; banner.textContent = 'Freno de emergencia activado'; }
  else if (m.ros_estop) { banner.hidden = false; banner.className = 'banner'; banner.textContent = 'Freno de emergencia activado desde ROS'; }
  else if (m.paused) { banner.hidden = false; banner.className = 'banner paused'; banner.textContent = 'En pausa'; }
  else banner.hidden = true;
}

function setNeedle(id, frac) {
  const a = -clamp(frac, -1, 1) * (80 * Math.PI / 180);
  const el = $(id);
  el.setAttribute('x2', (60 + 42 * Math.sin(a)).toFixed(1));
  el.setAttribute('y2', (58 - 42 * Math.cos(a)).toFixed(1));
}

function updateRos(r) {
  $('driveHz').textContent = r.drive_hz + ' Hz';
  $('scanHz').textContent = r.scan_hz + ' Hz';
  $('drivePubs').textContent = r.drive_pubs.length ? r.drive_pubs.join(', ') : 'nadie todavía';
  $('lastDrive').textContent = r.last_drive
    ? `${fmt(r.last_drive[0])} m/s, ${fmt(r.last_drive[1] * 180 / Math.PI, 1)}°` + (r.drive_age != null ? `, hace ${fmt(r.drive_age, 1)} s` : '')
    : '–';
  const sig = r.topics.map((t) => t.topic + t.type).join(',');
  if (sig !== S.topicsSig) { S.topicsSig = sig; rebuildTopicOptions(r.topics); }
}

function addEvents(evs) {
  const ol = $('log');
  for (const ev of evs) {
    if (S.logIds.has(ev.id)) continue;
    S.logIds.add(ev.id);
    const li = document.createElement('li');
    li.className = ev.level;
    const t = document.createElement('time'); t.textContent = fmt(ev.simt, 1) + ' s';
    const s = document.createElement('span'); s.textContent = ev.msg;
    li.append(t, s);
    ol.prepend(li);
    while (ol.children.length > 120) ol.lastChild.remove();
  }
}

/* ================================================================ topic plots */
function keyLabel(k) {
  if (k === 'sim|front') return 'distancia al frente';
  if (k === 'sim|min') return 'distancia mínima';
  const [topic, field] = k.split('|');
  return field && field !== 'data' ? `${topic} (${field})` : topic;
}
function rebuildTopicOptions(topics) {
  const sel = $('topicSelect');
  sel.innerHTML = '<option value="">Añadir un topic para graficar…</option>';
  const g1 = document.createElement('optgroup'); g1.label = 'LiDAR simulado';
  for (const k of ['sim|front', 'sim|min']) {
    const o = document.createElement('option'); o.value = k; o.textContent = keyLabel(k); g1.appendChild(o);
  }
  sel.appendChild(g1);
  if (topics.length) {
    const g2 = document.createElement('optgroup'); g2.label = 'Topics de ROS 2';
    for (const t of topics) for (const f of t.fields) {
      const o = document.createElement('option'); o.value = `${t.topic}|${f}`; o.textContent = keyLabel(o.value); g2.appendChild(o);
    }
    sel.appendChild(g2);
  }
  renderTopicLegend();
}
function renderTopicLegend() {
  const box = $('topicLegend');
  box.innerHTML = '';
  S.plotKeys.forEach((k, i) => {
    const chip = document.createElement('span'); chip.className = 'chip';
    const sw = document.createElement('i'); sw.style.background = SERIES_COLORS[i % SERIES_COLORS.length];
    const lab = document.createElement('span'); lab.textContent = keyLabel(k);
    const x = document.createElement('button'); x.textContent = '×'; x.title = 'Quitar'; x.setAttribute('aria-label', 'Quitar ' + keyLabel(k));
    x.onclick = () => { S.plotKeys = S.plotKeys.filter((q) => q !== k); delete S.hist.topics[k]; plotKeysChanged(); };
    chip.append(sw, lab, x); box.appendChild(chip);
  });
  $('topicHint').hidden = S.plotKeys.some((k) => !k.startsWith('sim|'));
}
function plotKeysChanged() { store.set('plotKeys', S.plotKeys); renderTopicLegend(); sendPlotKeys(); }
function sendPlotKeys() { send({ t: 'plot', keys: S.plotKeys.filter((k) => !k.startsWith('sim|')) }); }
$('topicSelect').onchange = (e) => {
  const k = e.target.value; e.target.value = '';
  if (k && !S.plotKeys.includes(k)) { S.plotKeys.push(k); if (S.plotKeys.length > 6) S.plotKeys.shift(); plotKeysChanged(); }
};

function drawChart(canvas, series, now, opts = {}) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== W * dpr || canvas.height !== H * dpr) { canvas.width = W * dpr; canvas.height = H * dpr; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const t0 = now - HISTORY_S;
  let lo = Infinity, hi = -Infinity;
  for (const s of series) for (const p of s.pts) if (p[0] >= t0 && isFinite(p[1])) { lo = Math.min(lo, p[1]); hi = Math.max(hi, p[1]); }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (opts.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
  const minSpan = opts.minSpan || 0.5;
  if (hi - lo < minSpan) { const c = (hi + lo) / 2; lo = c - minSpan / 2; hi = c + minSpan / 2; }
  const pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
  const L = 34, R = 6, T = 6, B = 16;
  const X = (t) => L + ((t - t0) / HISTORY_S) * (W - L - R);
  const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (H - T - B);

  // grid + labels
  ctx.font = '500 11px "Barlow Condensed", sans-serif';
  ctx.fillStyle = '#6d7680'; ctx.strokeStyle = 'rgba(255,255,255,0.06)'; ctx.lineWidth = 1;
  const step = niceStep((hi - lo) / 3);
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    ctx.fillText(fmt(v, step < 1 ? 1 : 0), L - 5, y);
  }
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  for (let s = 0; s <= HISTORY_S; s += 4) {
    ctx.textAlign = s === HISTORY_S ? 'right' : 'center';
    ctx.fillText(s === HISTORY_S ? 'ahora' : `−${HISTORY_S - s} s`, s === HISTORY_S ? W - R : X(t0 + s), H);
  }

  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 2; ctx.setLineDash(s.dash || []);
    ctx.beginPath(); let started = false;
    for (const p of s.pts) {
      if (p[0] < t0 || !isFinite(p[1])) { started = false; continue; }
      const x = X(p[0]), y = Y(p[1]);
      if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);
}
function niceStep(raw) {
  const p = Math.pow(10, Math.floor(Math.log10(raw || 1)));
  const n = raw / p;
  return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * p;
}

/* ================================================================ view */
const canvas = $('view');
const ctx = canvas.getContext('2d');
const stage = $('stage');

function viewSize() { return [canvas.clientWidth, canvas.clientHeight]; }
function w2s(x, y) { const [W, H] = viewSize(); return [W / 2 + (x - S.view.cx) * S.view.scale, H / 2 - (y - S.view.cy) * S.view.scale]; }
function s2w(px, py) { const [W, H] = viewSize(); return [S.view.cx + (px - W / 2) / S.view.scale, S.view.cy - (py - H / 2) / S.view.scale]; }

function fitView() {
  const m = S.map; if (!m) return;
  const [W, H] = viewSize();
  const mw = m.width * m.resolution, mh = m.height * m.resolution;
  S.view.scale = clamp(Math.min(W / mw, H / mh) * 0.92, 3, 300);
  S.view.cx = m.origin[0] + mw / 2; S.view.cy = m.origin[1] + mh / 2;
  if (S.follow && m.spawn) {
    // following the car: start closer than "whole map", centred on the start line
    S.view.scale = Math.max(S.view.scale, 45);
    if (mw * S.view.scale > W || mh * S.view.scale > H) { S.view.cx = m.spawn[0]; S.view.cy = m.spawn[1]; }
  }
}

function draw() {
  requestAnimationFrame(draw);
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  const st = S.st, m = S.map, h = S.hello;
  if (st && S.follow && !(S.pointer && S.pointer.kind === 'pan')) {
    // soft follow: keep the car inside the middle 40% of the view
    const [sx, sy] = w2s(st.x, st.y);
    const mx = W * 0.2, my = H * 0.2;
    if (sx < W / 2 - mx) S.view.cx -= (W / 2 - mx - sx) / S.view.scale;
    if (sx > W / 2 + mx) S.view.cx += (sx - W / 2 - mx) / S.view.scale;
    if (sy < H / 2 - my) S.view.cy += (H / 2 - my - sy) / S.view.scale;
    if (sy > H / 2 + my) S.view.cy -= (sy - H / 2 - my) / S.view.scale;
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = COLORS.outside; ctx.fillRect(0, 0, W, H);
  if (!m || !h) return;

  // map image
  const s = S.view.scale, res = m.resolution;
  const [mx0, my0] = w2s(m.origin[0], m.origin[1] + m.height * res);
  ctx.imageSmoothingEnabled = s * res < 2;
  if (S.mapCanvas.width) ctx.drawImage(S.mapCanvas, mx0, my0, m.width * res * s, m.height * res * s);

  drawGrid(W, H);
  drawStartLine();
  if (S.showTrail) drawTrail();
  if (st && S.lidarMode !== 2) drawLidar(st);
  if (st) drawCar(st.x, st.y, st.yaw, st.steer, st.coll ? COLORS.kerb : COLORS.yellow, 1);
  drawToolOverlay();
  drawScaleBar();
  drawCharts();
}

function drawGrid(W, H) {
  const s = S.view.scale;
  if (s < 14) return;
  const step = s > 120 ? 0.5 : 1;
  const [x0, y1] = s2w(0, 0), [x1, y0] = s2w(W, H);
  ctx.strokeStyle = 'rgba(32,37,43,0.07)'; ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil(x0 / step) * step; x < x1; x += step) { const p = Math.round(w2s(x, 0)[0]) + 0.5; ctx.moveTo(p, 0); ctx.lineTo(p, H); }
  for (let y = Math.ceil(y0 / step) * step; y < y1; y += step) { const p = Math.round(w2s(0, y)[1]) + 0.5; ctx.moveTo(0, p); ctx.lineTo(W, p); }
  ctx.stroke();
}

function drawStartLine() {
  const sp = S.map.spawn; if (!sp) return;
  const [x, y, yaw] = sp;
  const n = 10, half = 1.5, sq = (2 * half) / n;
  const nx = -Math.sin(yaw), ny = Math.cos(yaw), fx = Math.cos(yaw), fy = Math.sin(yaw);
  for (let i = 0; i < n; i++) for (let j = 0; j < 2; j++) {
    const a = -half + i * sq, b = j * sq * 0.5 - sq * 0.5;
    const cx = x + nx * a + fx * b, cy = y + ny * a + fy * b;
    ctx.fillStyle = (i + j) % 2 ? 'rgba(32,37,43,0.55)' : 'rgba(255,255,255,0.75)';
    poly([[0, 0], [sq, 0], [sq, sq * 0.5], [0, sq * 0.5]].map(([u, v]) => [cx + nx * u + fx * v, cy + ny * u + fy * v]));
    ctx.fill();
  }
}
function poly(pts) {
  ctx.beginPath();
  pts.forEach(([x, y], i) => { const [px, py] = w2s(x, y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
  ctx.closePath();
}

function speedColor(v) {
  const t = clamp(Math.abs(v) / 6, 0, 1);
  // teal (slow) -> yellow -> kerb red (fast)
  const stops = [[18, 164, 164], [245, 197, 24], [224, 68, 46]];
  const k = t < 0.5 ? 0 : 1, f = t < 0.5 ? t * 2 : (t - 0.5) * 2;
  const a = stops[k], b = stops[k + 1];
  return `rgb(${a.map((c, i) => Math.round(c + (b[i] - c) * f)).join(',')})`;
}
function drawTrail() {
  const tr = S.trail; if (tr.length < 2) return;
  ctx.lineWidth = Math.max(2, S.view.scale * 0.06); ctx.lineCap = 'round';
  let prev = w2s(tr[0][0], tr[0][1]);
  for (let i = 1; i < tr.length; i++) {
    const p = w2s(tr[i][0], tr[i][1]);
    ctx.strokeStyle = speedColor(tr[i][2]);
    ctx.globalAlpha = 0.35 + 0.65 * (i / tr.length);
    ctx.beginPath(); ctx.moveTo(prev[0], prev[1]); ctx.lineTo(p[0], p[1]); ctx.stroke();
    prev = p;
  }
  ctx.globalAlpha = 1;
}

function drawLidar(st) {
  const sc = S.hello.scan;
  const ox = st.x + sc.offset * Math.cos(st.yaw), oy = st.y + sc.offset * Math.sin(st.yaw);
  const [px, py] = w2s(ox, oy);
  const s = S.view.scale, maxR = sc.range - 0.05;
  if (S.lidarMode === 0) {
    ctx.strokeStyle = 'rgba(18,164,164,0.11)'; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i < st.scan.length; i += 2) {
      const r = Math.min(st.scan[i] / 100, 30), a = st.yaw + sc.angle_min + i * sc.inc;
      ctx.moveTo(px, py); ctx.lineTo(px + Math.cos(a) * r * s, py - Math.sin(a) * r * s);
    }
    ctx.stroke();
  }
  ctx.fillStyle = COLORS.teal;
  const dot = clamp(s * 0.04, 1.5, 4);
  for (let i = 0; i < st.scan.length; i++) {
    const r = st.scan[i] / 100; if (r >= maxR) continue;
    const a = st.yaw + sc.angle_min + i * sc.inc;
    ctx.fillRect(px + Math.cos(a) * r * s - dot / 2, py - Math.sin(a) * r * s - dot / 2, dot, dot);
  }
}

function drawCar(x, y, yaw, steer, color, alpha) {
  const car = S.hello.car, s = S.view.scale;
  const [px, py] = w2s(x, y);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.translate(px, py); ctx.rotate(-yaw);
  const L = car.length * s, Wd = car.width * s;
  if (L < 14) { // too small to read: draw an arrow marker instead
    ctx.fillStyle = color; ctx.strokeStyle = COLORS.asphalt; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-7, 6); ctx.lineTo(-4, 0); ctx.lineTo(-7, -6); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore(); return;
  }
  const wl = L * 0.2, ww = Wd * 0.22, wb = car.wheelbase * s;
  ctx.fillStyle = '#15191d';
  for (const [wx, wy, rot] of [[wb / 2, -Wd / 2, steer], [wb / 2, Wd / 2, steer], [-wb / 2, -Wd / 2, 0], [-wb / 2, Wd / 2, 0]]) {
    ctx.save(); ctx.translate(wx, wy); ctx.rotate(-rot); ctx.fillRect(-wl / 2, -ww / 2, wl, ww); ctx.restore();
  }
  ctx.fillStyle = color; ctx.strokeStyle = COLORS.asphalt; ctx.lineWidth = Math.max(1, s * 0.012);
  roundRect(-L / 2, -Wd * 0.38, L, Wd * 0.76, Math.min(L, Wd) * 0.18); ctx.fill(); ctx.stroke();
  // lidar puck + nose stripe
  ctx.fillStyle = COLORS.asphalt;
  ctx.beginPath(); ctx.arc(L * 0.18, 0, Wd * 0.16, 0, Math.PI * 2); ctx.fill();
  ctx.fillRect(L * 0.38, -Wd * 0.06, L * 0.1, Wd * 0.12);
  ctx.restore();
}
function roundRect(x, y, w, h, r) {
  ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

function drawToolOverlay() {
  const p = S.pointer;
  if (p && p.kind === 'pose') {
    const yaw = p.end ? Math.atan2(p.end[1] - p.start[1], p.end[0] - p.start[0]) : (S.st ? S.st.yaw : 0);
    drawCar(p.start[0], p.start[1], yaw, 0, COLORS.yellow, 0.55);
    if (p.end) {
      const [a, b] = w2s(p.start[0], p.start[1]), [c, d] = w2s(p.end[0], p.end[1]);
      ctx.strokeStyle = COLORS.asphalt; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); ctx.setLineDash([]);
    }
  }
  if (p && p.kind === 'measure' && p.end) {
    const [a, b] = w2s(p.start[0], p.start[1]), [c, d] = w2s(p.end[0], p.end[1]);
    const dist = Math.hypot(p.end[0] - p.start[0], p.end[1] - p.start[1]);
    ctx.strokeStyle = COLORS.kerb; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    for (const [x, y] of [[a, b], [c, d]]) { ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fillStyle = COLORS.kerb; ctx.fill(); }
    label(fmt(dist) + ' m', (a + c) / 2, (b + d) / 2 - 12);
  }
  if ((S.tool === 'wall' || S.tool === 'erase') && S.mouseW) {
    const [x, y] = w2s(S.mouseW[0], S.mouseW[1]);
    const r = parseFloat($('brush').value) * S.view.scale;
    ctx.strokeStyle = S.tool === 'wall' ? '#3a5fa8' : COLORS.kerb; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, Math.max(r, 3), 0, Math.PI * 2); ctx.stroke();
  }
}
function label(text, x, y) {
  ctx.font = '600 15px "Barlow Condensed", sans-serif';
  const w = ctx.measureText(text).width + 12;
  ctx.fillStyle = COLORS.asphalt; ctx.fillRect(x - w / 2, y - 11, w, 22);
  ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(text, x, y + 1);
}

function drawScaleBar() {
  const target = 110 / S.view.scale; // metres in ~110 px
  const nice = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50].find((v) => v >= target * 0.6) || 50;
  $('scaleBar').style.width = nice * S.view.scale + 'px';
  $('scaleLabel').textContent = nice < 1 ? `${Math.round(nice * 100)} cm` : `${nice} m`;
}

let lastChart = 0;
function drawCharts() {
  const now = performance.now();
  if (now - lastChart < 50 || !S.st) return;
  lastChart = now;
  const t = S.st.simt;
  drawChart($('chartSpeed'), [
    { pts: S.hist.speed.map((p) => [p[0], p[2]]), color: 'rgba(232,230,225,0.6)', width: 1.5, dash: [4, 3] },
    { pts: S.hist.speed.map((p) => [p[0], p[1]]), color: COLORS.yellow, width: 2 },
  ], t, { zero: true, minSpan: 1 });
  drawChart($('chartTopics'), S.plotKeys.map((k, i) => ({ pts: S.hist.topics[k] || [], color: SERIES_COLORS[i % SERIES_COLORS.length] })), t, { minSpan: 0.2 });
}

/* ================================================================ pointer */
canvas.addEventListener('contextmenu', (e) => e.preventDefault());
canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  const w = s2w(e.offsetX, e.offsetY);
  if (e.button === 1 || e.button === 2 || S.tool === 'pan') {
    S.pointer = { kind: 'pan', sx: e.offsetX, sy: e.offsetY, cx: S.view.cx, cy: S.view.cy };
    stage.classList.add('dragging');
    return;
  }
  if (S.tool === 'pose') S.pointer = { kind: 'pose', start: w, end: null };
  else if (S.tool === 'measure') S.pointer = { kind: 'measure', start: w, end: w };
  else if (S.tool === 'wall' || S.tool === 'erase') {
    S.painting = { mode: S.tool, buf: [w], last: w, lastFlush: performance.now() };
    S.pointer = { kind: 'paint' };
    previewPaint(w, w);
  }
});
canvas.addEventListener('pointermove', (e) => {
  const w = s2w(e.offsetX, e.offsetY);
  S.mouseW = w;
  $('cursorPos').textContent = `x ${fmt(w[0])}  y ${fmt(w[1])} m`;
  const p = S.pointer; if (!p) return;
  if (p.kind === 'pan') {
    S.view.cx = p.cx - (e.offsetX - p.sx) / S.view.scale;
    S.view.cy = p.cy + (e.offsetY - p.sy) / S.view.scale;
    if (Math.hypot(e.offsetX - p.sx, e.offsetY - p.sy) > 4 && S.follow) setFollow(false);
  } else if (p.kind === 'pose' || p.kind === 'measure') {
    p.end = w;
  } else if (p.kind === 'paint' && S.painting) {
    const pt = S.painting;
    previewPaint(pt.last, w);
    pt.buf.push(w); pt.last = w;
    if (performance.now() - pt.lastFlush > 70) flushPaint(false);
  }
});
canvas.addEventListener('pointerleave', () => { S.mouseW = null; $('cursorPos').textContent = ''; });
const endPointer = () => {
  const p = S.pointer; S.pointer = null; stage.classList.remove('dragging');
  if (!p) return;
  if (p.kind === 'pose') {
    const yaw = p.end && Math.hypot(p.end[0] - p.start[0], p.end[1] - p.start[1]) > 0.1
      ? Math.atan2(p.end[1] - p.start[1], p.end[0] - p.start[0]) : (S.st ? S.st.yaw : 0);
    send({ t: 'pose', x: p.start[0], y: p.start[1], yaw });
    S.trail = [];
  } else if (p.kind === 'paint') flushPaint(true);
};
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const before = s2w(e.offsetX, e.offsetY);
  S.view.scale = clamp(S.view.scale * Math.exp(-e.deltaY * 0.0015), 2, 400);
  if (!S.follow) {
    const after = s2w(e.offsetX, e.offsetY);
    S.view.cx += before[0] - after[0]; S.view.cy += before[1] - after[1];
  }
}, { passive: false });

function previewPaint(a, b) {
  const m = S.map, c = S.mapCanvas.getContext('2d');
  const r = parseFloat($('brush').value) / m.resolution;
  const toPx = ([x, y]) => [(x - m.origin[0]) / m.resolution, m.height - (y - m.origin[1]) / m.resolution];
  const [ax, ay] = toPx(a), [bx, by] = toPx(b);
  const col = S.tool === 'wall' ? COLORS.painted : COLORS.erased;
  c.strokeStyle = c.fillStyle = `rgb(${col.join(',')})`;
  c.lineWidth = r * 2; c.lineCap = 'round';
  c.beginPath(); c.moveTo(ax, ay); c.lineTo(bx, by); c.stroke();
}
function flushPaint(final) {
  const pt = S.painting; if (!pt) return;
  if (pt.buf.length) send({ t: 'paint', pts: pt.buf, r: parseFloat($('brush').value), mode: pt.mode === 'wall' ? 'wall' : 'erase' });
  pt.buf = [pt.last]; pt.lastFlush = performance.now();
  if (final) { S.painting = null; S.paintHoldUntil = performance.now() + 400; }
}

/* ================================================================ tools & buttons */
function setTool(t) {
  S.tool = t; stage.dataset.tool = t;
  for (const b of document.querySelectorAll('.tool[data-tool]')) b.classList.toggle('active', b.dataset.tool === t);
  $('brushBox').hidden = !(t === 'wall' || t === 'erase');
}
for (const b of document.querySelectorAll('.tool[data-tool]')) b.onclick = () => setTool(b.dataset.tool);
function setFollow(on) { S.follow = on; $('tglFollow').classList.toggle('on', on); }
$('tglFollow').onclick = () => setFollow(!S.follow);
const LIDAR_LABELS = ['Rayos', 'Puntos', 'Oculto'];
function setLidar(mode) {
  S.lidarMode = mode; store.set('lidar', mode);
  $('lidarLabel').textContent = LIDAR_LABELS[mode];
  $('tglLidar').classList.toggle('on', mode !== 2);
}
$('tglLidar').onclick = () => setLidar((S.lidarMode + 1) % 3);
$('tglTrail').onclick = () => { S.showTrail = !S.showTrail; $('tglTrail').classList.toggle('on', S.showTrail); };

$('btnPause').onclick = () => send({ t: 'pause', on: !(S.st && S.st.paused) });
$('btnStep').onclick = () => send({ t: 'step', n: 5 });
$('btnReset').onclick = () => { send({ t: 'reset' }); };
$('rtfSelect').onchange = (e) => send({ t: 'rtf', v: parseFloat(e.target.value) });
$('mapSelect').onchange = (e) => send({ t: 'map', name: e.target.value });
$('btnEstop').onclick = () => send({ t: 'estop', on: !(S.st && S.st.estop) });
$('chkAutoReset').onchange = (e) => send({ t: 'auto_reset', on: e.target.checked });
$('btnClearEdits').onclick = () => send({ t: 'clear_edits' });
for (const b of document.querySelectorAll('.modes button')) b.onclick = () => send({ t: 'mode', mode: b.dataset.mode });
$('brush').oninput = (e) => { $('brushOut').textContent = Math.round(e.target.value * 100) + ' cm'; };
const kbdV = $('kbdV');
kbdV.value = S.kbdV;
const showKbdV = () => { $('kbdVOut').textContent = fmt(parseFloat(kbdV.value), 1) + ' m/s'; };
kbdV.oninput = () => { S.kbdV = parseFloat(kbdV.value); store.set('kbdV', S.kbdV); showKbdV(); };
showKbdV();

/* ================================================================ keyboard */
const DRIVE_KEYS = { w: 'up', arrowup: 'up', s: 'down', arrowdown: 'down', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
const KEYCAP = { up: 'W', down: 'S', left: 'A', right: 'D' };
function isTyping(e) { const t = e.target; return t && (t.tagName === 'SELECT' || (t.tagName === 'INPUT' && t.type !== 'range' && t.type !== 'checkbox')); }
function capEl(name) { return [...document.querySelectorAll('#keysBox kbd')].find((k) => k.textContent === name); }

let rosHintShown = 0;
window.addEventListener('keydown', (e) => {
  if (isTyping(e) || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (DRIVE_KEYS[k]) {
    e.preventDefault();
    S.keys.add(DRIVE_KEYS[k]);
    const cap = capEl(KEYCAP[DRIVE_KEYS[k]]); if (cap) cap.classList.add('down');
    if (S.st && S.st.mode === 'ros' && performance.now() - rosHintShown > 4000) {
      rosHintShown = performance.now();
      flashHelp('Estás en modo ROS: el teclado no conduce. Pulsa M para cambiar de modo.');
    }
    return;
  }
  if (e.repeat) return;
  if (k === ' ') { e.preventDefault(); $('btnPause').click(); }
  else if (k === 'r') send({ t: 'reset' });
  else if (k === 'x') $('btnEstop').click();
  else if (k === 'm') { const order = ['ros', 'mix', 'kbd']; const cur = S.st ? S.st.mode : 'mix'; send({ t: 'mode', mode: order[(order.indexOf(cur) + 1) % 3] }); }
  else if (k === 'f') setFollow(!S.follow);
  else if (k === 'l') $('tglLidar').click();
  else if (k === 't') $('tglTrail').click();
  else if (k === '.') { if (S.st && S.st.paused) send({ t: 'step', n: 5 }); }
  else if ('12345'.includes(k)) setTool(['pan', 'pose', 'wall', 'erase', 'measure'][+k - 1]);
});
window.addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (DRIVE_KEYS[k]) { S.keys.delete(DRIVE_KEYS[k]); const cap = capEl(KEYCAP[DRIVE_KEYS[k]]); if (cap) cap.classList.remove('down'); }
});
window.addEventListener('blur', () => { S.keys.clear(); document.querySelectorAll('kbd.down').forEach((k) => k.classList.remove('down')); });

function flashHelp(text) {
  const el = $('modeHelp'); el.textContent = text; el.style.color = 'var(--yellow)';
  setTimeout(() => { el.style.color = ''; }, 2500);
}

// Teleop: 30 Hz while a key is held; steering ramps in and recentres like a real servo
let lastTick = performance.now();
setInterval(() => {
  const now = performance.now(), dt = (now - lastTick) / 1000; lastTick = now;
  if (!S.hello || !S.st) return;
  const smax = S.hello.car.s_max * 0.95;
  const target = ((S.keys.has('left') ? 1 : 0) - (S.keys.has('right') ? 1 : 0)) * smax;
  const rate = target === 0 ? 2.5 : 1.6;
  const d = target - S.steer;
  S.steer += clamp(d, -rate * dt, rate * dt);
  if (Math.abs(S.steer) < 1e-3 && target === 0) S.steer = 0;
  let v = 0;
  if (S.keys.has('up')) v = S.kbdV;
  else if (S.keys.has('down')) v = -Math.min(S.kbdV, 3);
  const active = S.keys.size > 0 || S.steer !== 0;
  if (S.st.mode !== 'ros' && (active || S.lastSentActive)) send({ t: 'kbd', v, s: S.steer });
  S.lastSentActive = active;
}, 33);

/* ================================================================ go */
setTool('pan');
setLidar(S.lidarMode);
window.addEventListener('resize', () => { if (!S.follow) return; });
connect();
requestAnimationFrame(draw);
