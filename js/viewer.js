// .tlog viewer page: load → decode → search/filter → inspect → 3D replay.
import { registerDefs, pyStr, dumpTime, enumName } from './mavlink.js';
import { MAVLINK_DEFS_FULL } from './mavlink-defs-full.js';
import { parseTlog } from './tlog.js';
import { CATS, catOf, colorOf, summary, inspectorHtml, esc, installTips, filterPills, direction } from './logcommon.js';
import { buildReplay } from './replay.js';

registerDefs(MAVLINK_DEFS_FULL);
installTips();

const $ = (s, el = document) => el.querySelector(s);
const MAX_BYTES = 5 * 1024 * 1024;
const ROW = 22;

const PRESETS = [
  { label: 'Arming & disarming', q: '/COMPONENT_ARM_DISARM|arming motors|disarming motors/' },
  { label: 'Flight-mode changes', q: '/DO_SET_MODE|SET_MODE|mode change/' },
  { label: 'State timeline (HEARTBEAT changes)', q: '', type: 'HEARTBEAT' },
  { label: 'Failsafes, errors & warnings', q: '/failsafe|\\[(emergency|alert|critical|error|warning)\\]|_FAILED|_DENIED|MAV_MISSION_ERROR/' },
  { label: 'Pre-arm / arming checks', q: '/prearm|arm:/' },
  { label: 'Flight plan upload', q: '/MISSION_COUNT|MISSION_ITEM_INT|MISSION_ITEM |MISSION_ACK|MISSION_CLEAR|flight plan/' },
  { label: 'Mission progress', q: '/MISSION_ITEM_REACHED|mission: /' },
  { label: 'Take-off & landing', q: '/TAKEOFF|NAV_LAND|takeoff|landing|land complete/' },
  { label: 'Return to launch', q: '/RETURN_TO_LAUNCH|\\bRTL\\b/' },
  { label: 'Parameter changes', q: '', type: 'PARAM_SET' },
  { label: 'Camera & photos', q: '/DIGICAM|CAMERA_FEEDBACK|IMAGE_CAPTURED|CAMERA_TRIGGER/' },
  { label: 'Home / take-off location', q: '/HOME_POSITION|GPS_GLOBAL_ORIGIN|SET_HOME|origin set/' },
  { label: 'Firmware & hardware identity', q: '/AUTOPILOT_VERSION|Ardu(Copter|Plane|Rover|Sub)|PX4|ChibiOS|Frame:/' },
  { label: 'GPS & EKF', q: '/\\bGPS \\d|EKF/' },
];

// ---------------------------------------------------------------------------- state
let all = [];
let filtered = [];
let stats = null;
let fileName = '';
let view = 'decoded';
let selected = -1;      // index into `filtered`
let current = -1;       // replay-synced row
let replay = null;
let scene = null;
const filterOn = Object.fromEntries(CATS.map((c) => [c.key, c.on]));

const vlist = $('#vlist'), spacer = $('#vspacer');
const updatePills = filterPills($('#filters'), filterOn, () => applyFilters());

// ---------------------------------------------------------------------------- loading
$('#file').addEventListener('change', (e) => { const f = e.target.files[0]; if (f) loadFile(f); e.target.value = ''; });
$('#btn-upload').addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); $('#file').click(); } });
$('#btn-upload').tabIndex = 0;

let dragDepth = 0;
window.addEventListener('dragenter', (e) => { e.preventDefault(); dragDepth++; document.body.classList.add('dragging'); });
window.addEventListener('dragleave', () => { if (--dragDepth <= 0) { dragDepth = 0; document.body.classList.remove('dragging'); } });
window.addEventListener('dragover', (e) => e.preventDefault());
window.addEventListener('drop', (e) => {
  e.preventDefault();
  dragDepth = 0;
  document.body.classList.remove('dragging');
  const f = e.dataTransfer.files[0];
  if (f) loadFile(f);
});

function showError(msg) {
  const el = $('#drop-err');
  el.textContent = msg;
  el.hidden = false;
  $('#drop').hidden = false;
}

async function loadFile(file) {
  $('#drop-err').hidden = true;
  if (file.size > MAX_BYTES) {
    showError(`"${file.name}" is ${(file.size / 1048576).toFixed(1)} MB. The limit is 5 MB for now.`);
    return;
  }
  if (!file.size) { showError('That file is empty.'); return; }
  $('#v-stat').textContent = `Decoding ${file.name}…`;
  await new Promise((r) => setTimeout(r, 20));
  let res;
  try {
    res = parseTlog(await file.arrayBuffer());
  } catch (err) {
    console.error(err);
    showError(`Could not read "${file.name}": ${err.message}`);
    return;
  }
  if (!res.messages.length) {
    showError(`No MAVLink messages found in "${file.name}". Is it a telemetry log (.tlog)? Dataflash logs (.bin) use a different format.`);
    $('#v-stat').textContent = 'No file loaded';
    return;
  }
  all = res.messages;
  stats = res.stats;
  fileName = file.name;
  selected = -1; current = -1;
  closeDetail();
  $('#drop').hidden = true;
  vlist.hidden = false;
  $('#filters').hidden = false;
  for (const id of ['q', 'type', 'src', 'preset']) $('#' + id).disabled = false;
  $('#v-export').disabled = false;
  $('#q').value = '';
  fillSelects();
  replay = buildReplay(all, stats);
  renderSummary();
  setupReplay();
  applyFilters();
}

function fillSelects() {
  const types = [...stats.types.entries()].sort((a, b) => a[0].localeCompare(b[0]));
  $('#type').innerHTML = `<option value="">All message types (${types.length})</option>` +
    types.map(([n, c]) => `<option value="${esc(n)}">${esc(n)} (${c.toLocaleString()})</option>`).join('');
  const srcs = [...stats.sources.values()].sort((a, b) => b.count - a.count);
  $('#src').innerHTML = '<option value="">All sources</option>' + srcs.map((s) => `<option value="${s.key}">${s.key} · ${esc(sourceLabel(s))} (${s.count.toLocaleString()})</option>`).join('');
  $('#preset').innerHTML = '<option value="">Presets…</option>' + PRESETS.map((p, i) => `<option value="${i}">${esc(p.label)}</option>`).join('');
}

function sourceLabel(s) {
  if (s.type === null) return s.sysid >= 200 ? 'GCS / script (no heartbeat)' : 'no heartbeat';
  if (s.type === 6) return 'GCS';
  const t = (enumName('MAV_TYPE', s.type) || 'type ' + s.type).replace('MAV_TYPE_', '');
  const ap = { 3: 'ArduPilot', 12: 'PX4', 8: '' }[s.autopilot] ?? (enumName('MAV_AUTOPILOT', s.autopilot) || '').replace('MAV_AUTOPILOT_', '');
  return `${t}${ap ? ' · ' + ap : ''}`;
}

// ---------------------------------------------------------------------------- summary
const fmtDur = (ms) => {
  const s = Math.round(ms / 1000), h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h ? `${h}h ${m}m ${s % 60}s` : m ? `${m}m ${s % 60}s` : `${s}s`;
};

function renderSummary() {
  const vehicles = [...stats.sources.values()].filter((s) => s.type !== null && s.type !== 6);
  const others = [...stats.sources.values()].filter((s) => !(s.type !== null && s.type !== 6));
  const fw = all.find((m) => m.name === 'STATUSTEXT' && /^(Ardu\w+|PX4|APM:\w+|ArduPilot)\b.*\d+\.\d+/.test(m.values.text));
  let fwText = fw ? fw.values.text : '';
  if (!fwText) {
    const av = all.find((m) => m.name === 'AUTOPILOT_VERSION');
    if (av) { const v = av.values.flight_sw_version; fwText = `v${(v >>> 24) & 255}.${(v >>> 16) & 255}.${(v >>> 8) & 255}`; }
  }
  const items = [
    ['Start', new Date(stats.start).toLocaleString()],
    ['Duration', fmtDur(stats.end - stats.start)],
    ['Vehicle', vehicles.map((s) => `${s.key} ${sourceLabel(s)}`).join(', ') || '—'],
    ['GCS / other', others.map((s) => s.key).join(', ') || '—'],
    ['Firmware', fwText || '—'],
  ];
  if (replay) {
    items.push(['Home', `${replay.home.lat.toFixed(6)}, ${replay.home.lon.toFixed(6)}`]);
    items.push(['Max alt', `${replay.maxU.toFixed(1)} m`]);
    items.push(['Distance', replay.dist > 1000 ? `${(replay.dist / 1000).toFixed(2)} km` : `${replay.dist.toFixed(0)} m`]);
  }
  const integ = stats.crcErrors + stats.badBytes + stats.unknown;
  items.push(['Integrity', integ ? `${stats.crcErrors} CRC err · ${stats.badBytes} bad bytes · ${stats.unknown} unknown` : 'all frames valid', integ ? 'warn' : 'good']);
  $('#v-summary').innerHTML = items.map(([k, v, c]) => `<div class="kv ${c || ''}"><span>${k}</span><b title="${esc(v)}">${esc(v)}</b></div>`).join('');
  $('#v-summary').hidden = false;
}

// ---------------------------------------------------------------------------- search
function compileQuery(q) {
  // tokens: -?/regex with spaces/  |  -?"a phrase"  |  -?word
  const terms = [];
  for (const mt of q.matchAll(/(-?)(?:\/((?:\\.|[^\\/])+)\/|"([^"]*)"|(\S+))/g)) {
    const neg = mt[1] === '-';
    if (mt[2] !== undefined) {
      try { terms.push({ re: new RegExp(mt[2], 'i'), neg }); } catch { return null; }
    } else {
      const s = (mt[3] ?? mt[4]).toLowerCase();
      if (s) terms.push({ s, neg });
    }
  }
  return terms;
}

const stripTags = (h) => h.replace(/<[^>]+>/g, '').replace(/&amp;/g, '&').replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"');
function haystack(m) {
  return m._h ??= `${m.name} ${pyStr(m)} ${stripTags(summary(m))}`.toLowerCase();
}

function applyFilters() {
  if (!all.length) return;
  const terms = compileQuery($('#q').value);
  $('#q').classList.toggle('invalid', terms === null);
  if (terms === null) return;
  const type = $('#type').value, src = $('#src').value;
  const counts = Object.fromEntries(CATS.map((c) => [c.key, 0]));
  const keep = [];
  for (const m of all) {
    if (type && m.name !== type) continue;
    if (src && `${m.sysid}:${m.compid}` !== src) continue;
    if (terms.length) {
      const h = haystack(m);
      let ok = true;
      for (const t of terms) {
        const hit = t.re ? t.re.test(h) : h.includes(t.s);
        if (hit === t.neg) { ok = false; break; }
      }
      if (!ok) continue;
    }
    const c = catOf(m);
    counts[c]++;
    if (filterOn[c]) keep.push(m);
  }
  const prevSel = filtered[selected];
  filtered = keep;
  selected = prevSel ? filtered.indexOf(prevSel) : -1;
  current = -1;
  updatePills(counts);
  $('#v-stat').textContent = `${fileName} · ${(stats.bytes / 1048576).toFixed(2)} MB · ${all.length.toLocaleString()} msgs · ${filtered.length.toLocaleString()} shown`;
  $('#vempty').hidden = filtered.length > 0;
  spacer.style.height = filtered.length * ROW + 'px';
  resetRowWidth();
  if (selected < 0) vlist.scrollTop = 0;
  renderRows(true);
}

let qTimer;
$('#q').addEventListener('input', () => { clearTimeout(qTimer); qTimer = setTimeout(applyFilters, 180); $('#preset').value = ''; });
$('#type').addEventListener('change', applyFilters);
$('#src').addEventListener('change', applyFilters);
$('#preset').addEventListener('change', (e) => {
  const p = PRESETS[e.target.value];
  if (!p) return;
  $('#q').value = p.q;
  const typeSel = $('#type');
  if (p.type && ![...typeSel.options].some((o) => o.value === p.type)) {
    typeSel.insertAdjacentHTML('beforeend', `<option value="${esc(p.type)}">${esc(p.type)} (0)</option>`);
  }
  typeSel.value = p.type ?? '';
  applyFilters();
});
document.querySelectorAll('.seg button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.seg button').forEach((x) => x.classList.toggle('on', x === b));
  view = b.dataset.view;
  resetRowWidth();
  renderRows(true);
}));

// ---------------------------------------------------------------------------- virtual list
let rendered = { from: -1, to: -1 };
let rowWidth = 0;
function resetRowWidth() { rowWidth = 0; vlist.style.removeProperty('--rw'); }
function rowHtml(m, i) {
  const cls = `vrow${i === selected ? ' sel' : ''}${i === current ? ' cur' : ''}`;
  const top = i * ROW;
  if (view === 'dump') {
    return `<div class="${cls} dump" style="top:${top}px" data-i="${i}"><span class="t">${dumpTime(m.t)}:</span> ${esc(pyStr(m))}</div>`;
  }
  const dir = direction(m);
  return `<div class="${cls}" style="top:${top}px" data-i="${i}"><span class="t">${dumpTime(m.t).slice(11)}</span><span class="d ${dir}">${dir === 'up' ? '↑' : '↓'}</span><span class="src">${m.sysid}:${m.compid}</span><span class="n" style="--c:${colorOf(m)}">${m.name}</span><span class="s">${summary(m)}</span></div>`;
}

function renderRows(force = false) {
  const h = vlist.clientHeight;
  const from = Math.max(0, Math.floor(vlist.scrollTop / ROW) - 10);
  const to = Math.min(filtered.length, Math.ceil((vlist.scrollTop + h) / ROW) + 10);
  if (!force && from === rendered.from && to === rendered.to) return;
  rendered = { from, to };
  let html = '';
  for (let i = from; i < to; i++) html += rowHtml(filtered[i], i);
  vlist.querySelectorAll('.vrow').forEach((n) => n.remove());
  spacer.insertAdjacentHTML('afterend', html);
  // grow the shared row width to the widest row seen so far (never shrinks while scrolling)
  let w = rowWidth;
  for (const n of vlist.querySelectorAll('.vrow')) w = Math.max(w, n.scrollWidth);
  if (w !== rowWidth) { rowWidth = w; vlist.style.setProperty('--rw', w + 'px'); }
}
let scrollRaf = 0;
vlist.addEventListener('scroll', () => {
  if (performance.now() > ignoreScrollUntil) lastUserScroll = performance.now();
  if (!scrollRaf) scrollRaf = requestAnimationFrame(() => { scrollRaf = 0; renderRows(); });
});
new ResizeObserver(() => renderRows(true)).observe(vlist);

vlist.addEventListener('click', (e) => {
  const r = e.target.closest('.vrow');
  if (!r) return;
  select(+r.dataset.i, true);
});
vlist.addEventListener('keydown', (e) => {
  if (!filtered.length) return;
  if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
    e.preventDefault();
    select(Math.min(filtered.length - 1, Math.max(0, selected + (e.key === 'ArrowDown' ? 1 : -1))), true);
    ensureVisible(selected);
  } else if (e.key === 'Escape') closeDetail();
});

let ignoreScrollUntil = 0;
function ensureVisible(i, center = false) {
  ignoreScrollUntil = performance.now() + 150;
  const top = i * ROW, h = vlist.clientHeight;
  if (center) vlist.scrollTop = Math.max(0, top - h / 2 + ROW / 2);
  else if (top < vlist.scrollTop) vlist.scrollTop = top;
  else if (top + ROW > vlist.scrollTop + h) vlist.scrollTop = top + ROW - h;
}

function select(i, seek) {
  selected = i;
  const m = filtered[i];
  if (!m) return;
  $('#detail-body').innerHTML = inspectorHtml(m);
  $('#detail').hidden = false;
  renderRows(true);
  if (seek && replay && simOpen) {
    replay.seek(m.t);
    scene?.snap();
  }
}
function closeDetail() { $('#detail').hidden = true; selected = -1; renderRows(true); }
$('#detail-close').addEventListener('click', closeDetail);

$('#v-export').addEventListener('click', () => {
  const text = filtered.map((m) => `${dumpTime(m.t)}: ${pyStr(m)}`).join('\n') + '\n';
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
  a.download = fileName.replace(/\.tlog$/i, '') + '-filtered.txt';
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
});

// ---------------------------------------------------------------------------- replay
let simOpen = false;
window.tlogViewer = { get replay() { return replay; }, get scene() { return scene; }, toggleSim: (o) => toggleSim(o), updateReplayUi };
let lastUserScroll = 0;
const btnSim = $('#btn-sim');

function setupReplay() {
  btnSim.disabled = !replay;
  btnSim.dataset.tip = replay
    ? 'Replay the flight in 3D: path, attitude, flight mode, arming and motors over time.'
    : 'This log has no GLOBAL_POSITION_INT or GPS position data, so the flight cannot be replayed.';
  if (!replay) { if (simOpen) toggleSim(false); return; }

  replay.speed = +$('#r-speed').value;
  const hs = Math.min(0.32, 9 / Math.max(1, replay.maxR));
  const vs = Math.max(hs, Math.min(0.32, 4.5 / Math.max(1, replay.maxU)));
  replay.scales = { hs, vs };
  const cell = 2.5 / hs;
  $('#r-scale').textContent = `Grid square ≈ ${cell >= 100 ? Math.round(cell) : cell.toFixed(1)} m${vs > hs * 1.05 ? ` · height ×${(vs / hs).toFixed(1)}` : ''} · rendered as a quadcopter`;
  replay.addEventListener('photo', () => { if (scene) scene.flash = 1; });
  replay.addEventListener('end', updatePlayBtn);

  // timeline markers
  const span = replay.duration || 1;
  const marks = replay.events.filter((e) => ['mode', 'arm', 'disarm', 'crit', 'photo'].includes(e.kind) || e.kind === 'warn').slice(0, 400);
  $('#r-marks').innerHTML = marks.map((e) => `<i class="mk ${e.kind}" style="left:${((e.t - replay.t0) / span) * 100}%" data-tip="${esc(dumpTime(e.t).slice(11))} · ${esc(e.label)}" data-t="${e.t}"></i>`).join('');

  if (scene) {
    scene.sim = replay;
    scene.setScale(hs, vs);
    scene.setTrack(replay.track);
    scene.snap();
  }
  updateReplayUi(true);
}

$('#r-marks').addEventListener('click', (e) => {
  const mk = e.target.closest('.mk');
  if (!mk || !replay) return;
  replay.seek(+mk.dataset.t);
  scene?.snap();
});

async function ensureScene() {
  if (scene || !replay) return;
  try {
    const { DroneScene } = await import('./scene.js');
    scene = new DroneScene($('#replay-host'), replay, $('#r-tooltip'), { hScale: replay.scales.hs, vScale: replay.scales.vs });
    window.droneScene = scene;
    scene.setTrack(replay.track);
    scene.follow = $('#r-follow').checked;
    scene.snap();
  } catch (err) {
    console.error(err);
    $('#replay-host').innerHTML = '<div class="webgl-error">3D view unavailable (WebGL could not start).</div>';
  }
}

async function toggleSim(open = !simOpen) {
  simOpen = open && !!replay;
  $('#vmain').classList.toggle('sim-open', simOpen);
  $('#replay').setAttribute('aria-hidden', String(!simOpen));
  btnSim.setAttribute('aria-expanded', String(simOpen));
  btnSim.querySelector('span').textContent = simOpen ? 'Hide simulation' : 'View simulation';
  btnSim.classList.toggle('on', simOpen);
  if (simOpen) {
    // wait until the panel has started to open so the renderer gets a real size
    await new Promise((r) => requestAnimationFrame(() => setTimeout(r, 60)));
    await ensureScene();
    scene?.resize();
    scene?.renderer.setAnimationLoop(() => scene.frame());
    if (selected >= 0 && filtered[selected]) { replay.seek(filtered[selected].t); scene?.snap(); }
  } else {
    replay && (replay.playing = false);
    updatePlayBtn();
    setTimeout(() => { if (!simOpen) scene?.renderer.setAnimationLoop(null); }, 500);
  }
}
btnSim.addEventListener('click', () => toggleSim());

$('#r-follow').addEventListener('change', (e) => { if (scene) scene.follow = e.target.checked; });
$('#r-speed').addEventListener('change', (e) => { if (replay) replay.speed = +e.target.value; });
$('#r-play').addEventListener('click', () => {
  if (!replay) return;
  if (!replay.playing && replay.time >= replay.t1) { replay.seek(replay.t0); scene?.snap(); }
  replay.playing = !replay.playing;
  updatePlayBtn();
});
function updatePlayBtn() {
  const playing = !!replay?.playing;
  $('#r-play').classList.toggle('playing', playing);
  $('#r-play').setAttribute('aria-label', playing ? 'Pause' : 'Play');
}
let seeking = false;
$('#r-seek').addEventListener('input', (e) => {
  if (!replay) return;
  seeking = true;
  replay.seek(replay.t0 + (e.target.value / 1000) * replay.duration);
  scene?.snap();
});
$('#r-seek').addEventListener('change', () => { seeking = false; });

const clock = (ms) => { const s = Math.max(0, Math.floor(ms / 1000)); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; };
const rhud = Object.fromEntries([...document.querySelectorAll('#rhud dd, .r-hud .hud-foot')].map((d) => [d.dataset.k, d]));
function setR(k, text, cls = '') {
  const el = rhud[k];
  if (el.textContent !== text) el.textContent = text;
  for (const c of ['good', 'warn', 'bad']) el.classList.toggle(c, c === cls);
}

let hudT = 0, syncT = 0;
function updateReplayUi(force) {
  if (!replay) return;
  const r = replay;
  if (!seeking) $('#r-seek').value = String(Math.round(((r.time - r.t0) / (r.duration || 1)) * 1000));
  $('#r-time').textContent = `${clock(r.time - r.t0)} / ${clock(r.duration)}`;
  const now = performance.now();
  if (force || now - hudT > 120) {
    hudT = now;
    setR('time', `T+${clock(r.time - r.t0)} · ${dumpTime(r.time).slice(11, 19)}`);
    setR('mode', r.modeLabel);
    setR('armed', r.armed ? (r.landed ? 'ARMED' : 'ARMED · flying') : 'Disarmed', r.armed ? 'warn' : '');
    setR('alt', `${r.pos.u.toFixed(1)} m AGL`);
    setR('gs', `${r.groundSpeed.toFixed(1)} m/s`);
    setR('batt', r.batt ? `${r.batt.v.toFixed(2)} V${r.batt.pct >= 0 ? ' · ' + r.batt.pct + '%' : ''}` : '—');
    setR('gps', `${(enumName('GPS_FIX_TYPE', r.gps.fix) || '—').replace('GPS_FIX_TYPE_', '')} · ${r.gps.sats} sats`, r.ekfPos ? 'good' : 'warn');
    setR('pos', `${r.lat.toFixed(6)}, ${r.lon.toFixed(6)} · ${(r.home.alt + r.pos.u).toFixed(1)} m`);
  }
  if ($('#r-sync').checked && (force || now - syncT > 200) && filtered.length) {
    syncT = now;
    let lo = 0, hi = filtered.length - 1, idx = 0;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (filtered[mid].t <= r.time) { idx = mid; lo = mid + 1; } else hi = mid - 1; }
    if (idx !== current) {
      current = idx;
      if (now - lastUserScroll > 1500) ensureVisible(idx, true);
      renderRows(true);
    }
  }
}

let lastFrame = performance.now();
function loop(now) {
  const dt = Math.min(200, now - lastFrame);
  lastFrame = now;
  if (replay && simOpen) {
    replay.tick(dt);
    updateReplayUi(false);
  }
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

// ---------------------------------------------------------------------------- ?log=<url>
// Open a log straight from a link, e.g. tlog.html?log=https://example.org/flight.tlog&sim=1
// (the server must allow CORS; the same 5 MB limit applies).
(async () => {
  const params = new URLSearchParams(location.search);
  const url = params.get('log');
  if (!url) return;
  try {
    const res = await fetch(url);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const blob = await res.blob();
    const name = decodeURIComponent(new URL(url, location.href).pathname.split('/').pop() || 'remote.tlog');
    await loadFile(new File([blob], name));
    if (params.get('sim') === '1' && replay) {
      await toggleSim(true);
      const at = parseFloat(params.get('t'));
      if (at >= 0 && at <= 1) { replay.seek(replay.t0 + at * replay.duration); scene?.snap(); }
    }
  } catch (err) {
    showError(`Could not load ${url}: ${err.message}`);
  }
})();
