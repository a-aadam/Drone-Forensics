// UI wiring: pilot actions, MAVLink log window, vehicle HUD.
import { Simulator, MODES, PLANS, EDITABLE_PARAMS, modeName, HOME } from './sim.js';
import { pyStr, dumpTime, enumName, buildTlog } from './mavlink.js';
import { CATS, catOf, colorOf, summary, inspectorHtml, esc, installTips, filterPills } from './logcommon.js';

const sim = new Simulator();
window.droneSim = sim; // handy for poking around in the console

const $ = (s, el = document) => el.querySelector(s);

// ---------------------------------------------------------------------------- 3D
import('./scene.js')
  .then(({ DroneScene }) => {
    const scene = window.droneScene = new DroneScene($('#canvas-host'), sim, $('#tooltip'));
    $('#opt-follow').addEventListener('change', (e) => { scene.follow = e.target.checked; });
    scene.controls.autoRotate = $('#opt-rotate').checked;
    $('#opt-rotate').addEventListener('change', (e) => { scene.controls.autoRotate = e.target.checked; });
    $('#opt-reset').addEventListener('click', () => scene.resetView());
  })
  .catch((err) => { console.error(err); $('#webgl-error').hidden = false; });

// ---------------------------------------------------------------------------- log
const filterOn = Object.fromEntries(CATS.map((c) => [c.key, c.on]));
const counts = Object.fromEntries(CATS.map((c) => [c.key, 0]));
let view = 'decoded';
let items = [];        // every message + annotation of this session
let pending = [];
let bytes = 0;
let stick = true;

const logEl = $('#log');
const filtersEl = $('#filters');

const updatePills = filterPills(filtersEl, filterOn, () => rebuildLog());
installTips();

document.querySelectorAll('.seg button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.seg button').forEach((x) => x.classList.toggle('on', x === b));
  view = b.dataset.view;
  rebuildLog();
}));

function rowHtml(m) {
  const t = dumpTime(m.t).slice(11);
  if (m.annotation) {
    return view === 'dump'
      ? `<div class="row note dump"><div class="row-line"><span class="t"># ${t}</span> <span class="s">${esc(m.text)}</span></div></div>`
      : `<div class="row note ${m.kind}"><div class="row-line"><span class="t">${t.slice(0, 8)}</span><span class="d">·</span><span class="n" style="--c:#b9a5ff">GCS</span><span class="s">${esc(m.text)}</span></div></div>`;
  }
  if (view === 'dump') {
    return `<div class="row dump"><div class="row-line"><span class="t">${dumpTime(m.t)}:</span> ${esc(pyStr(m))}</div></div>`;
  }
  return `<div class="row"><div class="row-line" title="${m.dir === 'up' ? 'GCS → vehicle' : 'vehicle → GCS'} · ${m.sysid}:${m.compid}"><span class="t">${t.slice(0, 8)}</span><span class="d ${m.dir}">${m.dir === 'up' ? '↑' : '↓'}</span><span class="n" style="--c:${colorOf(m)}">${m.name}</span><span class="s">${summary(m)}</span></div></div>`;
}

function passes(m) { return filterOn[catOf(m)] ?? true; }

function appendRows(list, animate) {
  const frag = document.createElement('div');
  frag.innerHTML = list.map(rowHtml).join('');
  const nodes = [...frag.children];
  nodes.forEach((n, i) => { n._m = list[i]; if (animate) n.classList.add('new'); });
  logEl.append(...nodes);
  while (logEl.children.length > 700) logEl.firstElementChild.remove();
}

function rebuildLog() {
  logEl.innerHTML = '';
  if (!items.length) { logEl.append(emptyEl); return; }
  appendRows(items.filter(passes).slice(-500), false);
  logEl.scrollTop = logEl.scrollHeight;
  stick = true;
  $('#log-jump').hidden = true;
}

const emptyEl = $('#log-empty');
sim.addEventListener('msg', (e) => {
  const m = e.detail;
  items.push(m);
  if (items.length > 30000) items = items.slice(-20000);
  if (!m.annotation) { bytes += m.frame.length; counts[catOf(m)] = (counts[catOf(m)] || 0) + 1; }
  if (passes(m)) pending.push(m);
});

function flush() {
  if (pending.length) {
    if (emptyEl.isConnected) emptyEl.remove();
    appendRows(pending, true);
    pending = [];
    if (stick) logEl.scrollTop = logEl.scrollHeight;
    else $('#log-jump').hidden = false;
  }
  requestAnimationFrame(flush);
}
requestAnimationFrame(flush);

logEl.addEventListener('scroll', () => {
  stick = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 30;
  if (stick) $('#log-jump').hidden = true;
});
$('#log-jump').addEventListener('click', () => { logEl.scrollTop = logEl.scrollHeight; });
logEl.addEventListener('click', (e) => {
  if (e.target.closest('.inspect')) return;
  const row = e.target.closest('.row');
  if (!row || !row._m || row._m.annotation) return;
  const open = row.querySelector('.inspect');
  if (open) { open.remove(); row.classList.remove('open'); return; }
  row.insertAdjacentHTML('beforeend', inspectorHtml(row._m));
  row.classList.add('open');
  stick = false;
});

$('#log-clear').addEventListener('click', () => {
  items = [];
  sim.entries = [];
  bytes = 0;
  for (const k in counts) counts[k] = 0;
  updatePills(counts);
  rebuildLog();
});

$('#log-tlog').addEventListener('click', () => {
  const frames = sim.entries.filter((m) => m.frame);
  if (!frames.length) return;
  const blob = new Blob([buildTlog(frames)], { type: 'application/octet-stream' });
  const a = document.createElement('a');
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  a.download = `drone-session-${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.tlog`;
  a.href = URL.createObjectURL(blob);
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
});

setInterval(() => {
  const n = sim.entries.length;
  $('#log-stat').textContent = `${n.toLocaleString()} msgs · ${bytes > 4096 ? (bytes / 1024).toFixed(1) + ' KB' : bytes + ' B'}`;
  $('#log-tlog').disabled = !n;
  updatePills(counts);
}, 1000);

// ---------------------------------------------------------------------------- actions
const I = {
  power: 'M12 3v8M6.3 7.2a8 8 0 1 0 11.4 0',
  reboot: 'M3 12a9 9 0 1 0 3-6.7M3 4v5h5',
  plan: 'M6 21a2 2 0 1 0 0-4 2 2 0 0 0 0 4zM18 7a2 2 0 1 0 0-4 2 2 0 0 0 0 4zM6 17V9a3 3 0 0 1 3-3h7M18 7v8a3 3 0 0 1-3 3H8',
  mode: 'M4 7h9M17 7h3M4 17h3M11 17h9M15 5v4M9 15v4',
  param: 'M12 15a3 3 0 1 0 0-6 3 3 0 0 0 0 6zM19 12h2M3 12h2M12 3v2M12 19v2M17 7l1.4-1.4M5.6 18.4 7 17M17 17l1.4 1.4M5.6 5.6 7 7',
  arm: 'M12 14a2 2 0 1 0 0-4 2 2 0 0 0 0 4zM12 10V3M12 14v7M10 12H3M14 12h7',
  takeoff: 'M12 18V5M6 11l6-6 6 6M5 21h14',
  mission: 'M7 4l13 8-13 8z',
  rtl: 'M3 11l9-7 9 7M5 10v10h14V10M10 20v-6h4v6',
  land: 'M12 3v13M6 10l6 6 6-6M5 21h14',
  camera: 'M4 8h3l2-3h6l2 3h3v11H4zM12 17a3.5 3.5 0 1 0 0-7 3.5 3.5 0 0 0 0 7z',
  radio: 'M4.9 19.1a10 10 0 0 1 0-14.2M19.1 4.9a10 10 0 0 1 0 14.2M8 16a5 5 0 0 1 0-8M16 8a5 5 0 0 1 0 8M3 3l18 18',
  battery: 'M3 8h15v8H3zM21 11v2M6 11v2',
  stop: 'M8 3h8l5 5v8l-5 5H8l-5-5V8zM9 9l6 6M15 9l-6 6',
  clear: 'M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13',
};
const svg = (d) => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="${d}"/></svg>`;

const online = () => sim.powered && sim.booted && sim.gcsConnected && sim.linkUp;
const idle = () => online() && !sim.busy;

const GROUPS = [
  {
    title: 'Power & setup',
    actions: [
      {
        id: 'power', icon: 'power', title: () => (sim.powered ? 'Power off' : 'Power on'), msg: () => (sim.powered ? 'link goes silent' : 'boot · HEARTBEAT · STATUSTEXT'),
        on: () => sim.powered, enabled: () => !sim.armed && (!sim.powered || !sim.busy || sim.busy === 'Booting'),
        run: () => (sim.powered ? sim.powerOff() : sim.powerOn()),
        help: 'Connect / disconnect the flight battery. Booting produces the firmware banner, sensor init and EKF STATUSTEXTs; power-off produces no message at all — only a gap.',
      },
      {
        id: 'reboot', icon: 'reboot', title: 'Reboot autopilot', msg: 'COMMAND_LONG · 246', enabled: () => idle(),
        run: () => sim.reboot(), help: 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (param1=1). Refused while armed.',
      },
      {
        id: 'param', icon: 'param', title: 'Set parameter', msg: 'PARAM_SET → PARAM_VALUE', enabled: () => idle(),
        extra: () => `<div class="row2"><select id="param-id" aria-label="Parameter">${EDITABLE_PARAMS.map((p) => `<option>${p}</option>`).join('')}</select><input id="param-val" type="number" step="any" aria-label="Value"></div>`,
        run: () => {
          const id = $('#param-id').value, val = parseFloat($('#param-val').value);
          if (Number.isFinite(val)) sim.setParam(id, val);
        },
        help: 'Changes a configuration parameter. The vehicle echoes PARAM_VALUE to confirm — a key forensic trace of tampering (e.g. RTL_ALT, FENCE_ENABLE).',
      },
      {
        id: 'plan', icon: 'plan', title: 'Upload flight plan', msg: 'MISSION_COUNT · ITEM_INT', enabled: () => idle() && !sim.missionActive,
        extra: () => `<select id="plan-key" aria-label="Flight plan">${Object.entries(PLANS).map(([k, p]) => `<option value="${k}">${p.label}</option>`).join('')}</select>`,
        run: () => sim.uploadPlan($('#plan-key').value),
        help: 'Mission upload handshake: MISSION_COUNT → (MISSION_REQUEST ↔ MISSION_ITEM_INT) × n → MISSION_ACK. Every waypoint coordinate ends up in the log.',
      },
    ],
  },
  {
    title: 'Flight',
    actions: [
      {
        id: 'mode', icon: 'mode', title: 'Set flight mode', msg: 'COMMAND_LONG · 176', enabled: () => idle(),
        extra: () => `<select id="mode-sel" aria-label="Flight mode">${Object.entries(MODES).map(([k, v]) => `<option value="${v}">${k}</option>`).join('')}</select>`,
        run: () => sim.setMode(+$('#mode-sel').value),
        help: 'MAV_CMD_DO_SET_MODE with the ArduCopter custom_mode. The new mode shows up in HEARTBEAT.custom_mode.',
      },
      {
        id: 'arm', icon: 'arm', title: () => (sim.armed ? 'Disarm motors' : 'Arm motors'), msg: 'COMMAND_LONG · 400',
        on: () => sim.armed, enabled: () => idle(),
        run: () => (sim.armed ? sim.disarm(false) : sim.arm()),
        help: 'MAV_CMD_COMPONENT_ARM_DISARM. Arming runs pre-arm checks (e.g. "Need Position Estimate" in GPS modes before EKF lock).',
      },
      {
        id: 'takeoff', icon: 'takeoff', title: 'Take off (10 m)', msg: 'COMMAND_LONG · 22', enabled: () => idle() && !sim.flying,
        run: () => sim.takeoff(10), help: 'Switches to GUIDED if needed, then MAV_CMD_NAV_TAKEOFF with param7 = 10 m. Fails if not armed.',
      },
      {
        id: 'mission', icon: 'mission', title: 'Start mission', msg: 'COMMAND_LONG · 300', enabled: () => idle() && sim.mission.length > 1 && !sim.missionActive,
        run: () => sim.startMissionCmd(), help: 'MAV_CMD_MISSION_START → AUTO mode. Progress: "Mission: n WP", MISSION_CURRENT, MISSION_ITEM_REACHED.',
      },
    ],
  },
  {
    title: 'Recovery & payload',
    actions: [
      {
        id: 'rtl', icon: 'rtl', title: 'Return to launch', msg: 'COMMAND_LONG · 20', enabled: () => idle() && sim.flying,
        run: () => sim.rtlCommand(), help: 'Climbs to RTL_ALT, flies home, loiters RTL_LOIT_TIME, lands and auto-disarms.',
      },
      {
        id: 'land', icon: 'land', title: 'Land here', msg: 'COMMAND_LONG · 21', enabled: () => idle() && sim.flying,
        run: () => sim.landCommand(), help: 'MAV_CMD_NAV_LAND → LAND mode. Descends at LAND_SPEED and disarms on touchdown.',
      },
      {
        id: 'camera', icon: 'camera', title: 'Take photo', msg: 'COMMAND_LONG · 203', enabled: () => idle(),
        run: () => sim.triggerCamera(), help: 'MAV_CMD_DO_DIGICAM_CONTROL. The autopilot answers with CAMERA_FEEDBACK — a geotag of where each photo was taken.',
      },
      {
        id: 'clear', icon: 'clear', title: 'Clear flight plan', msg: 'MISSION_CLEAR_ALL', enabled: () => idle() && sim.mission.length > 0,
        run: () => sim.clearMission(), help: 'Erases the mission stored on the vehicle (anti-forensics: the plan is gone from the drone but still in the GCS .tlog).',
      },
    ],
  },
  {
    title: 'Failsafes & emergency',
    actions: [
      {
        id: 'rcfs', icon: 'radio', title: () => (sim.rcFs ? 'Restore RC link' : 'Lose RC link'), msg: 'STATUSTEXT · RC_CHANNELS',
        on: () => sim.rcFs, enabled: () => online(),
        run: () => sim.setRadioFailsafe(!sim.rcFs), help: 'Simulates the pilot\'s transmitter going out of range: "Radio Failsafe" and an automatic RTL when flying.',
      },
      {
        id: 'battery', icon: 'battery', title: 'Low battery', msg: 'STATUSTEXT · SYS_STATUS', on: () => sim.battFs, enabled: () => online() && !sim.battFs,
        run: () => sim.simulateLowBattery(), help: 'Drops the pack below BATT_LOW_VOLT: CRITICAL "Battery 1 is low" and RTL (BATT_FS_LOW_ACT=2). Power-cycle to swap the pack.',
      },
      {
        id: 'estop', icon: 'stop', title: 'Emergency stop', msg: 'COMMAND_LONG · 400 · 21196', danger: true, enabled: () => idle() && sim.armed,
        run: () => sim.disarm(true), help: 'Force-disarm with the magic number 21196 — the motors stop immediately, even in flight. The drone falls.',
      },
    ],
  },
];

const val = (x) => (typeof x === 'function' ? x() : x);
const actionsEl = $('#actions');
actionsEl.innerHTML = GROUPS.map((g) => `<div><div class="group-title">${g.title}</div><div class="grid">${g.actions.map((a) => `
  <div class="act ${a.danger ? 'danger' : ''}" role="button" tabindex="0" data-id="${a.id}" data-tip="${esc(a.help)}">
    <span class="ic">${svg(I[a.icon])}</span>
    <span class="tt"></span>
    ${a.extra ? a.extra() : `<span class="mm"></span>`}
  </div>`).join('')}</div></div>`).join('');
const ACTS = Object.fromEntries(GROUPS.flatMap((g) => g.actions).map((a) => [a.id, a]));
$('#param-val').value = sim.params[$('#param-id').value].v;
$('#param-id').addEventListener('change', (e) => { $('#param-val').value = +sim.params[e.target.value].v.toFixed(3); });
$('#mode-sel').value = String(MODES.GUIDED);

function trigger(el) {
  const a = ACTS[el.dataset.id];
  if (!a || el.getAttribute('aria-disabled') === 'true') return;
  a.run();
  setTimeout(refresh, 30);
}
actionsEl.addEventListener('click', (e) => {
  if (e.target.closest('select, input')) return;
  const el = e.target.closest('.act');
  if (el) trigger(el);
});
actionsEl.addEventListener('keydown', (e) => {
  if ((e.key === 'Enter' || e.key === ' ') && e.target.classList.contains('act')) { e.preventDefault(); trigger(e.target); }
});
actionsEl.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && e.target.id === 'param-val') trigger(e.target.closest('.act'));
});

function refresh() {
  for (const el of actionsEl.querySelectorAll('.act')) {
    const a = ACTS[el.dataset.id];
    const enabled = a.enabled();
    el.setAttribute('aria-disabled', String(!enabled));
    el.classList.toggle('disabled', !enabled);
    el.classList.toggle('on', !!(a.on && a.on()));
    el.classList.toggle('running', !!sim.busy && runningMap[sim.busy] === a.id);
    const tt = el.querySelector('.tt'), title = val(a.title);
    if (tt.textContent !== title) tt.textContent = title;
    const mm = el.querySelector('.mm'), msg = val(a.msg);
    if (mm && mm.textContent !== msg) mm.textContent = msg;
  }
  updateHud();
}
const runningMap = {
  Booting: 'power', Rebooting: 'reboot', 'Setting parameter': 'param', 'Uploading plan': 'plan', 'Changing mode': 'mode',
  Arming: 'arm', Disarming: 'arm', 'Taking off': 'takeoff', 'Starting mission': 'mission', 'Return to launch': 'rtl',
  Landing: 'land', Camera: 'camera', 'Clearing plan': 'clear', 'Emergency stop': 'estop',
};

// ---------------------------------------------------------------------------- HUD
const hud = Object.fromEntries([...document.querySelectorAll('#hud dd, .hud-foot')].map((d) => [d.dataset.k, d]));
function setHud(k, text, cls = '') {
  const el = hud[k];
  if (el.textContent !== text) el.textContent = text;
  for (const c of ['good', 'warn', 'bad']) el.classList.toggle(c, c === cls);
}

function updateHud() {
  const led = sim.led;
  const ledEl = hud.led;
  ledEl.querySelector('.led').style.background = led.color || '#333';
  ledEl.querySelector('.led').style.boxShadow = led.color ? `0 0 8px ${led.color}` : 'none';
  ledEl.querySelector('span').textContent = led.label;
  setHud('power', !sim.powered ? 'Off' : !sim.linkUp ? 'Rebooting' : !sim.ready ? 'Booting' : `On · ${(sim.bootMs() / 1000).toFixed(0)} s`, sim.powered ? 'good' : '');
  if (!sim.powered) {
    for (const k of ['mode', 'armed', 'alt', 'gs', 'batt', 'gps', 'mission']) setHud(k, '—');
    setHud('pos', `home ${HOME.lat.toFixed(5)}, ${HOME.lon.toFixed(5)} · ${HOME.alt} m`);
  } else {
    setHud('mode', modeName(sim.mode));
    setHud('armed', sim.armed ? (sim.flying ? 'ARMED · flying' : 'ARMED') : 'Disarmed', sim.armed ? 'warn' : '');
    setHud('alt', `${sim.pos.u.toFixed(1)} m AGL`);
    setHud('gs', `${sim.groundSpeed.toFixed(1)} m/s`);
    setHud('batt', `${sim.batt.v.toFixed(2)} V · ${sim.battPct}%`, sim.battFs ? 'bad' : '');
    setHud('gps', sim.gps.sats ? `${(enumName('GPS_FIX_TYPE', sim.gps.fix) || '').replace('GPS_FIX_TYPE_', '')} · ${sim.gps.sats} sats` : 'searching…', sim.ekfPos ? 'good' : 'warn');
    setHud('mission', sim.mission.length ? (sim.missionActive ? `item ${sim.missionSeq}/${sim.mission.length - 1}` : `${sim.mission.length - 1} items loaded`) : 'none');
    setHud('pos', `${sim.lat.toFixed(6)}, ${sim.lon.toFixed(6)} · ${(HOME.alt + sim.pos.u).toFixed(1)} m`);
  }

  const chip = $('#chip-link');
  chip.classList.toggle('live', sim.linkUp);
  chip.classList.toggle('lost', !sim.linkUp && items.length > 0);
  chip.querySelector('span').textContent = sim.linkUp ? 'Link · 57600 baud' : items.length ? 'No link' : 'Offline';

  const busy = $('#busy');
  busy.hidden = !sim.busy;
  if (sim.busy) busy.querySelector('span:last-child').textContent = sim.busy + '…';

  $('#act-hint').textContent = hint();
}

function hint() {
  if (!sim.powered) return 'Start by powering on';
  if (!sim.ready) return 'Booting — watch the STATUSTEXT banner';
  if (!sim.ekfPos) return 'Waiting for GPS / EKF position…';
  if (sim.crashed && !sim.armed) return 'Crashed — check the log for the force-disarm';
  if (!sim.mission.length && !sim.armed) return 'Upload a plan or arm in GUIDED';
  if (!sim.armed && [MODES.AUTO, MODES.RTL, MODES.LAND].includes(sim.mode)) return `${modeName(sim.mode)} is not armable — set GUIDED or LOITER first`;
  if (!sim.armed) return 'Arm the motors (GUIDED / LOITER)';
  if (!sim.flying) return 'Take off or start the mission';
  if (sim.missionActive) return 'Mission running in AUTO';
  return 'Airborne — try RTL, land, photo or a failsafe';
}

sim.addEventListener('state', refresh);
setInterval(refresh, 250);
updatePills(counts);
refresh();
