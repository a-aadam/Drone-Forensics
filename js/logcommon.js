// Log rendering shared by the simulator page and the .tlog viewer.
import { pyStr, hex, enumName, describeFields } from './mavlink.js';

export const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

export const CATS = [
  { key: 'cmd', label: 'Commands', color: '#f08a4b', on: true,
    tip: 'What the pilot / ground station asked the vehicle to do (COMMAND_LONG, COMMAND_INT, SET_MODE) and whether it was accepted or refused (COMMAND_ACK).' },
  { key: 'status', label: 'Status text', color: '#ececee', on: true,
    tip: 'STATUSTEXT: readable messages from the autopilot, such as the boot banner, pre-arm failures, failsafes and mission progress. Warnings show in yellow, errors in red.' },
  { key: 'state', label: 'State changes', color: '#35e06f', on: true,
    tip: 'Vehicle HEARTBEATs where the flight mode, armed state or system status changed. Together they give the flight timeline at a glance.' },
  { key: 'mission', label: 'Mission', color: '#e9c46a', on: true,
    tip: 'MISSION_* messages: flight-plan upload/download handshake, waypoints (lat/lon/alt), current item and items reached.' },
  { key: 'param', label: 'Params', color: '#8ecae6', on: true,
    tip: 'PARAM_* messages: the parameter download when the GCS connects, plus any configuration change (PARAM_SET → PARAM_VALUE echo).' },
  { key: 'gcs', label: 'GCS', color: '#b9a5ff', on: true,
    tip: 'Ground-station housekeeping (data-stream requests, time sync, …). On the simulator page this also includes notes such as "link lost". Those notes are not MAVLink and do not go into the .tlog.' },
  { key: 'heartbeat', label: 'Heartbeats', color: '#9a9aa0', on: false,
    tip: 'The 1 Hz HEARTBEAT that every MAVLink node (vehicle and GCS) sends to show it is alive. Hidden by default because it repeats.' },
  { key: 'telemetry', label: 'Telemetry', color: '#77777d', on: false,
    tip: 'Periodic sensor streams: position, attitude, GPS, battery, RC inputs, motor outputs, EKF, IMU… Hidden by default because there is a lot of it.' },
];

export const catOf = (m) => (m.cat === 'warn' ? 'cmd' : m.cat);

export function colorOf(m) {
  if (m.cat === 'warn') return '#ff5a52';
  if (m.name === 'STATUSTEXT') return m.values.severity <= 3 ? '#ff5a52' : m.values.severity === 4 ? '#ffc21a' : '#ececee';
  return CATS.find((c) => c.key === m.cat)?.color;
}

/** Category of a decoded message (used for uploaded logs; the simulator sets its own) */
export function categorise(name) {
  if (/^(COMMAND_LONG|COMMAND_INT|COMMAND_ACK|SET_MODE|COMMAND_CANCEL)$/.test(name)) return 'cmd';
  if (name === 'MISSION_CURRENT') return 'telemetry';   // 1 Hz progress beacon
  if (name.startsWith('MISSION_')) return 'mission';
  if (name.startsWith('PARAM_')) return 'param';
  if (/^(STATUSTEXT|HOME_POSITION|GPS_GLOBAL_ORIGIN|AUTOPILOT_VERSION|CAMERA_FEEDBACK|CAMERA_IMAGE_CAPTURED)$/.test(name)) return 'status';
  if (name === 'HEARTBEAT') return 'heartbeat';
  if (/^(REQUEST_DATA_STREAM|TIMESYNC|FILE_TRANSFER_PROTOCOL|SETUP_SIGNING|PING|RADIO_STATUS|RADIO)$/.test(name)) return 'gcs';
  return 'telemetry';
}

// ---- Flight-mode names (ArduPilot per vehicle type, PX4 main/sub modes) ----------
const PX4_MAIN = { 1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO', 5: 'ACRO', 6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE' };
const PX4_AUTO = { 1: 'READY', 2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION', 5: 'RTL', 6: 'LAND', 8: 'FOLLOW', 9: 'PRECLAND' };

export function modeString(hb) {
  const { type, autopilot, custom_mode: cm, base_mode: bm } = hb;
  if (type === 6) return 'GCS';
  if (autopilot === 12) { // PX4
    if (!(bm & 1)) return 'UNKNOWN';
    const main = (cm >> 16) & 0xff, sub = (cm >>> 24) & 0xff;
    return main === 4 ? `AUTO.${PX4_AUTO[sub] || sub}` : PX4_MAIN[main] || `MODE_${main}`;
  }
  const table = type === 1 || (type >= 19 && type <= 25) ? 'PLANE_MODE'
    : type === 10 || type === 11 ? 'ROVER_MODE'
      : type === 12 ? 'SUB_MODE'
        : type === 7 ? 'PLANE_MODE' : 'COPTER_MODE';
  const n = enumName(table, cm);
  return n ? n.replace(/^(COPTER|PLANE|ROVER|SUB)_MODE_/, '') : `MODE_${cm}`;
}

// ---- One-line summaries ------------------------------------------------------------
export function summary(m) {
  const v = m.values;
  const P = (n) => v['param' + n];
  const ll = (a, b) => `${(a / 1e7).toFixed(6)}, ${(b / 1e7).toFixed(6)}`;
  const cmd = (c) => enumName('MAV_CMD', c) || `MAV_CMD ${c}`;
  let s = '';
  switch (m.name) {
    case 'COMMAND_LONG': case 'COMMAND_INT': {
      const ps = [1, 2, 3, 4, 5, 6, 7].filter((n) => P(n) && Number.isFinite(P(n))).map((n) => `p${n}=${+P(n).toFixed(4)}`).join(' ');
      const xy = m.name === 'COMMAND_INT' && (v.x || v.y) ? ` @ ${ll(v.x, v.y)} z=${+v.z.toFixed(1)}` : '';
      s = `<em>${cmd(v.command)}</em> ${ps}${xy}`;
      break;
    }
    case 'COMMAND_ACK': s = `${cmd(v.command)} → <em>${enumName('MAV_RESULT', v.result) || v.result}</em>`; break;
    case 'SET_MODE': s = `custom_mode=<em>${v.custom_mode}</em> base_mode=${v.base_mode}`; break;
    case 'HEARTBEAT':
      s = v.type === 6 ? 'GCS heartbeat'
        : `<em>${esc(modeString(v))}</em> · ${v.base_mode & 128 ? 'ARMED' : 'disarmed'} · ${(enumName('MAV_STATE', v.system_status) || '').replace('MAV_STATE_', '')}`;
      break;
    case 'STATUSTEXT':
      s = `<span style="color:${colorOf(m)}">[${(enumName('MAV_SEVERITY', v.severity) || '').replace('MAV_SEVERITY_', '')}]</span> <em>${esc(v.text)}</em>`;
      break;
    case 'MISSION_ITEM_INT': case 'MISSION_ITEM': {
      const isInt = m.name === 'MISSION_ITEM_INT';
      const pos = v.x || v.y ? (isInt ? ll(v.x, v.y) : `${v.x.toFixed(6)}, ${v.y.toFixed(6)}`) : '';
      s = `#${v.seq} <em>${cmd(v.command).replace('MAV_CMD_', '')}</em> ${pos} ${v.z ? 'alt ' + +v.z.toFixed(1) + ' m' : ''}`;
      break;
    }
    case 'MISSION_COUNT': s = `count=<em>${v.count}</em>`; break;
    case 'MISSION_REQUEST': case 'MISSION_REQUEST_INT': case 'MISSION_ITEM_REACHED': case 'MISSION_SET_CURRENT': s = `seq=<em>${v.seq}</em>`; break;
    case 'MISSION_CURRENT': s = `seq=<em>${v.seq}</em>${v.total ? '/' + v.total : ''}`; break;
    case 'MISSION_ACK': s = `<em>${enumName('MAV_MISSION_RESULT', v.type) || v.type}</em>`; break;
    case 'PARAM_VALUE': case 'PARAM_SET': s = `<em>${esc(v.param_id)}</em> = ${+v.param_value.toFixed(4)}`; break;
    case 'REQUEST_DATA_STREAM': s = `${enumName('MAV_DATA_STREAM', v.req_stream_id) || v.req_stream_id} @ ${v.req_message_rate} Hz`; break;
    case 'GLOBAL_POSITION_INT': s = `${ll(v.lat, v.lon)} rel_alt=${(v.relative_alt / 1000).toFixed(1)} m hdg=${(v.hdg / 100).toFixed(0)}°`; break;
    case 'SYS_STATUS': s = `${(v.voltage_battery / 1000).toFixed(2)} V ${(v.current_battery / 100).toFixed(1)} A ${v.battery_remaining}%`; break;
    case 'GPS_RAW_INT': s = `${(enumName('GPS_FIX_TYPE', v.fix_type) || '').replace('GPS_FIX_TYPE_', '')} sats=${v.satellites_visible}`; break;
    case 'ATTITUDE': s = `roll=${(v.roll * 57.3).toFixed(1)}° pitch=${(v.pitch * 57.3).toFixed(1)}° yaw=${(v.yaw * 57.3).toFixed(1)}°`; break;
    case 'VFR_HUD': s = `alt=${v.alt.toFixed(1)} m gs=${v.groundspeed.toFixed(1)} m/s climb=${v.climb.toFixed(1)} thr=${v.throttle}%`; break;
    case 'CAMERA_FEEDBACK': s = `img <em>#${v.img_idx}</em> @ ${ll(v.lat, v.lng)} alt_rel=${v.alt_rel.toFixed(1)} m`; break;
    case 'HOME_POSITION': case 'GPS_GLOBAL_ORIGIN': s = `${ll(v.latitude, v.longitude)} alt=${(v.altitude / 1000).toFixed(1)} m`; break;
    case 'SERVO_OUTPUT_RAW': s = `m1=${v.servo1_raw} m2=${v.servo2_raw} m3=${v.servo3_raw} m4=${v.servo4_raw}`; break;
    default:
      s = esc(pyStr(m).replace(/^\w+ \{/, '').replace(/\}$/, '').slice(0, 160));
  }
  if (m.note) s += ` <span class="nt">— ${esc(m.note)}</span>`;
  return s;
}

/** "GCS → vehicle" style direction from the source system */
export function direction(m) {
  if (m.dir) return m.dir;
  return m.sysid >= 200 || m.gcs ? 'up' : 'down';
}

export function inspectorHtml(m) {
  const f = m.frame;
  const v2 = f[0] === 0xfd;
  const hl = v2 ? 10 : 6, len = f[1];
  const header = hex(f.subarray(0, hl)), payload = hex(f.subarray(hl, hl + len)), crc = hex(f.subarray(hl + len, hl + len + 2));
  const sig = f.length > hl + len + 2 ? ' <i>' + hex(f.subarray(hl + len + 2)) + '</i>' : '';
  const rows = describeFields(m).map(([k, v, n]) => `<tr><td>${esc(k)}</td><td>${esc(v)}</td><td>${esc(n)}</td></tr>`).join('');
  const dir = direction(m) === 'up' ? 'GCS → vehicle' : 'Vehicle → GCS';
  return `<div class="inspect">
    <div class="meta">${dir} · src ${m.sysid}:${m.compid} · seq ${m.seq} · msgid ${m.id} · MAVLink ${v2 ? '2' : '1'} · ${f.length} bytes on the wire${m.note ? ' · ' + esc(m.note) : ''}</div>
    <div class="py">${esc(pyStr(m))}</div>
    <table>${rows}</table>
    <div class="hex"><b>${header}</b> ${payload} <b>${crc}</b>${sig}</div>
  </div>`;
}

// ---- Hover tooltips for any [data-tip] element ----------------------------------
let tipEl;
export function installTips(root = document) {
  if (!tipEl) {
    tipEl = document.createElement('div');
    tipEl.className = 'tip';
    tipEl.setAttribute('role', 'tooltip');
    tipEl.hidden = true;
    document.body.append(tipEl);
  }
  const show = (el) => {
    tipEl.textContent = el.dataset.tip;
    tipEl.hidden = false;
    const r = el.getBoundingClientRect();
    const w = tipEl.offsetWidth, h = tipEl.offsetHeight;
    let x = Math.min(Math.max(8, r.left), innerWidth - w - 8);
    let y = r.bottom + 8;
    if (y + h > innerHeight - 8) y = r.top - h - 8;
    tipEl.style.left = x + 'px';
    tipEl.style.top = y + 'px';
  };
  const hide = () => { tipEl.hidden = true; };
  root.addEventListener('mouseover', (e) => { const el = e.target.closest('[data-tip]'); if (el) show(el); });
  root.addEventListener('mouseout', (e) => { const el = e.target.closest('[data-tip]'); if (el && !el.contains(e.relatedTarget)) hide(); });
  root.addEventListener('focusin', (e) => { const el = e.target.closest('[data-tip]'); if (el) show(el); });
  root.addEventListener('focusout', hide);
}

// ---- Filter pills ------------------------------------------------------------------
/** Builds the category pills once; call update(counts) to refresh numbers in place */
export function filterPills(el, filterOn, onChange) {
  el.innerHTML = CATS.map((c) => `<button type="button" class="fchip ${filterOn[c.key] ? 'on' : ''}" data-k="${c.key}" style="--c:${c.color}" aria-pressed="${filterOn[c.key]}" data-tip="${esc(c.tip)}"><i></i>${c.label} <small>0</small></button>`).join('');
  el.addEventListener('click', (e) => {
    const b = e.target.closest('.fchip');
    if (!b) return;
    const k = b.dataset.k;
    filterOn[k] = !filterOn[k];
    b.classList.toggle('on', filterOn[k]);
    b.setAttribute('aria-pressed', String(filterOn[k]));
    onChange();
  });
  return (counts) => {
    for (const b of el.children) {
      const n = String(counts[b.dataset.k] ?? 0);
      const small = b.querySelector('small');
      if (small.textContent !== n) small.textContent = n;
    }
  };
}
