// Flight replay: turns the decoded messages of a .tlog into time-indexed
// tracks and exposes the same read-only interface as the live Simulator,
// so DroneScene can render a recorded flight.
import { modeString } from './logcommon.js';
import { enumName } from './mavlink.js';

const M_PER_DEG = 111319.5;

/** index of the last sample with t <= time (binary search), -1 if none */
function before(arr, time) {
  let lo = 0, hi = arr.length - 1, r = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid].t <= time) { r = mid; lo = mid + 1; } else hi = mid - 1;
  }
  return r;
}
const lerp = (a, b, k) => a + (b - a) * k;
const lerpAngle = (a, b, k) => a + Math.atan2(Math.sin(b - a), Math.cos(b - a)) * k;

/** Pick the vehicle: the non-GCS source with the most messages */
function pickVehicle(stats) {
  const cands = [...stats.sources.values()].filter((s) => s.type !== null && s.type !== 6);
  cands.sort((a, b) => b.count - a.count);
  return cands[0] || [...stats.sources.values()].sort((a, b) => b.count - a.count)[0];
}

export function buildReplay(messages, stats) {
  const veh = pickVehicle(stats);
  if (!veh) return null;
  const fromVeh = (m) => m.sysid === veh.sysid && m.compid === veh.compid;

  const raw = { pos: [], gps: [], att: [], hb: [], servo: [], batt: [], fix: [], mcur: [] };
  let home = null;
  const missionItems = new Map();
  const photos = [];
  const events = [];
  let lastHb = null;

  for (const m of messages) {
    const v = m.values;
    switch (m.name) {
      case 'GLOBAL_POSITION_INT':
        if (fromVeh(m) && (v.lat || v.lon)) raw.pos.push({ t: m.t, lat: v.lat / 1e7, lon: v.lon / 1e7, rel: v.relative_alt / 1000, alt: v.alt / 1000, vx: v.vx / 100, vy: v.vy / 100, hdg: v.hdg });
        break;
      case 'GPS_RAW_INT':
        if (fromVeh(m)) {
          raw.fix.push({ t: m.t, fix: v.fix_type, sats: v.satellites_visible });
          if (v.fix_type >= 3 && (v.lat || v.lon)) raw.gps.push({ t: m.t, lat: v.lat / 1e7, lon: v.lon / 1e7, alt: v.alt / 1000 });
        }
        break;
      case 'ATTITUDE':
        if (fromVeh(m)) raw.att.push({ t: m.t, roll: v.roll, pitch: v.pitch, yaw: v.yaw });
        break;
      case 'SERVO_OUTPUT_RAW':
        if (fromVeh(m) && v.port === 0) raw.servo.push({ t: m.t, pwm: [v.servo1_raw, v.servo2_raw, v.servo3_raw, v.servo4_raw] });
        break;
      case 'SYS_STATUS':
        if (fromVeh(m)) raw.batt.push({ t: m.t, v: v.voltage_battery / 1000, pct: v.battery_remaining });
        break;
      case 'HOME_POSITION':
        if (fromVeh(m) && !home && (v.latitude || v.longitude)) home = { lat: v.latitude / 1e7, lon: v.longitude / 1e7, alt: v.altitude / 1000 };
        break;
      case 'MISSION_CURRENT':
        if (fromVeh(m)) raw.mcur.push({ t: m.t, seq: v.seq });
        break;
      case 'MISSION_ITEM_INT': case 'MISSION_ITEM':
        if ((v.mission_type ?? 0) === 0) {
          const isInt = m.name === 'MISSION_ITEM_INT';
          missionItems.set(v.seq, { seq: v.seq, command: v.command, frame: v.frame, lat: isInt ? v.x / 1e7 : v.x, lon: isInt ? v.y / 1e7 : v.y, z: v.z });
        }
        break;
      case 'CAMERA_FEEDBACK':
        if (fromVeh(m)) { photos.push({ t: m.t }); events.push({ t: m.t, kind: 'photo', label: `Photo #${v.img_idx}` }); }
        break;
      case 'STATUSTEXT':
        if (fromVeh(m) && v.severity <= 4) events.push({ t: m.t, kind: v.severity <= 3 ? 'crit' : 'warn', label: v.text });
        break;
      case 'HEARTBEAT':
        if (fromVeh(m)) {
          const armed = !!(v.base_mode & 128);
          const mode = modeString(v);
          raw.hb.push({ t: m.t, armed, mode, status: v.system_status });
          if (!lastHb || lastHb.armed !== armed) events.push({ t: m.t, kind: armed ? 'arm' : 'disarm', label: armed ? 'Armed' : 'Disarmed' });
          if (!lastHb || lastHb.mode !== mode) events.push({ t: m.t, kind: 'mode', label: `Mode ${mode}` });
          lastHb = { armed, mode };
        }
        break;
    }
  }

  // position source: fused position if present, otherwise raw GPS
  let pos = raw.pos;
  if (pos.length < 2 && raw.gps.length >= 2) {
    const a0 = raw.gps[0].alt;
    pos = raw.gps.map((g) => ({ t: g.t, lat: g.lat, lon: g.lon, rel: g.alt - a0, alt: g.alt }));
  }
  if (pos.length < 2) return null;

  if (!home) {
    const firstArmed = raw.hb.find((h) => h.armed);
    const p = firstArmed ? pos[Math.max(0, before(pos, firstArmed.t))] : pos[0];
    home = { lat: p.lat, lon: p.lon, alt: p.alt - p.rel };
  }
  const kx = M_PER_DEG * Math.cos(home.lat * Math.PI / 180);
  const toEN = (lat, lon) => ({ e: (lon - home.lon) * kx, n: (lat - home.lat) * M_PER_DEG });

  const track = pos.map((p) => ({ t: p.t, ...toEN(p.lat, p.lon), u: p.rel, vx: p.vx ?? 0, vy: p.vy ?? 0, hdg: p.hdg }));

  let maxR = 0, maxU = 0, dist = 0;
  for (let i = 0; i < track.length; i++) {
    const p = track[i];
    maxR = Math.max(maxR, Math.abs(p.e), Math.abs(p.n));
    maxU = Math.max(maxU, p.u);
    if (i) dist += Math.hypot(p.e - track[i - 1].e, p.n - track[i - 1].n);
  }

  const mission = [...missionItems.values()].sort((a, b) => a.seq - b.seq).map((it) => {
    const hasPos = it.lat || it.lon;
    return hasPos && it.seq > 0
      ? { seq: it.seq, command: it.command, ...toEN(it.lat, it.lon), u: it.frame === 0 ? it.z - home.alt : it.z }
      : { seq: it.seq, command: it.command, u: it.z };
  });
  for (const it of mission) {
    if (it.e !== undefined) { maxR = Math.max(maxR, Math.abs(it.e), Math.abs(it.n)); maxU = Math.max(maxU, it.u); }
  }

  events.sort((a, b) => a.t - b.t);
  // the timeline spans the whole log; before the first fix the vehicle sits at its first position
  const t0 = Math.min(track[0].t, messages[0].t), t1 = Math.max(track[track.length - 1].t, messages[messages.length - 1].t);
  return new Replay({ veh, home, track, raw, mission, events, photos, t0, t1, maxR, maxU, dist });
}

export class Replay extends EventTarget {
  constructor(d) {
    super();
    Object.assign(this, d);
    this.time = d.t0;
    this.playing = false;
    this.speed = 5;
    // constant fields DroneScene reads from the live simulator
    this.powered = true; this.booted = true; this.ready = true; this.linkUp = true;
    this.bootAt = 0; this.planKey = 'replay'; this.camCount = 0;
    this.vehicleLabel = (enumName('MAV_TYPE', d.veh.type) || 'vehicle').replace('MAV_TYPE_', '');
    this.seek(d.t0);
  }

  get duration() { return this.t1 - this.t0; }

  seek(t) {
    this.time = Math.min(this.t1, Math.max(this.t0, t));
    this.update();
    this.dispatchEvent(new Event('seek'));
  }

  tick(dtMs) {
    if (!this.playing) return;
    const prev = this.time;
    this.time = Math.min(this.t1, this.time + dtMs * this.speed);
    for (const ph of this.photos) if (ph.t > prev && ph.t <= this.time) this.dispatchEvent(new Event('photo'));
    if (this.time >= this.t1) { this.playing = false; this.dispatchEvent(new Event('end')); }
    this.update();
  }

  sample(arr, t) {
    const i = before(arr, t);
    return i < 0 ? null : arr[i];
  }

  update() {
    const t = this.time;
    const tr = this.track;
    const i = before(tr, t);
    let p;
    if (i < 0) p = tr[0];
    else if (i >= tr.length - 1) p = tr[tr.length - 1];
    else {
      const a = tr[i], b = tr[i + 1], k = (t - a.t) / Math.max(1, b.t - a.t);
      p = { e: lerp(a.e, b.e, k), n: lerp(a.n, b.n, k), u: lerp(a.u, b.u, k), vx: lerp(a.vx, b.vx, k), vy: lerp(a.vy, b.vy, k), hdg: a.hdg };
    }
    this.pos = { e: p.e, n: p.n, u: Math.max(0, p.u) };
    this.vel = { n: p.vx, e: p.vy, u: 0 };
    this.groundSpeed = Math.hypot(p.vx, p.vy);

    const at = this.raw.att, j = before(at, t);
    if (j >= 0) {
      const a = at[j], b = at[Math.min(j + 1, at.length - 1)], k = b.t > a.t ? Math.min(1, (t - a.t) / (b.t - a.t)) : 0;
      this.roll = lerp(a.roll, b.roll, k); this.pitch = lerp(a.pitch, b.pitch, k); this.yaw = lerpAngle(a.yaw, b.yaw, k);
    } else {
      this.roll = 0; this.pitch = 0; this.yaw = p.hdg !== undefined && p.hdg !== 65535 ? p.hdg / 100 * Math.PI / 180 : 0;
    }

    const hb = this.sample(this.raw.hb, t);
    this.armed = hb ? hb.armed : false;
    this.modeLabel = hb ? hb.mode : '—';
    this.status = hb ? hb.status : 0;
    this.landed = !(this.armed && this.pos.u > 0.3);

    const sv = this.sample(this.raw.servo, t);
    this.motors = sv ? sv.pwm.map((x) => x || 1000) : this.armed ? [1500, 1500, 1500, 1500] : [1000, 1000, 1000, 1000];

    const fx = this.sample(this.raw.fix, t);
    this.gps = { fix: fx ? fx.fix : 0, sats: fx ? fx.sats : 0 };
    this.ekfPos = this.gps.fix >= 3;
    const bt = this.sample(this.raw.batt, t);
    this.batt = bt;

    const mc = this.sample(this.raw.mcur, t);
    this.missionSeq = mc ? mc.seq : 0;
    this.missionActive = /AUTO/.test(this.modeLabel) && this.armed;
  }

  get lat() { return this.home.lat + this.pos.n / M_PER_DEG; }
  get lon() { return this.home.lon + this.pos.e / (M_PER_DEG * Math.cos(this.home.lat * Math.PI / 180)); }

  get led() {
    const crit = this.events.some((e) => (e.kind === 'crit') && e.t <= this.time && e.t > this.time - 5000);
    if (crit) return { color: '#ffc21a', pattern: 'flash', label: 'Critical message' };
    if (this.armed) return this.ekfPos ? { color: '#35e06f', pattern: 'solid', label: 'Armed, GPS lock' } : { color: '#3b82ff', pattern: 'solid', label: 'Armed, no GPS lock' };
    return this.ekfPos ? { color: '#35e06f', pattern: 'flash', label: 'Disarmed, GPS lock' } : { color: '#3b82ff', pattern: 'flash', label: 'Disarmed, no GPS lock' };
  }
}
