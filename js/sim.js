// ArduCopter + ground-control-station simulator.
// Emits the MAVLink traffic a real GCS would record in a .tlog when the
// pilot performs each action (see docs/PILOT_ACTIONS.md for the research).
import { MavEncoder, enumName } from './mavlink.js';

// --- Identities --------------------------------------------------------------
export const UAV = { sys: 1, comp: 1 };      // autopilot
export const GCS = { sys: 255, comp: 190 };  // Mission Planner / QGC default ids

// --- Enum values used below (MAVLink common + ardupilotmega) -----------------
const MAV_TYPE_QUADROTOR = 2, MAV_TYPE_GCS = 6;
const MAV_AUTOPILOT_ARDUPILOTMEGA = 3, MAV_AUTOPILOT_INVALID = 8;
const ST = { BOOT: 1, STANDBY: 3, ACTIVE: 4, CRITICAL: 5, POWEROFF: 7 };
const SEV = { EMERGENCY: 0, ALERT: 1, CRITICAL: 2, ERROR: 3, WARNING: 4, NOTICE: 5, INFO: 6 };
const RES = { ACCEPTED: 0, TEMPORARILY_REJECTED: 1, DENIED: 2, UNSUPPORTED: 3, FAILED: 4 };
const CMD = {
  NAV_WAYPOINT: 16, NAV_RETURN_TO_LAUNCH: 20, NAV_LAND: 21, NAV_TAKEOFF: 22,
  DO_SET_MODE: 176, DO_DIGICAM_CONTROL: 203, PREFLIGHT_REBOOT_SHUTDOWN: 246,
  MISSION_START: 300, COMPONENT_ARM_DISARM: 400, REQUEST_MESSAGE: 512,
};
const FRAME_GLOBAL = 0, FRAME_GLOBAL_RELATIVE_ALT = 3;

export const MODES = {
  STABILIZE: 0, ALT_HOLD: 2, AUTO: 3, GUIDED: 4, LOITER: 5, RTL: 6, LAND: 9, POSHOLD: 16,
};
export const modeName = (n) => (enumName('COPTER_MODE', n) || `MODE_${n}`).replace('COPTER_MODE_', '');
const NEEDS_POSITION = new Set([MODES.AUTO, MODES.GUIDED, MODES.LOITER, MODES.RTL, MODES.POSHOLD]);
const ARMABLE = new Set([MODES.STABILIZE, MODES.ALT_HOLD, MODES.LOITER, MODES.GUIDED, MODES.POSHOLD]);
const TAKEOFF_MODES = new Set([MODES.GUIDED]);

// The original coordinates set by ClaudeCode was ArduPilot SITL default home: Canberra Model Aircraft Club (CMAC)
// Replace with 37.2416295,-115.8200443 for Area 51 as a little easter egg.
export const HOME = { lat: 37.241920, lon: -115.819197, alt: 365.0 };
const M_PER_DEG = 111319.5;

// --- Flight plans (east/north offsets in metres) ------------------------------
export const PLANS = {
  survey: {
    label: 'Survey square', alt: 12, end: 'RTL',
    wps: [[-15, 15], [15, 15], [15, -15], [-15, -15]],
  },
  orbit: {
    label: 'Inspection hexagon', alt: 15, end: 'RTL',
    wps: Array.from({ length: 6 }, (_, i) => {
      const a = (i / 6) * Math.PI * 2;
      return [Math.round(Math.sin(a) * 18 * 10) / 10, Math.round(Math.cos(a) * 18 * 10) / 10];
    }),
  },
  delivery: {
    label: 'Out & land (delivery)', alt: 10, end: 'LAND',
    wps: [[0, 12], [10, 26]],
  },
};

// --- Parameters (subset of ArduCopter 4.5 defaults; LAND_SPEED raised for demo)
const PARAMS = [
  ['SYSID_THISMAV', 1, 2], ['FRAME_CLASS', 1, 2], ['FRAME_TYPE', 1, 2],
  ['ARMING_CHECK', 1, 6], ['DISARM_DELAY', 10, 2], ['AHRS_EKF_TYPE', 3, 2], ['EK3_ENABLE', 1, 2],
  ['GPS_TYPE', 1, 2], ['COMPASS_USE', 1, 2], ['MOT_SPIN_ARM', 0.1, 9], ['MOT_SPIN_MIN', 0.15, 9],
  ['PILOT_TKOFF_ALT', 0, 9], ['WPNAV_SPEED', 500, 9], ['WPNAV_SPEED_UP', 250, 9], ['WPNAV_SPEED_DN', 150, 9],
  ['WPNAV_RADIUS', 200, 9], ['LAND_SPEED', 100, 2], ['RTL_ALT', 1500, 6], ['RTL_LOIT_TIME', 5000, 6],
  ['RTL_SPEED', 0, 2], ['BATT_MONITOR', 4, 2], ['BATT_CAPACITY', 5200, 6], ['BATT_LOW_VOLT', 14.4, 9],
  ['BATT_CRT_VOLT', 13.6, 9], ['BATT_FS_LOW_ACT', 2, 2], ['BATT_FS_CRT_ACT', 1, 2],
  ['FS_THR_ENABLE', 1, 2], ['FS_THR_VALUE', 975, 4], ['FS_GCS_ENABLE', 0, 2], ['FENCE_ENABLE', 0, 2],
  ['FLTMODE1', 0, 2], ['FLTMODE6', 6, 2], ['CAM1_TYPE', 1, 2], ['LOG_BITMASK', 176126, 6],
];
export const EDITABLE_PARAMS = ['RTL_ALT', 'WPNAV_SPEED', 'LAND_SPEED', 'BATT_LOW_VOLT', 'DISARM_DELAY', 'FENCE_ENABLE'];

class Cancelled extends Error {}

export class Simulator extends EventTarget {
  constructor() {
    super();
    this.enc = new MavEncoder();
    this.entries = [];
    this.gen = 0;
    this.params = Object.fromEntries(PARAMS.map(([k, v, t]) => [k, { v, t }]));
    this.resetVehicle();
    this.powered = false;
    this.busy = null;
    this.last = performance.now();
    setInterval(() => this.update(), 50);
  }

  p(name) { return this.params[name].v; }

  resetVehicle() {
    Object.assign(this, {
      booted: false, ready: false, linkUp: false, gcsConnected: false,
      gps: { fix: 0, sats: 0, hdop: 99.99 }, ekfPos: false, homeSet: false,
      armed: false, mode: MODES.STABILIZE, landed: true, status: ST.BOOT,
      pos: { e: 0, n: 0, u: 0 }, vel: { e: 0, n: 0, u: 0 }, yaw: 0, roll: 0, pitch: 0, yawRate: 0,
      batt: { v: 16.8, a: 0, used: 0 }, motors: [1000, 1000, 1000, 1000],
      mission: [], missionSeq: 0, missionActive: false, planKey: null,
      nav: { kind: 'idle' }, rcFs: false, battFs: false, crashed: false,
      camCount: 0, disarmTimer: 0, prearmTimer: 25, bootAt: Date.now(), tel: {}, ledFlash: null,
    });
  }

  // ---- Scheduling helpers -------------------------------------------------------
  wait(ms) {
    const g = this.gen;
    return new Promise((res, rej) => setTimeout(() => (g === this.gen ? res() : rej(new Cancelled())), ms));
  }
  async run(label, fn) {
    if (this.busy) return false;
    const token = {};
    this.busy = label; this.busyToken = token; this.changed();
    try { await fn(); } catch (e) { if (!(e instanceof Cancelled)) console.error(e); }
    finally { if (this.busyToken === token) { this.busy = null; this.changed(); } }
    return true;
  }
  changed() { this.dispatchEvent(new Event('state')); }

  // ---- Message output -------------------------------------------------------------
  send(from, name, values, meta = {}) {
    if (!this.linkUp && from === UAV) return null;
    const m = this.enc.encode(name, from.sys, from.comp, values);
    m.t = Date.now();
    m.dir = from === GCS ? 'up' : 'down';
    m.cat = meta.cat || categorise(name);
    m.note = meta.note || '';
    this.entries.push(m);
    this.dispatchEvent(new CustomEvent('msg', { detail: m }));
    return m;
  }
  uav(name, v, meta) { return this.send(UAV, name, v, meta); }
  gcs(name, v, meta) { return this.send(GCS, name, v, meta); }
  text(sev, text, note) { return this.uav('STATUSTEXT', { severity: sev, text }, { note }); }
  annotate(text, kind = 'info') {
    const m = { t: Date.now(), annotation: true, text, kind, cat: 'gcs' };
    this.dispatchEvent(new CustomEvent('msg', { detail: m }));
  }
  bootMs() { return Date.now() - this.bootAt; }

  heartbeat(meta) {
    let base = 1 | 16 | 64; // CUSTOM_MODE_ENABLED | STABILIZE_ENABLED | MANUAL_INPUT_ENABLED
    if (NEEDS_POSITION.has(this.mode) || this.mode === MODES.LAND) base |= 8; // GUIDED_ENABLED
    if (this.armed) base |= 128; // SAFETY_ARMED
    return this.uav('HEARTBEAT', {
      type: MAV_TYPE_QUADROTOR, autopilot: MAV_AUTOPILOT_ARDUPILOTMEGA, base_mode: base,
      custom_mode: this.mode, system_status: this.status, mavlink_version: 3,
    }, meta);
  }

  commandLong(cmd, params = [], note) {
    const v = { target_system: UAV.sys, target_component: UAV.comp, command: cmd, confirmation: 0 };
    params.forEach((p, i) => { v['param' + (i + 1)] = p; });
    return this.gcs('COMMAND_LONG', v, { note });
  }
  ack(cmd, result, note) {
    return this.uav('COMMAND_ACK', {
      command: cmd, result, progress: 0, result_param2: 0,
      target_system: GCS.sys, target_component: GCS.comp,
    }, { note, cat: result === RES.ACCEPTED ? 'cmd' : 'warn' });
  }

  // ---- Derived state ------------------------------------------------------------
  get flying() { return this.armed && !this.landed; }
  get lat() { return HOME.lat + this.pos.n / M_PER_DEG; }
  get lon() { return HOME.lon + this.pos.e / (M_PER_DEG * Math.cos(HOME.lat * Math.PI / 180)); }
  get battPct() { return Math.max(0, Math.round(100 - (this.batt.used / this.p('BATT_CAPACITY')) * 100)); }
  get groundSpeed() { return Math.hypot(this.vel.e, this.vel.n); }

  /** ArduPilot RGB status LED pattern (see ArduPilot "LED meanings" wiki) */
  get led() {
    if (!this.powered) return { color: null, pattern: 'off', label: 'Off' };
    if (!this.booted) return { color: '#ff5050', pattern: 'cycle', label: 'Booting / initialising' };
    if (this.battFs || this.rcFs) return { color: '#ffc21a', pattern: 'flash', label: 'Failsafe active' };
    if (!this.ready) return { color: '#ff5050', pattern: 'flash', label: 'Initialising sensors' };
    if (this.armed) return this.ekfPos
      ? { color: '#35e06f', pattern: 'solid', label: 'Armed, GPS lock' }
      : { color: '#3b82ff', pattern: 'solid', label: 'Armed, no GPS lock' };
    if (NEEDS_POSITION.has(this.mode) && !this.ekfPos) return { color: '#ffc21a', pattern: 'flash', label: 'Pre-arm check failing' };
    return this.ekfPos
      ? { color: '#35e06f', pattern: 'flash', label: 'Disarmed, GPS lock' }
      : { color: '#3b82ff', pattern: 'flash', label: 'Disarmed, no GPS lock' };
  }

  // =================================================================================
  // PILOT ACTIONS
  // =================================================================================

  powerOn() {
    if (this.powered) return;
    this.gen++;
    this.resetVehicle();
    this.powered = true;
    this.bootAt = Date.now();
    this.annotate('Battery connected — flight controller powering up');
    this.bootSequence();
  }

  powerOff() {
    if (!this.powered || this.armed) return;
    this.gen++;
    this.busy = null; this.busyToken = null;
    this.powered = false;
    this.linkUp = false;
    this.booted = this.ready = false;
    this.motors = [1000, 1000, 1000, 1000];
    this.annotate('Battery disconnected — vehicle stops transmitting (no shutdown message is sent)');
    const g = this.gen;
    setTimeout(() => {
      if (g === this.gen) this.annotate('GCS: no HEARTBEAT from 1:1 for 5 s — link lost (gap in .tlog)', 'warn');
    }, 5000);
    this.changed();
  }

  async bootSequence() {
    await this.run('Booting', async () => {
      await this.wait(900);
      this.linkUp = true;           // telemetry radio is up, autopilot starts talking
      this.status = ST.BOOT;
      this.heartbeat({ note: 'First HEARTBEAT: system_status BOOT' });
      await this.wait(250);
      this.text(SEV.INFO, 'Barometer 1 calibration complete');
      this.text(SEV.INFO, 'Barometer 2 calibration complete');
      await this.wait(300);
      this.text(SEV.INFO, 'ArduCopter V4.5.7 (2a3dc4b7)', 'Firmware banner (vehicle type + version + git hash)');
      this.text(SEV.INFO, 'ChibiOS: 6a85082c');
      this.text(SEV.INFO, 'CubeOrangePlus 0030003E 3331510A 32313938', 'Board name and unique MCU serial');
      this.text(SEV.INFO, 'RCOut: PWM:1-14');
      this.text(SEV.INFO, 'IMU0: fast sampling enabled 8.0kHz/2.0kHz');
      this.text(SEV.INFO, 'Frame: QUAD/X', 'Airframe class / type (FRAME_CLASS, FRAME_TYPE)');
      this.booted = true; this.changed();
      await this.wait(400);
      await this.gcsConnect();
      await this.wait(300);
      this.text(SEV.INFO, 'ArduPilot Ready');
      this.text(SEV.INFO, 'AHRS: DCM active');
      await this.wait(700);
      this.text(SEV.INFO, 'EKF3 IMU0 initialised');
      this.text(SEV.INFO, 'EKF3 IMU1 initialised');
      this.text(SEV.INFO, 'AHRS: EKF3 active');
      this.status = ST.STANDBY;
      this.ready = true; this.changed();
      this.heartbeat({ cat: 'state', note: 'system_status → STANDBY' });
      await this.wait(900);
      this.text(SEV.INFO, 'EKF3 IMU0 tilt alignment complete');
      this.text(SEV.INFO, 'EKF3 IMU1 tilt alignment complete');
      this.text(SEV.INFO, 'EKF3 IMU0 MAG0 initial yaw alignment complete');
      this.text(SEV.INFO, 'EKF3 IMU1 MAG0 initial yaw alignment complete');
    });
    // GPS acquisition continues in the background so the pilot can act meanwhile
    this.gpsSequence();
  }

  async gpsSequence() {
    try {
      await this.wait(1200);
      this.text(SEV.INFO, 'GPS 1: probing for u-blox at 230400 baud');
      this.text(SEV.INFO, 'GPS 1: detected u-blox', 'GPS receiver detected on serial port');
      for (let s = 3; s <= 14; s++) {
        await this.wait(380);
        this.gps.sats = s;
        this.gps.fix = s < 6 ? 1 : s < 9 ? 3 : s < 12 ? 4 : 6;
        this.gps.hdop = Math.max(0.6, 4.5 - s * 0.28);
      }
      this.text(SEV.INFO, 'EKF3 IMU0 origin set', 'EKF origin = first GPS position');
      this.uav('GPS_GLOBAL_ORIGIN', {
        latitude: Math.round(HOME.lat * 1e7), longitude: Math.round(HOME.lon * 1e7),
        altitude: Math.round(HOME.alt * 1000), time_usec: this.bootMs() * 1000,
      });
      await this.wait(500);
      this.text(SEV.INFO, `Field Elevation Set: ${Math.round(HOME.alt)}m`);
      this.text(SEV.INFO, 'EKF3 IMU1 origin set');
      this.homeSet = true;
      this.sendHome('Home position = take-off point (forensically: where the flight started)');
      await this.wait(1800);
      this.text(SEV.INFO, 'EKF3 IMU0 is using GPS', 'EKF now has a position estimate → GPS modes can arm');
      this.text(SEV.INFO, 'EKF3 IMU1 is using GPS');
      this.ekfPos = true;
      this.changed();
    } catch (e) { if (!(e instanceof Cancelled)) throw e; }
  }

  sendHome(note) {
    this.uav('HOME_POSITION', {
      latitude: Math.round(HOME.lat * 1e7), longitude: Math.round(HOME.lon * 1e7),
      altitude: Math.round(HOME.alt * 1000), x: 0, y: 0, z: -0, q: [1, 0, 0, 0],
      approach_x: 0, approach_y: 0, approach_z: 0, time_usec: this.bootMs() * 1000,
    }, { cat: 'status', note });
  }

  async gcsConnect() {
    this.gcsConnected = true;
    this.annotate('GCS detected vehicle heartbeat (sysid 1) — connecting');
    this.gcs('HEARTBEAT', { type: MAV_TYPE_GCS, autopilot: MAV_AUTOPILOT_INVALID, base_mode: 0, custom_mode: 0, system_status: ST.ACTIVE, mavlink_version: 3 },
      { cat: 'heartbeat', note: 'GCS heartbeat (MAV_TYPE_GCS)' });
    await this.wait(120);
    this.commandLong(CMD.REQUEST_MESSAGE, [148], 'Request AUTOPILOT_VERSION');
    await this.wait(40);
    this.ack(CMD.REQUEST_MESSAGE, RES.ACCEPTED);
    this.uav('AUTOPILOT_VERSION', {
      capabilities: 64495, flight_sw_version: 0x040507ff, middleware_sw_version: 0, os_sw_version: 0,
      board_version: 0x044c0000, flight_custom_version: [0x32, 0x61, 0x33, 0x64, 0x63, 0x34, 0x62, 0x37],
      os_custom_version: [0x36, 0x61, 0x38, 0x35, 0x30, 0x38, 0x32, 0x63],
      vendor_id: 0x2dae, product_id: 0x1058, uid: 3472329338437718579n, uid2: Array(18).fill(0),
    }, { cat: 'status', note: 'Firmware version, board id, capabilities, UID' });
    for (const [stream, rate] of [[1, 2], [2, 2], [3, 2], [6, 3], [10, 10], [11, 2], [12, 3]]) {
      this.gcs('REQUEST_DATA_STREAM', { target_system: 1, target_component: 1, req_stream_id: stream, req_message_rate: rate, start_stop: 1 },
        { cat: 'gcs', note: `Request stream ${enumName('MAV_DATA_STREAM', stream)} @ ${rate} Hz` });
    }
    await this.wait(150);
    this.gcs('PARAM_REQUEST_LIST', { target_system: 1, target_component: 1 }, { cat: 'param', note: 'GCS downloads all parameters' });
    const keys = Object.keys(this.params);
    for (let i = 0; i < keys.length; i++) {
      if (i % 6 === 0) await this.wait(30);
      this.paramValue(keys[i], i);
    }
    this.annotate(`Parameters downloaded (${keys.length}/${keys.length})`);
  }

  paramValue(id, index = 65535, note) {
    const pr = this.params[id];
    return this.uav('PARAM_VALUE', {
      param_id: id, param_value: pr.v, param_type: pr.t, param_count: Object.keys(this.params).length, param_index: index,
    }, { note });
  }

  reboot() {
    return this.run('Rebooting', async () => {
      this.commandLong(CMD.PREFLIGHT_REBOOT_SHUTDOWN, [1, 0, 0, 0, 0, 0, 0], 'param1=1: reboot autopilot');
      await this.wait(60);
      if (this.armed) {
        this.ack(CMD.PREFLIGHT_REBOOT_SHUTDOWN, RES.FAILED, 'Reboot refused while armed');
        return;
      }
      this.ack(CMD.PREFLIGHT_REBOOT_SHUTDOWN, RES.ACCEPTED);
      await this.wait(200);
      this.gen++;
      this.linkUp = false;
      this.booted = this.ready = false;
      this.changed();
      this.annotate('Autopilot rebooting — telemetry silent', 'warn');
      const g = this.gen;
      this.busyToken = null; this.busy = null;
      setTimeout(() => {
        if (g !== this.gen || !this.powered) return;
        this.resetVehicle(); this.bootAt = Date.now(); this.bootSequence();
      }, 2500);
    });
  }

  setMode(mode, via = 'command') {
    return this.run('Changing mode', async () => {
      this.commandLong(CMD.DO_SET_MODE, [1, mode], `MAV_CMD_DO_SET_MODE → ${modeName(mode)}`);
      await this.wait(50);
      this.applyMode(mode, CMD.DO_SET_MODE);
    });
  }

  /** returns true on success, also sends ACK + failure text */
  applyMode(mode, cmd, quiet = false) {
    let reason = null;
    if (NEEDS_POSITION.has(mode) && !this.ekfPos) reason = 'requires position';
    else if (mode === MODES.AUTO && this.mission.length < 2) reason = 'init failed';
    if (reason) {
      if (cmd) this.ack(cmd, RES.FAILED, `Mode change refused: ${reason}`);
      this.text(SEV.WARNING, `Mode change to ${modeName(mode)} failed: ${reason}`);
      return false;
    }
    if (cmd && !quiet) this.ack(cmd, RES.ACCEPTED);
    this.enterMode(mode);
    return true;
  }

  enterMode(mode) {
    this.mode = mode;
    this.missionActive = false;
    if (this.flying || (this.armed && mode === MODES.LAND)) {
      if (mode === MODES.RTL) this.startRTL();
      else if (mode === MODES.LAND) this.nav = { kind: 'land' };
      else if (mode === MODES.AUTO) this.startMission();
      else this.nav = { kind: 'hold', e: this.pos.e, n: this.pos.n, u: this.pos.u };
    }
    this.heartbeat({ cat: 'state', note: `custom_mode → ${modeName(mode)}` });
    this.changed();
  }

  arm() {
    return this.run('Arming', async () => {
      this.commandLong(CMD.COMPONENT_ARM_DISARM, [1, 0], 'param1=1: ARM');
      await this.wait(60);
      const fail = this.armCheck();
      if (fail) {
        this.ack(CMD.COMPONENT_ARM_DISARM, RES.FAILED, 'Arming refused by pre-arm checks');
        this.text(SEV.CRITICAL, `Arm: ${fail}`, 'Reason the autopilot refused to arm');
        return;
      }
      this.ack(CMD.COMPONENT_ARM_DISARM, RES.ACCEPTED);
      this.text(SEV.INFO, 'Arming motors');
      this.armed = true; this.landed = true; this.crashed = false;
      this.disarmTimer = 0; this.nav = { kind: 'idle' };
      this.changed();
      await this.wait(400);
      this.heartbeat({ cat: 'state', note: 'base_mode |= MAV_MODE_FLAG_SAFETY_ARMED (128)' });
    });
  }

  armCheck() {
    if (!this.ready) return 'AHRS not healthy';
    if (this.armed) return null;
    if (this.battFs) return 'Battery 1 low voltage failsafe';
    if (this.rcFs) return 'Radio failsafe on';
    if (!ARMABLE.has(this.mode)) return 'Mode not armable';
    if (NEEDS_POSITION.has(this.mode) && !this.ekfPos) return 'Need Position Estimate';
    return null;
  }

  disarm(force = false) {
    return this.run(force ? 'Emergency stop' : 'Disarming', async () => {
      this.commandLong(CMD.COMPONENT_ARM_DISARM, force ? [0, 21196] : [0, 0],
        force ? 'param1=0, param2=21196: FORCE disarm (motor kill)' : 'param1=0: DISARM');
      await this.wait(50);
      if (!this.armed) { this.ack(CMD.COMPONENT_ARM_DISARM, RES.ACCEPTED, 'Already disarmed'); return; }
      if (!force && !this.landed) {
        this.ack(CMD.COMPONENT_ARM_DISARM, RES.FAILED, 'Disarm refused: vehicle is flying (needs force=21196)');
        return;
      }
      this.ack(CMD.COMPONENT_ARM_DISARM, RES.ACCEPTED);
      this.doDisarm(force ? 'Emergency motor stop in flight!' : '');
      if (force && this.pos.u > 0.05) {
        this.nav = { kind: 'fall' };
        this.crashed = true;
      }
    });
  }

  doDisarm(note) {
    this.text(SEV.INFO, 'Disarming motors', note);
    this.armed = false;
    this.prearmTimer = 0;
    this.missionActive = false;
    if (this.nav.kind !== 'fall') this.nav = { kind: 'idle' };
    this.status = ST.STANDBY;
    this.changed();
    setTimeout(() => this.powered && this.heartbeat({ cat: 'state', note: 'SAFETY_ARMED flag cleared' }), 300);
  }

  takeoff(alt = 10) {
    return this.run('Taking off', async () => {
      if (!TAKEOFF_MODES.has(this.mode)) {
        this.commandLong(CMD.DO_SET_MODE, [1, MODES.GUIDED], 'GCS switches to GUIDED before take-off');
        await this.wait(50);
        if (!this.applyMode(MODES.GUIDED, CMD.DO_SET_MODE)) return;
        await this.wait(300);
      }
      this.commandLong(CMD.NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt], `MAV_CMD_NAV_TAKEOFF to ${alt} m (param7)`);
      await this.wait(60);
      if (!this.armed) { this.ack(CMD.NAV_TAKEOFF, RES.FAILED, 'Take-off refused: motors not armed'); return; }
      if (!this.landed) { this.ack(CMD.NAV_TAKEOFF, RES.FAILED, 'Take-off refused: already flying'); return; }
      this.ack(CMD.NAV_TAKEOFF, RES.ACCEPTED);
      this.landed = false;
      this.status = ST.ACTIVE;
      this.nav = { kind: 'takeoff', u: alt };
      this.heartbeat({ cat: 'state', note: 'system_status → ACTIVE (airborne)' });
      this.changed();
    });
  }

  uploadPlan(key) {
    const plan = PLANS[key];
    return this.run('Uploading plan', async () => {
      const items = [];
      const ll = (e, n) => [Math.round((HOME.lat + n / M_PER_DEG) * 1e7),
        Math.round((HOME.lon + e / (M_PER_DEG * Math.cos(HOME.lat * Math.PI / 180))) * 1e7)];
      const [hx, hy] = ll(0, 0);
      items.push({ command: CMD.NAV_WAYPOINT, frame: FRAME_GLOBAL, x: hx, y: hy, z: HOME.alt, e: 0, n: 0, u: 0, autocontinue: 1 });
      items.push({ command: CMD.NAV_TAKEOFF, frame: FRAME_GLOBAL_RELATIVE_ALT, x: 0, y: 0, z: plan.alt, e: 0, n: 0, u: plan.alt, autocontinue: 1 });
      for (const [e, n] of plan.wps) {
        const [x, y] = ll(e, n);
        items.push({ command: CMD.NAV_WAYPOINT, frame: FRAME_GLOBAL_RELATIVE_ALT, x, y, z: plan.alt, e, n, u: plan.alt, autocontinue: 1 });
      }
      if (plan.end === 'RTL') items.push({ command: CMD.NAV_RETURN_TO_LAUNCH, frame: FRAME_GLOBAL_RELATIVE_ALT, x: 0, y: 0, z: 0, autocontinue: 1 });
      else {
        const [e, n] = plan.wps[plan.wps.length - 1];
        const [x, y] = ll(e, n);
        items.push({ command: CMD.NAV_LAND, frame: FRAME_GLOBAL_RELATIVE_ALT, x, y, z: 0, e, n, u: 0, autocontinue: 1 });
      }

      this.gcs('MISSION_COUNT', { target_system: 1, target_component: 1, count: items.length, mission_type: 0 },
        { note: `Plan "${plan.label}": ${items.length} items (item 0 = home)` });
      for (let seq = 0; seq < items.length; seq++) {
        await this.wait(70);
        this.uav('MISSION_REQUEST', { target_system: GCS.sys, target_component: GCS.comp, seq, mission_type: 0 },
          { note: `Vehicle asks for item ${seq}` });
        await this.wait(30);
        const it = items[seq];
        this.gcs('MISSION_ITEM_INT', {
          target_system: 1, target_component: 1, seq, frame: it.frame, command: it.command,
          current: 0, autocontinue: it.autocontinue, param1: 0, param2: 0, param3: 0, param4: 0,
          x: it.x, y: it.y, z: it.z, mission_type: 0,
        }, { note: `${enumName('MAV_CMD', it.command)}${it.z ? ` alt ${it.z} m` : ''}` });
      }
      await this.wait(60);
      this.uav('MISSION_ACK', { target_system: GCS.sys, target_component: GCS.comp, type: 0, mission_type: 0 },
        { note: 'MAV_MISSION_ACCEPTED' });
      this.text(SEV.INFO, 'Flight plan received');
      this.mission = items;
      this.planKey = key;
      this.missionSeq = 1;
      this.changed();
    });
  }

  clearMission() {
    return this.run('Clearing plan', async () => {
      this.gcs('MISSION_CLEAR_ALL', { target_system: 1, target_component: 1, mission_type: 0 });
      await this.wait(60);
      if (this.missionActive) {
        this.uav('MISSION_ACK', { target_system: GCS.sys, target_component: GCS.comp, type: 1, mission_type: 0 }, { note: 'MAV_MISSION_ERROR: mission running', cat: 'warn' });
        return;
      }
      this.uav('MISSION_ACK', { target_system: GCS.sys, target_component: GCS.comp, type: 0, mission_type: 0 });
      this.mission = []; this.planKey = null; this.missionSeq = 0;
      this.changed();
    });
  }

  startMissionCmd() {
    return this.run('Starting mission', async () => {
      this.commandLong(CMD.MISSION_START, [0, 0], 'Start AUTO mission from first item');
      await this.wait(60);
      if (!this.armed) { this.ack(CMD.MISSION_START, RES.FAILED, 'Refused: motors not armed'); return; }
      if (this.applyMode(MODES.AUTO, CMD.MISSION_START) && this.landed) {
        this.landed = false;
        this.status = ST.ACTIVE;
        this.startMission();
      }
    });
  }

  startMission() {
    this.missionActive = true;
    this.missionSeq = 1;
    this.beginItem();
  }

  beginItem() {
    const it = this.mission[this.missionSeq];
    if (!it) { this.missionActive = false; this.text(SEV.INFO, 'Mission complete'); this.nav = { kind: 'hold', ...this.pos }; return; }
    const names = { 16: 'WP', 20: 'RTL', 21: 'Land', 22: 'Takeoff' };
    this.text(SEV.INFO, `Mission: ${this.missionSeq} ${names[it.command] || it.command}`, 'AUTO mode started a new mission item');
    this.sendMissionCurrent();
    if (it.command === CMD.NAV_TAKEOFF) this.nav = { kind: 'takeoff', u: it.u, mission: true };
    else if (it.command === CMD.NAV_WAYPOINT) this.nav = { kind: 'goto', e: it.e, n: it.n, u: it.u, mission: true };
    else if (it.command === CMD.NAV_RETURN_TO_LAUNCH) this.startRTL(true);
    else if (it.command === CMD.NAV_LAND) this.nav = { kind: 'goto', e: it.e, n: it.n, u: this.pos.u, mission: true, thenLand: true };
  }

  sendMissionCurrent(cat = 'mission') {
    this.uav('MISSION_CURRENT', {
      seq: this.missionSeq, total: Math.max(0, this.mission.length - 1),
      mission_state: this.missionActive ? 3 : (this.mission.length ? 2 : 1), mission_mode: this.mode === MODES.AUTO ? 1 : 0,
    }, { cat });
  }

  itemReached() {
    this.uav('MISSION_ITEM_REACHED', { seq: this.missionSeq }, { note: `Reached item ${this.missionSeq}` });
    this.missionSeq++;
    this.beginItem();
  }

  startRTL(fromMission = false) {
    const rtlAlt = this.p('RTL_ALT') / 100;
    this.nav = { kind: 'rtl', stage: 'climb', u: Math.max(this.pos.u, rtlAlt), mission: fromMission };
  }

  rtlCommand() {
    return this.run('Return to launch', async () => {
      this.commandLong(CMD.NAV_RETURN_TO_LAUNCH, [], 'MAV_CMD_NAV_RETURN_TO_LAUNCH');
      await this.wait(50);
      this.applyMode(MODES.RTL, CMD.NAV_RETURN_TO_LAUNCH);
    });
  }

  landCommand() {
    return this.run('Landing', async () => {
      this.commandLong(CMD.NAV_LAND, [0, 0, 0, 0, 0, 0, 0], 'MAV_CMD_NAV_LAND at current position');
      await this.wait(50);
      if (!this.armed) { this.ack(CMD.NAV_LAND, RES.FAILED, 'Nothing to land'); return; }
      this.applyMode(MODES.LAND, CMD.NAV_LAND);
    });
  }

  setParam(id, value) {
    return this.run('Setting parameter', async () => {
      const pr = this.params[id];
      this.gcs('PARAM_SET', { target_system: 1, target_component: 1, param_id: id, param_value: value, param_type: pr.t },
        { note: `Change ${id}: ${pr.v} → ${value}` });
      await this.wait(70);
      pr.v = pr.t === 9 ? Math.fround(value) : Math.round(value);
      this.paramValue(id, 65535, 'Vehicle echoes the new value = confirmation');
      this.changed();
    });
  }

  triggerCamera() {
    return this.run('Camera', async () => {
      this.commandLong(CMD.DO_DIGICAM_CONTROL, [0, 0, 0, 0, 1, 0, 0], 'param5=1: shoot one photo');
      await this.wait(60);
      this.ack(CMD.DO_DIGICAM_CONTROL, RES.ACCEPTED);
      this.camCount++;
      this.dispatchEvent(new Event('photo'));
      await this.wait(120);
      this.uav('CAMERA_FEEDBACK', {
        time_usec: Date.now() * 1000, target_system: 1, cam_idx: 0, img_idx: this.camCount,
        lat: Math.round(this.lat * 1e7), lng: Math.round(this.lon * 1e7),
        alt_msl: HOME.alt + this.pos.u, alt_rel: this.pos.u,
        roll: this.roll * 57.2958, pitch: this.pitch * 57.2958, yaw: (this.yaw * 57.2958 + 360) % 360,
        foc_len: 4.5, flags: 0, completed_captures: this.camCount,
      }, { cat: 'status', note: `Photo #${this.camCount} geotagged — position of the vehicle at capture` });
    });
  }

  setRadioFailsafe(on) {
    if (on === this.rcFs) return;
    this.rcFs = on;
    if (on) {
      this.annotate('Pilot RC transmitter switched off / out of range (throttle PWM < FS_THR_VALUE)', 'warn');
      this.text(SEV.WARNING, 'Radio Failsafe', 'RC link lost');
      if (this.flying) this.applyMode(MODES.RTL, null);
      else if (this.armed) { this.text(SEV.WARNING, 'Radio Failsafe - Disarming'); this.doDisarm(); }
    } else {
      this.text(SEV.INFO, 'Radio Failsafe Cleared');
    }
    this.changed();
  }

  simulateLowBattery() {
    if (this.battFs) return;
    const cap = this.p('BATT_CAPACITY');
    this.batt.used = Math.max(this.batt.used, cap * 0.82);
    this.batt.v = this.p('BATT_LOW_VOLT') - 0.12;
    this.battFs = true;
    this.text(SEV.CRITICAL, `Battery 1 is low ${this.batt.v.toFixed(2)}V used ${Math.round(this.batt.used)} mAh`,
      'Low-voltage failsafe (BATT_LOW_VOLT)');
    if (this.flying && this.p('BATT_FS_LOW_ACT') === 2 && this.mode !== MODES.LAND) this.applyMode(MODES.RTL, null);
    if (this.flying) { this.status = ST.CRITICAL; }
    this.changed();
  }

  // =================================================================================
  // PHYSICS + TELEMETRY (20 Hz)
  // =================================================================================
  update() {
    const now = performance.now();
    const dt = Math.min(0.2, (now - this.last) / 1000);
    this.last = now;
    if (!this.powered) return;
    this.physics(dt);
    this.housekeeping(dt);
    if (this.linkUp && this.gcsConnected) this.telemetry(dt);
  }

  physics(dt) {
    const nav = this.nav;
    const v = this.vel, p = this.pos;
    let want = { e: 0, n: 0, u: 0 };
    const hspd = this.p('WPNAV_SPEED') / 100;
    const upSpd = this.p('WPNAV_SPEED_UP') / 100, dnSpd = this.p('WPNAV_SPEED_DN') / 100;
    const towards = (te, tn, spd) => {
      const de = te - p.e, dn = tn - p.n, d = Math.hypot(de, dn);
      const s = Math.min(spd, d * 0.9);
      return d > 0.01 ? { e: (de / d) * s, n: (dn / d) * s, d } : { e: 0, n: 0, d };
    };
    const vert = (tu) => Math.max(-dnSpd, Math.min(upSpd, (tu - p.u) * 1.2));
    const landRate = () => -(p.u > 10 ? dnSpd : this.p('LAND_SPEED') / 100);

    if (nav.kind === 'fall') {
      v.u -= 9.81 * dt; v.e *= 0.99; v.n *= 0.99;
      p.e += v.e * dt; p.n += v.n * dt; p.u += v.u * dt;
      this.roll += 2.2 * dt; this.pitch += 1.3 * dt;
      if (p.u <= 0) {
        p.u = 0;
        if (v.u < -1.5) { v.u = -v.u * 0.25; } else { v.u = 0; v.e = v.n = 0; this.nav = { kind: 'idle' }; this.landed = true; this.roll = 0.06; this.pitch = -0.03; this.status = ST.STANDBY; this.changed(); }
      }
      return;
    }

    if (this.armed && !this.landed) {
      if (nav.kind === 'takeoff') {
        want.u = vert(nav.u);
        if (Math.abs(p.u - nav.u) < 0.3) {
          if (nav.mission) this.itemReached();
          else this.nav = { kind: 'hold', e: p.e, n: p.n, u: nav.u };
        }
      } else if (nav.kind === 'hold') {
        want = { ...towards(nav.e, nav.n, hspd), u: vert(nav.u) };
      } else if (nav.kind === 'goto') {
        const t = towards(nav.e, nav.n, hspd);
        want = { e: t.e, n: t.n, u: vert(nav.u) };
        if (t.d < this.p('WPNAV_RADIUS') / 100 * 0.5 && Math.abs(p.u - nav.u) < 0.5) {
          if (nav.thenLand) { this.text(SEV.INFO, 'Mission: landing'); this.nav = { kind: 'land', mission: true }; }
          else if (nav.mission) this.itemReached();
        }
      } else if (nav.kind === 'rtl') {
        if (nav.stage === 'climb') {
          want.u = vert(nav.u);
          if (Math.abs(p.u - nav.u) < 0.3) nav.stage = 'return';
        } else if (nav.stage === 'return') {
          const t = towards(0, 0, this.p('RTL_SPEED') ? this.p('RTL_SPEED') / 100 : hspd);
          want = { e: t.e, n: t.n, u: vert(nav.u) };
          if (t.d < 0.4) { nav.stage = 'loiter'; nav.until = performance.now() + this.p('RTL_LOIT_TIME'); }
        } else if (nav.stage === 'loiter') {
          want = { ...towards(0, 0, 1), u: vert(nav.u) };
          if (performance.now() > nav.until) nav.stage = 'descend';
        } else {
          want = { ...towards(0, 0, 1), u: landRate() };
        }
      } else if (nav.kind === 'land') {
        want = { e: 0, n: 0, u: landRate() };
      }
    }

    // first-order velocity response with acceleration limits
    const acc = 3.0;
    const ae = clamp((want.e - v.e) * 2.2, -acc, acc), an = clamp((want.n - v.n) * 2.2, -acc, acc);
    v.e += ae * dt; v.n += an * dt;
    v.u += clamp((want.u - v.u) * 3, -4, 4) * dt;
    if (this.landed) { v.e = v.n = v.u = 0; }
    p.e += v.e * dt; p.n += v.n * dt; p.u += v.u * dt;

    // touchdown
    if (!this.landed && p.u <= 0.02 && v.u <= 0) {
      p.u = 0; v.u = v.e = v.n = 0;
      this.landed = true;
      this.status = ST.STANDBY;
      this.annotate('Touchdown detected (land_complete)');
      if (['land', 'rtl'].includes(this.nav.kind) && this.armed) {
        const missionEnd = this.nav.mission;
        setTimeout(() => {
          if (!this.armed) return;
          if (missionEnd) { this.uav('MISSION_ITEM_REACHED', { seq: this.missionSeq }); this.missionActive = false; }
          this.doDisarm('Automatic disarm after landing');
        }, 1200);
      }
      this.nav = { kind: 'idle' };
      this.changed();
    }

    // attitude: lean into acceleration + drag, gentle hover rocking when airborne
    const yawC = Math.cos(this.yaw), yawS = Math.sin(this.yaw);
    const fwdA = (ae + v.e * 0.25) * yawS + (an + v.n * 0.25) * yawC;
    const rightA = (ae + v.e * 0.25) * yawC - (an + v.n * 0.25) * yawS;
    const tgtPitch = this.landed ? 0 : -Math.atan(fwdA / 9.81);
    const tgtRoll = this.landed ? 0 : Math.atan(rightA / 9.81);
    this.pitch += (tgtPitch - this.pitch) * Math.min(1, dt * 4);
    this.roll += (tgtRoll - this.roll) * Math.min(1, dt * 4);

    // yaw towards direction of travel in AUTO/RTL (WP_YAW_BEHAVIOR = 2)
    const gs = Math.hypot(v.e, v.n);
    if (!this.landed && gs > 1.5 && ['goto', 'rtl'].includes(this.nav.kind)) {
      const want = Math.atan2(v.e, v.n);
      let d = want - this.yaw;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      this.yawRate = clamp(d * 1.5, -1.2, 1.2);
      this.yaw = wrapPi(this.yaw + this.yawRate * dt);
    } else this.yawRate = 0;

    // motor outputs (QUAD/X: 1 FR, 2 BL, 3 FL, 4 BR)
    if (!this.armed) this.motors = [1000, 1000, 1000, 1000];
    else if (this.landed) this.motors = [1100, 1100, 1100, 1100];
    else {
      const base = 1500 + v.u * 40 + Math.hypot(ae, an) * 12;
      const pr = this.pitch * 300, rl = this.roll * 300;
      this.motors = [base + pr - rl, base - pr + rl, base + pr + rl, base - pr - rl].map((x) => Math.round(clamp(x, 1150, 1950)));
    }

    // battery model (4S LiPo 5200 mAh)
    this.batt.a = !this.armed ? 0.6 : this.landed ? 2.4 : 16 + gs * 0.8 + Math.max(0, v.u) * 3;
    this.batt.used += this.batt.a * dt / 3.6;
    if (!this.battFs) {
      const frac = this.batt.used / this.p('BATT_CAPACITY');
      this.batt.v = 16.8 - frac * 2.8 - this.batt.a * 0.012;
    }
  }

  housekeeping(dt) {
    if (!this.ready) return;
    // automatic disarm after DISARM_DELAY seconds on the ground
    if (this.armed && this.landed && this.nav.kind === 'idle' && !this.busy) {
      this.disarmTimer += dt;
      if (this.disarmTimer > this.p('DISARM_DELAY') && this.p('DISARM_DELAY') > 0) {
        this.disarmTimer = 0;
        this.doDisarm(`Auto-disarm: landed and idle for DISARM_DELAY=${this.p('DISARM_DELAY')} s`);
      }
    } else this.disarmTimer = 0;
    // periodic pre-arm failure report (every 30 s while disarmed)
    if (!this.armed) {
      this.prearmTimer += dt;
      if (this.prearmTimer > 30) {
        this.prearmTimer = 0;
        const f = this.armCheck();
        if (f) this.text(SEV.CRITICAL, `PreArm: ${f}`, 'Periodic pre-arm check report');
      }
    }
  }

  telemetry(dt) {
    const T = this.tel;
    const every = (key, period) => {
      T[key] = (T[key] ?? Math.random() * period) + dt;
      if (T[key] >= period) { T[key] -= period; return true; }
      return false;
    };
    const boot = this.bootMs();
    const latI = Math.round(this.lat * 1e7), lonI = Math.round(this.lon * 1e7);
    const hdg = Math.round(((this.yaw * 180 / Math.PI) + 360) % 360 * 100);

    if (every('hb', 1)) this.heartbeat({ cat: 'heartbeat' });
    if (every('ghb', 1)) this.gcs('HEARTBEAT', { type: MAV_TYPE_GCS, autopilot: MAV_AUTOPILOT_INVALID, base_mode: 0, custom_mode: 0, system_status: ST.ACTIVE, mavlink_version: 3 }, { cat: 'heartbeat' });
    if (every('sys', 1)) this.uav('SYS_STATUS', {
      onboard_control_sensors_present: 1399979055, onboard_control_sensors_enabled: 1380031535 | (this.armed ? 0x8000 : 0),
      onboard_control_sensors_health: this.rcFs ? 1196530735 & ~0x10000 : 1196530735, load: 210,
      voltage_battery: Math.round(this.batt.v * 1000), current_battery: Math.round(this.batt.a * 100),
      battery_remaining: this.battPct, drop_rate_comm: 0, errors_comm: 0,
    });
    if (every('gps', 1)) this.uav('GPS_RAW_INT', {
      time_usec: boot * 1000, fix_type: this.gps.fix, lat: this.gps.fix >= 3 ? latI : 0, lon: this.gps.fix >= 3 ? lonI : 0,
      alt: this.gps.fix >= 3 ? Math.round((HOME.alt + this.pos.u) * 1000) : 0, eph: Math.round(this.gps.hdop * 100), epv: Math.round(this.gps.hdop * 160),
      vel: Math.round(this.groundSpeed * 100), cog: Math.round(((Math.atan2(this.vel.e, this.vel.n) * 180 / Math.PI) + 360) % 360 * 100),
      satellites_visible: this.gps.sats, alt_ellipsoid: 0, h_acc: 300, v_acc: 400, vel_acc: 40, hdg_acc: 0, yaw: 0,
    });
    if (every('gpi', 0.5)) this.uav('GLOBAL_POSITION_INT', {
      time_boot_ms: boot, lat: this.ekfPos ? latI : 0, lon: this.ekfPos ? lonI : 0,
      alt: Math.round((HOME.alt + this.pos.u) * 1000), relative_alt: Math.round(this.pos.u * 1000),
      vx: Math.round(this.vel.n * 100), vy: Math.round(this.vel.e * 100), vz: Math.round(-this.vel.u * 100), hdg,
    });
    if (every('att', 0.5)) this.uav('ATTITUDE', {
      time_boot_ms: boot, roll: this.roll, pitch: this.pitch, yaw: this.yaw, rollspeed: 0, pitchspeed: 0, yawspeed: this.yawRate,
    });
    if (every('vfr', 1)) this.uav('VFR_HUD', {
      airspeed: this.groundSpeed, groundspeed: this.groundSpeed, heading: Math.round(hdg / 100),
      throttle: this.armed ? Math.round((this.motors[0] - 1000) / 10) : 0, alt: HOME.alt + this.pos.u, climb: this.vel.u,
    });
    if (every('servo', 1)) this.uav('SERVO_OUTPUT_RAW', {
      time_usec: (boot * 1000) >>> 0, port: 0, servo1_raw: this.motors[0], servo2_raw: this.motors[1], servo3_raw: this.motors[2], servo4_raw: this.motors[3],
    });
    if (every('rc', 1)) this.uav('RC_CHANNELS', {
      time_boot_ms: boot, chancount: this.rcFs ? 0 : 8, chan1_raw: this.rcFs ? 0 : 1500, chan2_raw: this.rcFs ? 0 : 1500,
      chan3_raw: this.rcFs ? 950 : (this.flying ? 1500 : 1000), chan4_raw: this.rcFs ? 0 : 1500, chan5_raw: this.rcFs ? 0 : 1165,
      chan6_raw: this.rcFs ? 0 : 1000, chan7_raw: this.rcFs ? 0 : 1000, chan8_raw: this.rcFs ? 0 : 1800, rssi: this.rcFs ? 0 : 255,
    });
    if (every('bat', 2)) this.uav('BATTERY_STATUS', {
      id: 0, battery_function: 0, type: 1, temperature: 32767,
      voltages: [Math.round(this.batt.v * 1000), 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
      current_battery: Math.round(this.batt.a * 100), current_consumed: Math.round(this.batt.used), energy_consumed: -1,
      battery_remaining: this.battPct, time_remaining: 0, charge_state: this.battFs ? 2 : 1,
      voltages_ext: [0, 0, 0, 0], mode: 0, fault_bitmask: 0,
    });
    if (every('ekf', 2)) this.uav('EKF_STATUS_REPORT', {
      flags: this.ekfPos ? 0x3a7 : 0xa7, velocity_variance: 0.02, pos_horiz_variance: 0.01, pos_vert_variance: 0.02,
      compass_variance: 0.03, terrain_alt_variance: 0, airspeed_variance: 0,
    });
    if (every('time', 4)) this.uav('SYSTEM_TIME', { time_unix_usec: this.gps.fix >= 3 ? Date.now() * 1000 : 0, time_boot_ms: boot });
    if (this.mission.length && every('mc', 1)) this.sendMissionCurrent('telemetry');
  }
}

function categorise(name) {
  if (name === 'COMMAND_LONG' || name === 'COMMAND_ACK' || name === 'SET_MODE') return 'cmd';
  if (name.startsWith('MISSION_')) return 'mission';
  if (name.startsWith('PARAM_')) return 'param';
  if (name === 'STATUSTEXT') return 'status';
  if (name === 'HEARTBEAT') return 'heartbeat';
  if (name === 'REQUEST_DATA_STREAM') return 'gcs';
  return 'telemetry';
}
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));
