// MAVLink v2 encoder, pymavlink-style formatter and .tlog writer.
// Message layouts come from js/mavlink-defs.js, which is generated from
// pymavlink (tools/gen_mavlink_defs.py), so frames are byte-compatible
// with mavutil / mavlogdump.py.
import { MAVLINK_DEFS } from './mavlink-defs.js';

export const DEFS = MAVLINK_DEFS;
const MSGS = DEFS.messages;
const ENUMS = DEFS.enums;

const SIZE = { B: 1, b: 1, c: 1, s: 1, '?': 1, H: 2, h: 2, I: 4, i: 4, f: 4, Q: 8, q: 8, d: 8 };

// ---- CRC-16/MCRF4XX (X.25) as used by MAVLink ---------------------------------
function crcAccumulate(byte, crc) {
  let tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}
function crc16(bytes, start, end, extra) {
  let crc = 0xffff;
  for (let i = start; i < end; i++) crc = crcAccumulate(bytes[i], crc);
  return crcAccumulate(extra, crc);
}

// ---- Payload packing ----------------------------------------------------------
const toBig = (v) => (typeof v === 'bigint' ? v : BigInt(Math.round(Number(v ?? 0))));
function writeScalar(dv, off, code, v) {
  switch (code) {
    case 'B': case 'c': case '?': dv.setUint8(off, Number(v) & 0xff); break;
    case 'b': dv.setInt8(off, Number(v)); break;
    case 'H': dv.setUint16(off, Number(v) & 0xffff, true); break;
    case 'h': dv.setInt16(off, Number(v), true); break;
    case 'I': dv.setUint32(off, Number(v) >>> 0, true); break;
    case 'i': dv.setInt32(off, Number(v) | 0, true); break;
    case 'f': dv.setFloat32(off, Number(v), true); break;
    case 'd': dv.setFloat64(off, Number(v), true); break;
    case 'Q': dv.setBigUint64(off, BigInt.asUintN(64, toBig(v)), true); break;
    case 'q': dv.setBigInt64(off, BigInt.asIntN(64, toBig(v)), true); break;
  }
}

function packPayload(def, values) {
  let size = 0;
  for (const f of def.wire) size += SIZE[f.fmt] * (f.len || 1);
  const buf = new Uint8Array(size);
  const dv = new DataView(buf.buffer);
  let off = 0;
  for (const f of def.wire) {
    const v = values[f.name];
    if (f.fmt === 's') {
      const bytes = new TextEncoder().encode(String(v ?? ''));
      buf.set(bytes.subarray(0, f.len), off);
      off += f.len;
    } else if (f.len) {
      for (let k = 0; k < f.len; k++) {
        writeScalar(dv, off, f.fmt, Array.isArray(v) ? (v[k] ?? 0) : 0);
        off += SIZE[f.fmt];
      }
    } else {
      writeScalar(dv, off, f.fmt, v ?? 0);
      off += SIZE[f.fmt];
    }
  }
  return buf;
}

// Normalise values the way they would read back after a round trip
// (float32 precision, integer wrap), so the printed text matches pymavlink.
function normalise(def, values) {
  const out = {};
  for (const f of def.wire) {
    let v = values[f.name];
    const one = (x) => {
      if (f.fmt === 'f') return Math.fround(Number(x ?? 0));
      if (f.fmt === 'd') return Number(x ?? 0);
      if (f.fmt === 'Q' || f.fmt === 'q') return toBig(x);
      return Math.trunc(Number(x ?? 0));
    };
    if (f.fmt === 's') v = String(v ?? '').slice(0, f.len);
    else if (f.len) v = Array.from({ length: f.len }, (_, k) => one(Array.isArray(v) ? v[k] : 0));
    else v = one(v);
    out[f.name] = v;
  }
  return out;
}

// ---- Frame encoder --------------------------------------------------------------
export class MavEncoder {
  constructor() { this.seq = {}; }

  encode(name, sysid, compid, values) {
    const def = MSGS[name];
    if (!def) throw new Error('Unknown MAVLink message ' + name);
    const full = packPayload(def, values);
    // MAVLink 2 trailing-zero payload truncation (min. 1 byte)
    let len = full.length;
    while (len > 1 && full[len - 1] === 0) len--;
    const key = sysid + ':' + compid;
    const seq = this.seq[key] = ((this.seq[key] ?? -1) + 1) & 0xff;
    const frame = new Uint8Array(10 + len + 2);
    frame[0] = 0xfd; frame[1] = len; frame[2] = 0; frame[3] = 0; frame[4] = seq;
    frame[5] = sysid; frame[6] = compid;
    frame[7] = def.id & 0xff; frame[8] = (def.id >> 8) & 0xff; frame[9] = (def.id >> 16) & 0xff;
    frame.set(full.subarray(0, len), 10);
    const crc = crc16(frame, 1, 10 + len, def.crc_extra);
    frame[10 + len] = crc & 0xff;
    frame[11 + len] = crc >> 8;
    return { name, id: def.id, sysid, compid, seq, frame, values: normalise(def, values) };
  }
}

// ---- pymavlink str() formatting --------------------------------------------------
export function pyFloat(x) {
  if (Number.isNaN(x)) return 'nan';
  if (!Number.isFinite(x)) return x > 0 ? 'inf' : '-inf';
  if (Object.is(x, -0)) return '-0.0';
  const exp = x === 0 ? 0 : Math.floor(Math.log10(Math.abs(x)));
  if (exp < -4 || exp >= 16) {
    let [m, e] = x.toExponential().split('e');
    const sign = e[0] === '-' ? '-' : '+';
    e = e.replace(/^[+-]/, '').padStart(2, '0');
    return `${m}e${sign}${e}`;
  }
  if (Number.isInteger(x)) return x.toFixed(1);
  return String(x);
}

function fmtValue(def, fname, v) {
  const w = def.wire.find((f) => f.name === fname);
  const one = (x) => (w.fmt === 'f' || w.fmt === 'd') ? pyFloat(x) : String(x);
  if (w.fmt === 's') return v;
  if (w.len) return '[' + v.map(one).join(', ') + ']';
  return one(v);
}

/** Exactly what `print(msg)` shows in pymavlink / mavlogdump.py */
export function pyStr(msg) {
  const def = MSGS[msg.name];
  return `${msg.name} {` + def.fields.map((f) => `${f} : ${fmtValue(def, f, msg.values[f])}`).join(', ') + '}';
}

/** mavlogdump.py timestamp prefix: "YYYY-mm-dd HH:MM:SS.cc" (local time) */
export function dumpTime(ms) {
  const d = new Date(ms);
  const p = (n, w = 2) => String(n).padStart(w, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(Math.floor(d.getMilliseconds() / 10))}`;
}

export function hex(bytes) {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join(' ');
}

// ---- Enum helpers ---------------------------------------------------------------
export function enumName(enumName, value) {
  const e = ENUMS[enumName];
  return e && e[String(value)] ? e[String(value)][0] : null;
}
export function enumDesc(enumName, value) {
  const e = ENUMS[enumName];
  return e && e[String(value)] ? e[String(value)][1] : '';
}
export function bitmaskNames(enumName, value) {
  const e = ENUMS[enumName];
  if (!e) return [];
  const v = BigInt(value);
  return Object.keys(e)
    .filter((k) => { const b = BigInt(k); return b > 0n && (v & b) === b && (b & (b - 1n)) === 0n; })
    .map((k) => e[k][0]);
}

/** Field rows for the inspector: [field, printed value, annotation] */
export function describeFields(msg) {
  const def = MSGS[msg.name];
  return def.fields.map((f) => {
    const v = msg.values[f];
    let note = '';
    const en = def.enums[f];
    if (en && typeof v !== 'object') {
      note = def.display[f] === 'bitmask' ? bitmaskNames(en, v).join(' | ') : (enumName(en, v) || '');
    } else if (en && typeof v === 'bigint') {
      note = bitmaskNames(en, v).join(' | ');
    }
    if (!note && def.units[f]) note = def.units[f];
    else if (note && def.units[f]) note += ` (${def.units[f]})`;
    return [f, fmtValue(def, f, v), note];
  });
}

// ---- Telemetry log (.tlog) ------------------------------------------------------
// Mission Planner / MAVProxy format: for every packet a big-endian uint64
// of microseconds since the Unix epoch, followed by the raw MAVLink frame.
export function buildTlog(entries) {
  let size = 0;
  for (const e of entries) size += 8 + e.frame.length;
  const out = new Uint8Array(size);
  const dv = new DataView(out.buffer);
  let off = 0;
  for (const e of entries) {
    dv.setBigUint64(off, BigInt(Math.round(e.t * 1000)), false);
    out.set(e.frame, off + 8);
    off += 8 + e.frame.length;
  }
  return out;
}
