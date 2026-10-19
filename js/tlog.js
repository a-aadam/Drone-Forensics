// .tlog parser: [uint64 BE µs timestamp][MAVLink 1 or 2 frame] repeated.
// Frames are CRC-checked with the dialect's CRC_EXTRA (same as pymavlink);
// on garbage it resynchronises byte by byte, like mavutil does.
import { DEFS } from './mavlink.js';
import { categorise } from './logcommon.js';

const SIZE = { B: 1, b: 1, c: 1, s: 1, '?': 1, H: 2, h: 2, I: 4, i: 4, f: 4, Q: 8, q: 8, d: 8 };
const TS_MIN = 946684800000;   // 2000-01-01
const TS_MAX = 4102444800000;  // 2100-01-01

function crc16(bytes, start, end, extra) {
  let crc = 0xffff;
  const acc = (b) => {
    let t = b ^ (crc & 0xff);
    t = (t ^ (t << 4)) & 0xff;
    crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xffff;
  };
  for (let i = start; i < end; i++) acc(bytes[i]);
  acc(extra);
  return crc;
}

function prepare(def) {
  if (def._size !== undefined) return def;
  let off = 0;
  def._fields = def.wire.map((f) => {
    const n = f.len || 1;
    const r = { name: f.name, fmt: f.fmt, len: f.len, off };
    off += SIZE[f.fmt] * n;
    return r;
  });
  def._size = off;
  return def;
}

function readScalar(dv, off, fmt) {
  switch (fmt) {
    case 'B': case 'c': case '?': return dv.getUint8(off);
    case 'b': return dv.getInt8(off);
    case 'H': return dv.getUint16(off, true);
    case 'h': return dv.getInt16(off, true);
    case 'I': return dv.getUint32(off, true);
    case 'i': return dv.getInt32(off, true);
    case 'f': return dv.getFloat32(off, true);
    case 'd': return dv.getFloat64(off, true);
    case 'Q': return dv.getBigUint64(off, true);
    case 'q': return dv.getBigInt64(off, true);
  }
  return 0;
}

const latin = new TextDecoder('utf-8', { fatal: false });

function decodePayload(def, src, start, len) {
  prepare(def);
  const buf = new Uint8Array(def._size);           // zero padded (MAVLink 2 truncation)
  buf.set(src.subarray(start, start + Math.min(len, def._size)));
  const dv = new DataView(buf.buffer);
  const values = {};
  for (const f of def._fields) {
    if (f.fmt === 's') {
      let end = f.off;
      while (end < f.off + f.len && buf[end] !== 0) end++;
      values[f.name] = latin.decode(buf.subarray(f.off, end));
    } else if (f.len) {
      const sz = SIZE[f.fmt], arr = new Array(f.len);
      for (let k = 0; k < f.len; k++) arr[k] = readScalar(dv, f.off + k * sz, f.fmt);
      values[f.name] = arr;
    } else {
      values[f.name] = readScalar(dv, f.off, f.fmt);
    }
  }
  return values;
}

/**
 * @param {ArrayBuffer} buffer
 * @returns {{messages: object[], stats: object}}
 */
export function parseTlog(buffer) {
  const u8 = new Uint8Array(buffer);
  const dv = new DataView(buffer);
  const n = u8.length;
  const byId = new Map(Object.values(DEFS.messages).map((d) => [d.id, d]));
  const messages = [];
  const stats = { bytes: n, badBytes: 0, crcErrors: 0, unknown: 0, v1: 0, v2: 0, signed: 0 };
  const lastState = new Map();
  const gcsSources = new Set();

  const frameAt = (f) => {
    const magic = u8[f];
    if (magic !== 0xfd && magic !== 0xfe) return 0;
    if (f + 2 > n) return 0;
    const v2 = magic === 0xfd;
    const hl = v2 ? 10 : 6;
    const sig = v2 && (u8[f + 2] & 1) ? 13 : 0;
    const total = hl + u8[f + 1] + 2 + sig;
    return f + total <= n ? total : 0;
  };
  const plausibleTs = (p) => {
    if (p + 8 > n) return false;
    const ms = Number(dv.getBigUint64(p, false)) / 1000;
    return ms > TS_MIN && ms < TS_MAX;
  };

  let p = 0;
  while (p + 8 + 8 <= n) {
    if (plausibleTs(p)) {
      const f = p + 8;
      const total = frameAt(f);
      if (total) {
        const v2 = u8[f] === 0xfd;
        const hl = v2 ? 10 : 6;
        const len = u8[f + 1];
        const msgid = v2 ? u8[f + 7] | (u8[f + 8] << 8) | (u8[f + 9] << 16) : u8[f + 5];
        const def = byId.get(msgid);
        if (def) {
          const crc = crc16(u8, f + 1, f + hl + len, def.crc_extra);
          if (crc === (u8[f + hl + len] | (u8[f + hl + len + 1] << 8))) {
            const sysid = u8[f + (v2 ? 5 : 3)], compid = u8[f + (v2 ? 6 : 4)];
            const m = {
              idx: messages.length,
              t: Number(dv.getBigUint64(p, false)) / 1000,
              name: def.name, id: msgid, sysid, compid, seq: u8[f + (v2 ? 4 : 2)],
              frame: u8.subarray(f, f + total),
              values: decodePayload(def, u8, f + hl, len),
            };
            m.cat = categorise(m.name);
            if (m.name === 'HEARTBEAT') {
              const key = sysid + ':' + compid;
              if (m.values.type === 6) gcsSources.add(key);
              else {
                const st = `${m.values.base_mode & 128}|${m.values.custom_mode}|${m.values.system_status}`;
                if (lastState.get(key) !== st) { m.cat = 'state'; lastState.set(key, st); }
              }
            } else if (m.name === 'COMMAND_ACK' && m.values.result !== 0) m.cat = 'warn';
            messages.push(m);
            v2 ? stats.v2++ : stats.v1++;
            if (total > hl + len + 2) stats.signed++;
            p = f + total;
            continue;
          }
          stats.crcErrors++;
        } else if (plausibleTs(f + total) || f + total === n) {
          stats.unknown++;   // message not in the dialect: skip the whole frame
          p = f + total;
          continue;
        }
      }
    }
    stats.badBytes++;
    p++;
  }

  for (const m of messages) {
    const key = m.sysid + ':' + m.compid;
    m.dir = gcsSources.has(key) || (!lastState.has(key) && m.sysid >= 200) ? 'up' : 'down';
  }

  // summary statistics
  const types = new Map(), sources = new Map();
  for (const m of messages) {
    types.set(m.name, (types.get(m.name) || 0) + 1);
    const key = m.sysid + ':' + m.compid;
    if (!sources.has(key)) sources.set(key, { key, sysid: m.sysid, compid: m.compid, count: 0, type: null, autopilot: null });
    const s = sources.get(key);
    s.count++;
    if (m.name === 'HEARTBEAT' && s.type === null) { s.type = m.values.type; s.autopilot = m.values.autopilot; }
  }
  stats.types = types;
  stats.sources = sources;
  stats.start = messages.length ? messages[0].t : 0;
  stats.end = messages.length ? messages[messages.length - 1].t : 0;
  return { messages, stats };
}
