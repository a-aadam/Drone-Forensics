# Pilot actions and the MAVLink messages they leave behind

These are research notes for the **Drone Forensics** simulator. For each thing a drone pilot can do, they list
the MAVLink traffic a ground control station (GCS) such as Mission Planner, QGroundControl or MAVProxy writes
into its telemetry log (`.tlog`), plus what an investigator can learn from it.

The focus is **ArduCopter** (ArduPilot multirotor firmware) over **MAVLink 2**, the most common open-source
stack. PX4 uses the same MAVLink messages, but its `STATUSTEXT` wording and `custom_mode` encoding are different.

---

## 1. How a `.tlog` is built

A `.tlog` is the raw MAVLink byte stream the GCS saw. Each packet has an 8-byte timestamp in front of it:

```
┌──────────────────────────────┬──────────────────────────────────────────────┐
│ uint64 big-endian            │ MAVLink v2 frame                             │
│ µs since Unix epoch (GCS)    │ FD len incompat compat seq sys comp msgid×3  │
│                              │ payload… crc_lo crc_hi                       │
└──────────────────────────────┴──────────────────────────────────────────────┘
```

* **Both directions are logged.** Packets from the vehicle (`sysid 1, compid 1`) and from the GCS
  (`sysid 255`, Mission Planner `compid 190`, or a script/companion computer such as `250:0`) are
  interleaved. The source `sysid:compid` tells you **who issued a command**.
* **The timestamp is the GCS clock**, not the drone's. Vehicle uptime is in fields like `time_boot_ms`.
  `SYSTEM_TIME.time_unix_usec` gives the vehicle's GPS-derived UTC time once it has a fix.
* **MAVLink 2 truncates trailing zero bytes** of the payload. Decoders pad them back, so a short frame is normal.
* Each frame's CRC is seeded with a per-message `CRC_EXTRA` byte. A decoder needs the right dialect
  (`ardupilotmega` for ArduPilot) to validate and decode it.

Reading a log with pymavlink:

```python
from pymavlink import mavutil
mlog = mavutil.mavlink_connection("flight.tlog", dialect="ardupilotmega")
while (msg := mlog.recv_match()) is not None:
    if msg.get_type() == "BAD_DATA":
        continue
    print(mlog.timestamp, msg.get_srcSystem(), msg.get_srcComponent(), msg)
```

or from the command line: `mavlogdump.py --types STATUSTEXT,COMMAND_LONG,COMMAND_ACK flight.tlog`.

### Message families an investigator cares about

| Family | Messages | Why it matters |
|---|---|---|
| Identity | `HEARTBEAT`, `AUTOPILOT_VERSION`, boot `STATUSTEXT` | Vehicle type, autopilot, firmware version + git hash, board name, **MCU serial / UID** |
| State | `HEARTBEAT.base_mode / custom_mode / system_status` | Armed flag (bit 128), flight mode, on ground (`STANDBY`) vs airborne (`ACTIVE`) |
| Commands | `COMMAND_LONG` / `COMMAND_INT` → `COMMAND_ACK` | Exactly what the pilot asked for, and whether the vehicle accepted it |
| Mission | `MISSION_COUNT`, `MISSION_REQUEST(_INT)`, `MISSION_ITEM_INT`, `MISSION_ACK`, `MISSION_CURRENT`, `MISSION_ITEM_REACHED` | The planned route (every waypoint lat/lon/alt) and how far the vehicle got |
| Configuration | `PARAM_REQUEST_LIST`, `PARAM_VALUE`, `PARAM_SET` | Full parameter dump on connect; any change made during the session |
| Text | `STATUSTEXT` | Human-readable autopilot messages: pre-arm failures, failsafes, mission progress |
| Position | `GPS_RAW_INT`, `GLOBAL_POSITION_INT`, `HOME_POSITION`, `GPS_GLOBAL_ORIGIN` | Where it took off, where it flew, altitude above home |
| Health | `SYS_STATUS`, `BATTERY_STATUS`, `EKF_STATUS_REPORT`, `RC_CHANNELS` | Battery, sensor health, RC link state |
| Payload | `CAMERA_FEEDBACK` | Geotag of every photo |

---

## 2. Pilot actions

Notation: `↑` = GCS → vehicle, `↓` = vehicle → GCS. Command numbers are `MAV_CMD` values; results are
`MAV_RESULT` (0 = ACCEPTED, 4 = FAILED, …).

### 2.1 Power on (connect the flight battery)

There is no "power on" message. The first sign of life is the vehicle's first `HEARTBEAT`, followed by the boot banner:

```
↓ HEARTBEAT  type=2 (QUADROTOR) autopilot=3 (ARDUPILOTMEGA) system_status=1 (BOOT)
↓ STATUSTEXT Barometer 1 calibration complete
↓ STATUSTEXT ArduCopter V4.5.7 (2a3dc4b7)           ← firmware + git hash
↓ STATUSTEXT CubeOrangePlus 0030003E 3331510A …     ← board + unique MCU id
↓ STATUSTEXT Frame: QUAD/X
↓ STATUSTEXT ArduPilot Ready
↓ STATUSTEXT EKF3 IMU0 initialised / AHRS: EKF3 active / … tilt alignment complete
↓ HEARTBEAT  system_status=3 (STANDBY)
… GPS acquisition …
↓ STATUSTEXT GPS 1: detected u-blox
↓ GPS_GLOBAL_ORIGIN, HOME_POSITION                  ← take-off location
↓ STATUSTEXT Field Elevation Set: 584m
↓ STATUSTEXT EKF3 IMU0 is using GPS                 ← position estimate available
```

**Forensics:** firmware version, hardware serial, airframe and **home/launch location** all come from the first
seconds of the log. On the drone itself, the RGB status LED shows red/blue/green while it initialises, then
flashing blue (no GPS) and flashing green (GPS lock, ready to arm).

### 2.2 GCS connects (automatic)

```
↑ HEARTBEAT            type=6 (GCS) autopilot=8 (INVALID)        ← the GCS, 1 Hz
↑ COMMAND_LONG         MAV_CMD_REQUEST_MESSAGE (512) p1=148
↓ COMMAND_ACK          512 ACCEPTED
↓ AUTOPILOT_VERSION    flight_sw_version, board_version, vendor/product id, uid
↑ REQUEST_DATA_STREAM  (×7)  or  MAV_CMD_SET_MESSAGE_INTERVAL (511)
↑ PARAM_REQUEST_LIST
↓ PARAM_VALUE          × param_count  (≈1 300 on a real ArduCopter)
```

**Forensics:** the GCS `sysid/compid` identifies the ground station software. The parameter dump is a
snapshot of the whole configuration (failsafe actions, geofence, RTL altitude, …).

### 2.3 Set a parameter

```
↑ PARAM_SET    param_id=RTL_ALT param_value=3000 param_type=6
↓ PARAM_VALUE  param_id=RTL_ALT param_value=3000 … param_index=65535   ← echo = confirmation
```

**Forensics:** look for changes that weaken safety, such as `FENCE_ENABLE=0`, `FS_THR_ENABLE=0`,
`ARMING_CHECK=0` or `BATT_FS_*` set to 0, made right before a flight.

### 2.4 Upload a flight plan (mission)

```
↑ MISSION_COUNT     count=N mission_type=0
↓ MISSION_REQUEST   seq=0          ┐
↑ MISSION_ITEM_INT  seq=0 (home)   │  repeated N times
↓ MISSION_REQUEST   seq=1          │  x/y = lat/lon ×1e7, z = alt (m)
↑ MISSION_ITEM_INT  seq=1 MAV_CMD_NAV_TAKEOFF z=12 ┘
…
↓ MISSION_ACK       type=0 (MAV_MISSION_ACCEPTED)
↓ STATUSTEXT        Flight plan received
```

Common mission commands: `16 NAV_WAYPOINT`, `22 NAV_TAKEOFF`, `21 NAV_LAND`, `20 NAV_RETURN_TO_LAUNCH`,
`17 NAV_LOITER_UNLIM`, `203 DO_DIGICAM_CONTROL`. Frame `3` = `GLOBAL_RELATIVE_ALT` (altitude above home).

**Forensics:** this is the **intended route**, with every coordinate. Compare it with `GLOBAL_POSITION_INT`
to see whether the flight followed it. ArduPilot replies with `MISSION_REQUEST` even when the GCS sends
`MISSION_ITEM_INT`. Your sample `.tlog` shows this too.

### 2.5 Clear the flight plan

```
↑ MISSION_CLEAR_ALL
↓ MISSION_ACK   type=0
```

The mission is gone from the vehicle, but the upload is still in the GCS `.tlog`. A clear sent while the
mission is running is rejected (`MAV_MISSION_ERROR`).

### 2.6 Change flight mode

```
↑ COMMAND_LONG  MAV_CMD_DO_SET_MODE (176) p1=1 (CUSTOM_MODE_ENABLED) p2=<custom_mode>
   (older GCS / pymavlink set_mode():  ↑ SET_MODE base_mode=1 custom_mode=<n>)
↓ COMMAND_ACK   176 ACCEPTED
↓ HEARTBEAT     custom_mode=<n>
```

ArduCopter `custom_mode`: 0 STABILIZE, 2 ALT_HOLD, 3 AUTO, 4 GUIDED, 5 LOITER, 6 RTL, 9 LAND, 16 POSHOLD.
If the mode needs a position estimate and the EKF doesn't have one yet:
`COMMAND_ACK FAILED` + `STATUSTEXT "Mode change to GUIDED failed: requires position"`.

**Forensics:** reconstruct the mode timeline from `HEARTBEAT.custom_mode` (pymavlink: `mlog.flightmode`).

### 2.7 Arm the motors

```
↑ COMMAND_LONG  MAV_CMD_COMPONENT_ARM_DISARM (400) p1=1
↓ COMMAND_ACK   400 ACCEPTED
↓ STATUSTEXT    Arming motors
↓ HEARTBEAT     base_mode |= 128 (MAV_MODE_FLAG_SAFETY_ARMED)
```

When a pre-arm check fails, the result is `FAILED` plus a reason, e.g. `Arm: Need Position Estimate`,
`Arm: Mode not armable` (AUTO/RTL/LAND) or `Arm: Battery 1 low voltage failsafe`. While disarmed,
the autopilot also repeats `PreArm: <reason>` every 30 s.
The props spin at idle (`MOT_SPIN_ARM`), visible in `SERVO_OUTPUT_RAW` (≈1100 µs).

### 2.8 Disarm

```
↑ COMMAND_LONG  400 p1=0
↓ COMMAND_ACK   ACCEPTED            (only when landed — FAILED in flight)
↓ STATUSTEXT    Disarming motors
↓ HEARTBEAT     armed bit cleared
```

ArduCopter also **auto-disarms** after `DISARM_DELAY` (10 s) on the ground without taking off, and after
landing in LAND/RTL. These disarms produce `Disarming motors` with **no preceding command**, which is how you
tell them apart from a pilot disarm.

### 2.9 Take off (GUIDED)

```
↑ COMMAND_LONG  176 → GUIDED        (GCS switches mode first if needed)
↑ COMMAND_LONG  MAV_CMD_NAV_TAKEOFF (22) p7=<alt m>
↓ COMMAND_ACK   22 ACCEPTED         (FAILED if disarmed / wrong mode)
↓ HEARTBEAT     system_status=4 (ACTIVE)
↓ GLOBAL_POSITION_INT relative_alt rising, VFR_HUD climb > 0
```

### 2.10 Start the mission (AUTO)

```
↑ COMMAND_LONG       MAV_CMD_MISSION_START (300)    (or a mode change to AUTO)
↓ COMMAND_ACK        ACCEPTED
↓ HEARTBEAT          custom_mode=3 (AUTO)
↓ STATUSTEXT         Mission: 1 Takeoff
↓ MISSION_ITEM_REACHED seq=1
↓ STATUSTEXT         Mission: 2 WP
↓ MISSION_CURRENT    seq=2 … (1 Hz)
…
↓ STATUSTEXT         Mission: 6 RTL
```

**Forensics:** `MISSION_ITEM_REACHED` proves the vehicle actually reached each waypoint, and when.

### 2.11 Return to launch (RTL)

```
↑ COMMAND_LONG  MAV_CMD_NAV_RETURN_TO_LAUNCH (20)   (or mode change → 6)
↓ COMMAND_ACK   ACCEPTED
↓ HEARTBEAT     custom_mode=6 (RTL)
```

Behaviour: climb to `RTL_ALT` → fly home → loiter `RTL_LOIT_TIME` → descend → `Disarming motors`.

### 2.12 Land

```
↑ COMMAND_LONG  MAV_CMD_NAV_LAND (21)
↓ COMMAND_ACK   ACCEPTED
↓ HEARTBEAT     custom_mode=9 (LAND)
… touchdown → STATUSTEXT Disarming motors, system_status=STANDBY
```

### 2.13 Take a photo

```
↑ COMMAND_LONG     MAV_CMD_DO_DIGICAM_CONTROL (203) p5=1
↓ COMMAND_ACK      ACCEPTED
↓ CAMERA_FEEDBACK  img_idx, lat, lng, alt_msl, alt_rel, roll, pitch, yaw
```

**Forensics:** `CAMERA_FEEDBACK` records **where and when each image was taken**. Match it with the EXIF data
of images recovered from the SD card.

### 2.14 Reboot the autopilot

```
↑ COMMAND_LONG  MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) p1=1
↓ COMMAND_ACK   ACCEPTED (FAILED while armed)
… silence … → a new boot banner, time_boot_ms restarts from 0
```

### 2.15 Power off

**No message.** The heartbeats stop and the GCS reports a lost link. In the `.tlog` this is a **gap**
followed, if the battery is reconnected, by a new boot banner and `time_boot_ms` starting again near zero.

### 2.16 Failsafes (no pilot command)

| Event | Messages | Default reaction |
|---|---|---|
| RC transmitter lost | `STATUSTEXT [WARNING] Radio Failsafe`, `RC_CHANNELS` throttle < `FS_THR_VALUE`, rssi 0 | RTL if flying (`FS_THR_ENABLE=1`), disarm if landed |
| RC restored | `STATUSTEXT Radio Failsafe Cleared` | — |
| Low battery | `STATUSTEXT [CRITICAL] Battery 1 is low 14.28V used 4264 mAh`, `BATTERY_STATUS.charge_state=2` (LOW) | `BATT_FS_LOW_ACT` (2 = RTL) |
| GCS link lost | nothing reaches the GCS, so nothing is logged | `FS_GCS_ENABLE` action |

A failsafe shows up as a mode change in `HEARTBEAT` that **no `COMMAND_LONG` from the GCS asked for**. That is
how you tell an automatic RTL from one the pilot commanded.

### 2.17 Emergency motor stop (force disarm)

```
↑ COMMAND_LONG  400 p1=0 p2=21196          ← "magic" force value
↓ COMMAND_ACK   ACCEPTED
↓ STATUSTEXT    Disarming motors
↓ HEARTBEAT     armed bit cleared while relative_alt > 0
… GLOBAL_POSITION_INT shows free-fall (vz large, positive = down)
```

**Forensics:** a disarm while airborne together with `param2=21196` points to a deliberate kill, not a malfunction.

---

## 3. Telemetry streamed all the time

| Message | Rate (sim) | Content |
|---|---|---|
| `HEARTBEAT` (both sides) | 1 Hz | mode, armed, system status |
| `SYS_STATUS` | 1 Hz | battery V/A/%, sensor present/enabled/health bitmasks |
| `GPS_RAW_INT` | 1 Hz | fix type, satellites, raw lat/lon |
| `GLOBAL_POSITION_INT` | 2 Hz | fused position, relative altitude, velocity, heading |
| `ATTITUDE` | 2 Hz | roll/pitch/yaw (rad) |
| `VFR_HUD` | 1 Hz | speed, altitude, climb, throttle |
| `SERVO_OUTPUT_RAW` | 1 Hz | motor PWM (1000 = stopped) |
| `RC_CHANNELS` | 1 Hz | pilot stick inputs, RSSI |
| `BATTERY_STATUS`, `EKF_STATUS_REPORT`, `SYSTEM_TIME`, `MISSION_CURRENT` | 0.25–1 Hz | |

Real vehicles stream more (and faster): `RAW_IMU`, `SCALED_PRESSURE`, `VIBRATION`, `TERRAIN_*`, `ESC_TELEMETRY_*`,
`POWER_STATUS`, `MEMINFO`, `NAV_CONTROLLER_OUTPUT`, … Your sample log has about 30 such types at roughly 4 Hz.

---

## 4. Observations from the sample `.tlog` (research only)

The attached ArduCopter SITL log was parsed with pymavlink (`mavutil.mavlink_connection(..., dialect="ardupilotmega")`)
to check the sequences above. It confirms:

* the boot banner order (`Barometer … calibration complete` → firmware line → board id → `Frame:` → `ArduPilot Ready` → EKF init);
* a separate script/GCS (`250:0`) uploading a 9-item mission with `MISSION_COUNT` / `MISSION_REQUEST` / `MISSION_ITEM_INT`,
  then `MISSION_ACK` + `Flight plan received`;
* `SET_MODE` to GUIDED, two rejected arm attempts (`COMMAND_ACK result=4` + `Arm: Need Position Estimate`), then a
  successful arm after `EKF3 IMU0 is using GPS`;
* `SET_MODE` AUTO → `Mission: 1 Takeoff`, followed by an automatic `Disarming motors` with no GCS command;
* a second GCS (`255:230`) polling `MAV_CMD_GET_HOME_POSITION` (410) every 2 s, which the vehicle rejected (`result=4`).

The simulator's messages use the same structure but different values (different home, firmware banner, plans), and it
does not reproduce the sample.

---

## 5. References

* MAVLink common message set — <https://mavlink.io/en/messages/common.html>
* ArduPilot dialect — <https://mavlink.io/en/messages/ardupilotmega.html>
* MAVLink 2 serialisation & CRC_EXTRA — <https://mavlink.io/en/guide/serialization.html>
* Mission protocol — <https://mavlink.io/en/services/mission.html>
* Command protocol — <https://mavlink.io/en/services/command.html>
* ArduCopter flight modes — <https://ardupilot.org/copter/docs/flight-modes.html>
* ArduCopter pre-arm checks — <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>
* ArduCopter failsafes — <https://ardupilot.org/copter/docs/failsafe-landing-page.html>
* ArduPilot LED meanings — <https://ardupilot.org/copter/docs/common-leds-pixhawk.html>
* pymavlink / mavutil — <https://github.com/ArduPilot/pymavlink>
* PX4 MAVLink usage (for comparison) — <https://docs.px4.io/main/en/mavlink/>
