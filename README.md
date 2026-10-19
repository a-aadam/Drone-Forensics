# Drone Simulator & Forensics: MAVLink flight-log forensics

An interactive, 100% client-side simulator of an **ArduCopter quadcopter** that shows, live, the **MAVLink messages a ground control station records in its telemetry log (`.tlog`)** when a drone pilot takes an action. It is meant for earning drone forensics: power the drone on, upload a flight plan, arm, take off, trigger failsafes and watch exactly what ends up in the log.

**Live demo:** https://a-aadam.github.io/Drone-Forensics/

## Backstory

Early October 2026, HackTheBox released their Drone Forensics Sherlock CTF for free for a limited time. I took a shot at time; I spent more time researching how a drone works, it's log messages and what tools to use and which ones wouldn't require major install setup. Checking the official writeup, I saw that they used Pymavlink & mavutil: so I settled for it and completed the CTF.

I also went ahead and asked ChatGPT to make use of Pymavlink to analyze the .tlog file, I asked the questions from the CTF and ChatGPT got some of the answers.

I then went to ClaudeCode to make a quick tool to help me view the logs that a drone generates to help me learn visually how things work (that is; without buying drone to crash). Hence, it gave birth to 'Drone Simulator'.

![layout: 3D drone on the left, MAVLink log top-right, pilot actions bottom-right](docs/screenshot.png)

## Features

- **3D drone model** (Three.js, built in code, no model files) with an ArduPilot-style status LED, front/rear
  orientation LEDs, spinning props, GPS mast, landing gear and a camera gimbal. Hover a part to inspect it.
- **Animations tied to the vehicle state**: LEDs boot and change pattern with GPS lock, arm state and failsafes;
  props idle when armed; the drone climbs, hovers with a slight rock, banks into turns, flies the mission and
  falls after an emergency motor stop. A follow-camera, flight trail and waypoint markers are drawn in the scene.
- **Real MAVLink v2 frames.** Every message is byte-encoded with the same wire order, `CRC_EXTRA` and checksum
  that **pymavlink** uses. The message definitions are generated from pymavlink itself.
- **Log window** with two views:
  - *Decoded*: direction (↑ GCS→vehicle / ↓ vehicle→GCS), message name, a short summary and an explanation.
    Click a row to see every field with its enum name, the exact `print(msg)` text and the raw frame in hex.
  - *mavlogdump*: the exact text `mavlogdump.py` would print.
  - Category filters (commands, status text, state changes, mission, params, GCS, heartbeats, telemetry).
- **Download the session as a `.tlog`.** It opens in `mavlogdump.py`, pymavlink `mavutil`, Mission Planner or
  MAVExplorer, so you can practise log analysis on a flight you just flew.

## Pilot actions

| Action | Key MAVLink traffic |
|---|---|
| Power on / off | boot `HEARTBEAT` + `STATUSTEXT` banner, GCS handshake, parameter download / silence (log gap) |
| Reboot autopilot | `COMMAND_LONG` 246 → `COMMAND_ACK`, then a new boot |
| Set parameter | `PARAM_SET` → `PARAM_VALUE` echo |
| Upload flight plan | `MISSION_COUNT` → (`MISSION_REQUEST` ↔ `MISSION_ITEM_INT`)×n → `MISSION_ACK` |
| Clear flight plan | `MISSION_CLEAR_ALL` → `MISSION_ACK` |
| Set flight mode | `COMMAND_LONG` 176 → `COMMAND_ACK` → `HEARTBEAT.custom_mode` |
| Arm / disarm | `COMMAND_LONG` 400 → `COMMAND_ACK`, `Arming motors` / pre-arm failures |
| Take off | `COMMAND_LONG` 22 (param7 = altitude) |
| Start mission | `COMMAND_LONG` 300 → AUTO, `Mission: n WP`, `MISSION_ITEM_REACHED` |
| Return to launch / Land | `COMMAND_LONG` 20 / 21, automatic disarm on touchdown |
| Take photo | `COMMAND_LONG` 203 → `CAMERA_FEEDBACK` (geotag) |
| Lose RC link | `Radio Failsafe` → automatic RTL |
| Low battery | `Battery 1 is low …` → automatic RTL |
| Emergency stop | `COMMAND_LONG` 400 with `param2 = 21196` (force disarm in flight) |

The simulator also models realistic refusals and automatic events: arming in a GPS mode before the EKF has a
position (`Arm: Need Position Estimate`), arming in AUTO/RTL (`Mode not armable`), disarming in flight, taking
off while disarmed, the periodic `PreArm:` reports, and the auto-disarm after `DISARM_DELAY`.

The research behind every action is in **[docs/PILOT_ACTIONS.md](docs/PILOT_ACTIONS.md)**: message sequences,
field meanings, how to read a `.tlog` with pymavlink, and what each trace tells an investigator.

## Run locally

It is a static site with no build step. Any static file server works:

```bash
node tools/serve.mjs 8080
```

Then open http://localhost:8080. (`python -m http.server` works too. Opening `index.html` directly from disk
does not, because browsers block ES modules on `file://`.)

## Analyse an exported log

```bash
pip install pymavlink
mavlogdump.py --types STATUSTEXT,COMMAND_LONG,COMMAND_ACK,HEARTBEAT drone-session-*.tlog
```

```python
from pymavlink import mavutil
mlog = mavutil.mavlink_connection("drone-session.tlog", dialect="ardupilotmega")
while (msg := mlog.recv_match()) is not None:
    print(mlog.timestamp, msg.get_srcSystem(), msg)
```

## Project layout

```
index.html                 page shell (Three.js loaded from jsDelivr via an import map)
css/style.css              styles
js/app.js                  UI: actions, log window, HUD
js/sim.js                  ArduCopter + GCS simulator (state machine, physics, telemetry)
js/scene.js                Three.js scene and drone model
js/mavlink.js              MAVLink v2 encoder, pymavlink-style formatter, .tlog writer
js/mavlink-defs.js         generated from pymavlink (do not edit)
tools/gen_mavlink_defs.py  generator for js/mavlink-defs.js
tools/serve.mjs            tiny local static server
docs/PILOT_ACTIONS.md      research notes
```

To regenerate the message definitions (for example after upgrading pymavlink or adding messages):

```bash
pip install pymavlink
python tools/gen_mavlink_defs.py > js/mavlink-defs.js
```

## Notes and limitations

- It is a teaching simulator, not SITL. Timings, physics and the telemetry set are simplified: about 12 message
  types instead of the ~30 a real ArduCopter streams.
- The firmware banner, board serial, UID and hash values are illustrative. The home position is ArduPilot's
  default SITL location (CMAC, Canberra).
- `LAND_SPEED` is set to 100 cm/s (default 50) so that landings don't take too long in the demo.

## Credits

- [MAVLink](https://mavlink.io) and [pymavlink](https://github.com/ArduPilot/pymavlink) (message definitions, `mavutil`)
- [ArduPilot](https://ardupilot.org) documentation (flight modes, pre-arm checks, failsafes, LED meanings)
- [PX4](https://docs.px4.io) MAVLink documentation, for comparison
- [Three.js](https://threejs.org)
