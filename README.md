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

## .tlog log viewer (`tlog.html`)

Click **Upload .tlog file** in the header to open your own telemetry log (max 5 MB). The file is decoded
**in your browser**; nothing is uploaded to a server.

- **Full ArduPilot dialect decoding** (MAVLink 1 and 2, all 301 `ardupilotmega` messages) with CRC checks and
  resynchronisation past corrupt bytes. The decoded text is identical to pymavlink's `print(msg)`.
- **Log summary**: start time, duration, vehicle and GCS ids, firmware, home position, max altitude, distance
  flown, and frame integrity (CRC errors / bad bytes, which can indicate corruption or tampering).
- **Search** in three complementary ways:
  - free text over the message name, the decoded fields and the readable summary (MAV_CMD names, flight modes…):
    `arm takeoff` (AND), `"Need Position"` (phrase), `-PreArm` (exclude), `/fail(safe|ed)/` (regex);
  - **presets** for common forensic questions (arming, mode changes, failsafes, flight-plan upload, parameter
    changes, home location, firmware identity, …);
  - **message type** and **source** (`sysid:compid`) dropdowns, plus the same category pills as the simulator.
- **Virtualised list** that stays fast on 100k+ messages, a *mavlogdump* view, a message inspector
  (fields, enum names, raw hex) and **Export view** to a text file.
- **3D flight replay** (*View simulation*): the recorded path, attitude, flight mode, arming and motor outputs,
  played back at 1–60× with a timeline of mode changes, arming, warnings and photos. Clicking a log row seeks the
  replay, and with *Sync log* the list follows playback.
- Open a log from a link: `tlog.html?log=<url-of-a-.tlog>&sim=1` (the host must allow CORS).

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
js/mavlink-defs.js         generated from pymavlink, simulator subset (do not edit)
js/mavlink-defs-full.js    generated from pymavlink, whole ardupilotmega dialect (do not edit)
js/logcommon.js            log rendering shared by both pages (categories, summaries, inspector)
tlog.html, css/viewer.css  .tlog viewer page
js/viewer.js               viewer UI: load, search, virtual list, details, replay controls
js/tlog.js                 .tlog parser (MAVLink 1/2, CRC check, resync)
js/replay.js               turns a decoded log into a time-indexed flight for the 3D scene
tools/gen_mavlink_defs.py  generator for both defs files
tools/serve.mjs            tiny local static server
docs/PILOT_ACTIONS.md      research notes
```

To regenerate the message definitions (for example after upgrading pymavlink or adding messages):

```bash
pip install pymavlink
python tools/gen_mavlink_defs.py > js/mavlink-defs.js
python tools/gen_mavlink_defs.py --all > js/mavlink-defs-full.js
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

## ClaudeCode prompt

Note: Edits have been conducted from the original prompt to provide better results if you decide to try and vibe-code your own drone project.

1st Prompt:
```
I want to build a browser-based 3D drone simulator hosted on GitHub Pages.

**Requirements:**
- The app must run entirely client-side, with no backend.
- Use Three.js or a suitable alternative for the 3D drone.
- Research MAVLink, ArduPilot, PX4, `pymavlink`, and `mavutil` as needed. Since Python libraries cannot run directly in a standard browser environment, find a suitable browser-compatible solution.
- Research common pilot actions: powering on, arming/disarming, takeoff, hovering, landing, flight modes, and mission planning.
- Display the relevant MAVLink messages and status events generated by each simulated action. Distinguish actual protocol messages from application-generated events.
- Use the supplied `.tlog` file for research only. Do not copy its contents or flight sequence into the simulation.
- Put the 3D drone on the left, pilot controls on the lower-right, and a live log panel on the upper-right.
- Animate the drone and its LEDs according to its simulated state. Ensure actions follow sensible state transitions.

**Implementation:**
- Inspect the existing repository before making changes.
- Keep the code reasonably modular and document important technical limitations.
- Create a README with setup, build, and deployment instructions.

**Testing and delivery:**
- Test the build, main controls, animations, and log output.
- Check that the app works as a static site suitable for GitHub Pages.
- Prepare the project for browser testing, but do not push changes to GitHub.
- Summarize what was implemented and any outstanding issues.
- I will do the browser testing and need to approve before publishing.
```

2nd Prompt:  
//Note: The referred "dronesim.drawio.xml" is a little wireframe that I made in Draw.io to provide a visual referance.  
//Note: The prompt's section referring to the search feature: I asked a few questions for Claude to answer; it was in plan mode.
```
Extend the existing Drone Simulator with a separate `.tlog` viewer page. Preserve the current design and functionality.

**Navigation and design:**
- Add an "Upload .tlog File" button to the left of the four existing top-right icons. Do not move those icons.
- On the new page, retain the same header and icons.
- Use `dronesim.drawio.xml` as a visual reference only.

**Upload and log viewer:**
- Allow users to upload `.tlog` files up to 5 Mb.
- Process files entirely in the browser; do not upload them to a server.
- Validate files and handle invalid or oversized files.
- Decode available MAVLink messages and timestamps.
- Display logs below a toolbar with text search and category filters.
- Keep the logs vertically scrollable, including when the simulation is open.
- Keep the UI responsive when processing larger files.

**Flight playback:**
- First assess whether the telemetry contains sufficient position and attitude data to reconstruct the flight.
- If feasible, reuse the existing 3D model and animate it using recorded telemetry.
- Add a button on the left to open a sliding simulation panel occupying no more than 50% of the viewport width.
- Keep the remaining space for the log viewer.
- Provide basic playback controls and synchronize playback with log timestamps where possible.
- Do not invent a flight path if the required telemetry is missing. Explain any limitations.

**Testing and delivery:**
- Test uploading, searching, filtering, scrolling, navigation, and playback where supported.
- Ensure GitHub Pages deployment remains compatible.
- Report the results and wait for my feedback.
- I will do the browser testing and need to approve before publishing.
```

3rd Prompt (minor adjustments):  
//Note: Impeccable is a Claude UI/UX skill & \[/impeccable audit] audits the UX of the buttons.
```
Make two targeted improvements to the existing Drone Simulator.

**1. Fix log scrolling**
- When a `.tlog` is loaded and the simulation drawer is open, some log entries are cut off.
- Ensure all log entries are accessible by vertical scrolling, with the drawer both open and closed.
- Find and fix the underlying layout or overflow issue.
- Preserve search, filters, playback, and the existing layout.

**2. Refine the orange buttons**
- The orange buttons are too eye-catching.
- Use `/impeccable audit` to investigate better styling.
- Make the buttons more subtle while retaining the orange accent and existing theme.
- Refine backgrounds, shadows, borders, and hover states as appropriate.
- Preserve button hierarchy, readability, and accessibility.

Make only the necessary changes. Test scrolling in both layouts and verify that the styling changes do not break existing functionality.

Do not push changes to GitHub. Report the results and wait for my feedback.
I will do the browser testing and need to approve before publishing.
```
