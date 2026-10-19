// Three.js scene: a procedurally built ArduCopter-style quad (QUAD/X),
// driven by the simulator state.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const S = 0.32;            // world units per metre of flight
const GROUND_OFF = 0.52;   // skid bottom to body centre
const ORANGE = 0xf08a4b;

// QUAD/X motor layout (ArduPilot numbering): position (x right, z back) + spin
const MOTORS = [
  { n: 1, x: 0.82, z: -0.82, dir: -1, label: 'front-right', spin: 'CCW' },
  { n: 2, x: -0.82, z: 0.82, dir: -1, label: 'rear-left', spin: 'CCW' },
  { n: 3, x: -0.82, z: -0.82, dir: 1, label: 'front-left', spin: 'CW' },
  { n: 4, x: 0.82, z: 0.82, dir: 1, label: 'rear-right', spin: 'CW' },
];

function glowTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  const r = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  r.addColorStop(0, 'rgba(255,255,255,1)');
  r.addColorStop(0.25, 'rgba(255,255,255,0.55)');
  r.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = r;
  g.fillRect(0, 0, 64, 64);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function groundTexture() {
  const N = 1024, c = document.createElement('canvas');
  c.width = c.height = N;
  const g = c.getContext('2d');
  g.fillStyle = '#1d1d1f';
  g.fillRect(0, 0, N, N);
  g.strokeStyle = 'rgba(255,255,255,0.06)';
  g.lineWidth = 1.5;
  const step = N / 24;
  for (let i = 0; i <= 24; i++) {
    g.beginPath(); g.moveTo(i * step, 0); g.lineTo(i * step, N); g.stroke();
    g.beginPath(); g.moveTo(0, i * step); g.lineTo(N, i * step); g.stroke();
  }
  // radial fade to transparent
  g.globalCompositeOperation = 'destination-in';
  const r = g.createRadialGradient(N / 2, N / 2, N * 0.08, N / 2, N / 2, N / 2);
  r.addColorStop(0, 'rgba(0,0,0,1)');
  r.addColorStop(0.7, 'rgba(0,0,0,0.6)');
  r.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = r;
  g.fillRect(0, 0, N, N);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

function labelSprite(text, color = '#f08a4b') {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(20,20,22,0.85)';
  g.beginPath(); g.arc(64, 64, 44, 0, Math.PI * 2); g.fill();
  g.lineWidth = 6; g.strokeStyle = color; g.stroke();
  g.fillStyle = '#fff';
  g.font = '600 52px Inter, system-ui, sans-serif';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 64, 67);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false, transparent: true }));
  s.scale.set(0.42, 0.42, 1);
  s.renderOrder = 10;
  return s;
}

export class DroneScene {
  constructor(container, sim, tooltip) {
    this.sim = sim;
    this.container = container;
    this.tooltip = tooltip;
    this.follow = true;
    this.clock = new THREE.Clock();
    this.parts = [];
    this.flash = 0;
    this.trailPts = [];

    const r = this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.ACESFilmicToneMapping;
    r.toneMappingExposure = 1.05;
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(r.domElement);

    const scene = this.scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(r);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

    this.camera = new THREE.PerspectiveCamera(36, 1, 0.05, 200);
    this.camera.position.set(4.6, 3.4, -5.6);
    const c = this.controls = new OrbitControls(this.camera, r.domElement);
    c.target.set(0, 0.5, 0);
    c.enableDamping = true;
    c.dampingFactor = 0.08;
    c.minDistance = 2.2;
    c.maxDistance = 30;
    c.maxPolarAngle = Math.PI * 0.495;
    c.autoRotateSpeed = 0.8;

    scene.add(new THREE.HemisphereLight(0xdfe6ff, 0x1a1410, 0.55));
    const key = this.key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(4, 9, -3);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    Object.assign(key.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 0.5, far: 30 });
    key.shadow.radius = 6;
    key.shadow.bias = -0.0004;
    scene.add(key, key.target);
    const rim = new THREE.DirectionalLight(0xffb27a, 0.7);
    rim.position.set(-5, 3, 6);
    scene.add(rim);

    this.buildGround();
    this.buildDrone();
    this.missionGroup = new THREE.Group();
    scene.add(this.missionGroup);
    this.buildTrail();

    this.raycaster = new THREE.Raycaster();
    this.mouse = new THREE.Vector2(9, 9);
    r.domElement.addEventListener('pointermove', (e) => this.onPointer(e));
    r.domElement.addEventListener('pointerleave', () => { this.mouse.set(9, 9); tooltip.hidden = true; });

    sim.addEventListener('photo', () => { this.flash = 1; });
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    r.setAnimationLoop(() => this.frame());
  }

  // ---------------------------------------------------------------- building
  buildGround() {
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
      new THREE.MeshStandardMaterial({ map: groundTexture(), transparent: true, roughness: 1, metalness: 0 }));
    ground.rotation.x = -Math.PI / 2;
    this.scene.add(ground);
    const shadow = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.ShadowMaterial({ opacity: 0.45 }));
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.y = 0.002;
    shadow.receiveShadow = true;
    this.scene.add(shadow);

    // home pad
    const pad = new THREE.Group();
    const ring = new THREE.Mesh(new THREE.RingGeometry(1.25, 1.32, 64),
      new THREE.MeshBasicMaterial({ color: ORANGE, transparent: true, opacity: 0.55 }));
    ring.rotation.x = -Math.PI / 2;
    pad.add(ring);
    const c = document.createElement('canvas');
    c.width = c.height = 256;
    const g = c.getContext('2d');
    g.fillStyle = 'rgba(240,138,75,0.35)';
    g.font = '700 150px Inter, system-ui, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText('H', 128, 138);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    const h = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 1.4), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    h.rotation.x = -Math.PI / 2;
    h.position.y = 0.003;
    pad.add(h);
    pad.position.y = 0.004;
    this.scene.add(pad);
  }

  part(mesh, name, info) {
    mesh.userData.part = { name, info };
    mesh.castShadow = true;
    this.parts.push(mesh);
    return mesh;
  }

  buildDrone() {
    const d = this.drone = new THREE.Group();
    d.rotation.order = 'YXZ';
    this.scene.add(d);
    const mat = {
      body: new THREE.MeshPhysicalMaterial({ color: 0xf2f1ee, roughness: 0.42, clearcoat: 0.35, clearcoatRoughness: 0.4 }),
      top: new THREE.MeshPhysicalMaterial({ color: 0x8f9aa8, roughness: 0.3, metalness: 0.25, clearcoat: 0.6 }),
      orange: new THREE.MeshPhysicalMaterial({ color: ORANGE, roughness: 0.45, clearcoat: 0.3 }),
      dark: new THREE.MeshStandardMaterial({ color: 0x2c2e33, roughness: 0.55, metalness: 0.35 }),
      darker: new THREE.MeshStandardMaterial({ color: 0x1d1e21, roughness: 0.6, metalness: 0.2 }),
      grey: new THREE.MeshStandardMaterial({ color: 0x55585f, roughness: 0.5, metalness: 0.4 }),
      glass: new THREE.MeshPhysicalMaterial({ color: 0x0b0c10, roughness: 0.05, metalness: 0.1, clearcoat: 1 }),
      metal: new THREE.MeshStandardMaterial({ color: 0xc9ccd2, roughness: 0.25, metalness: 0.9 }),
    };

    const bodyInfo = 'Flight controller (CubeOrange+ running ArduCopter), ESCs, power module and telemetry radio. Source of every vehicle→GCS MAVLink message (sysid 1, compid 1).';
    d.add(this.part(new THREE.Mesh(new RoundedBoxGeometry(0.9, 0.3, 1.1, 5, 0.1), mat.body), 'Fuselage / flight controller', bodyInfo));
    const top = this.part(new THREE.Mesh(new RoundedBoxGeometry(0.8, 0.06, 0.98, 4, 0.03), mat.top), 'Canopy', 'Removable top cover over the autopilot and battery bay.');
    top.position.y = 0.155;
    d.add(top);
    for (const x of [-0.18, 0.18]) {
      const slit = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.01, 0.55), mat.darker);
      slit.position.set(x, 0.188, -0.05);
      d.add(slit);
    }
    // side vents
    for (const sx of [-1, 1]) for (let i = 0; i < 4; i++) {
      const v = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.12, 0.018), mat.grey);
      v.position.set(sx * 0.452, 0, 0.2 + i * 0.05);
      d.add(v);
    }

    // front obstacle sensor (two lenses)
    const sensor = this.part(new THREE.Mesh(new RoundedBoxGeometry(0.42, 0.12, 0.08, 3, 0.035), mat.orange),
      'Forward rangefinder', 'Twin-lens proximity sensor (MAV_SENSOR_ORIENTATION forward). Reports via DISTANCE_SENSOR on real vehicles.');
    sensor.position.set(0, 0.02, -0.56);
    d.add(sensor);
    for (const x of [-0.1, 0.1]) {
      const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.03, 24), mat.glass);
      lens.rotation.x = Math.PI / 2;
      lens.position.set(x, 0.02, -0.605);
      d.add(lens);
    }

    // arms, motors, props, LEDs
    this.props = [];
    this.armLeds = [];
    for (const m of MOTORS) {
      const len = Math.hypot(m.x, m.z);
      const arm = this.part(new THREE.Mesh(new RoundedBoxGeometry(0.1, 0.07, len, 3, 0.03), mat.orange),
        `Arm ${m.n}`, `Carbon/PLA arm carrying motor ${m.n} (${m.label}).`);
      arm.position.set(m.x / 2, 0.02, m.z / 2);
      arm.rotation.y = Math.atan2(m.x, m.z);
      d.add(arm);

      const mount = this.part(new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.13, 0.07, 32), mat.dark),
        `Motor ${m.n} (${m.label}, ${m.spin})`, `Brushless motor on ESC output SERVO${m.n}. Its PWM appears in SERVO_OUTPUT_RAW.servo${m.n}_raw.`);
      mount.position.set(m.x, 0.06, m.z);
      mount.userData.motor = m.n - 1;
      d.add(mount);
      const bell = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.12, 32), mat.darker);
      bell.position.set(m.x, 0.155, m.z);
      bell.castShadow = true;
      d.add(bell);
      const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.05, 0.035, 24), mat.orange);
      cap.position.set(m.x, 0.23, m.z);
      d.add(cap);

      const prop = new THREE.Group();
      prop.position.set(m.x, 0.255, m.z);
      const bladeMat = new THREE.MeshStandardMaterial({ color: 0x232428, roughness: 0.5, transparent: true });
      for (const s of [-1, 1]) {
        const b = this.part(new THREE.Mesh(new RoundedBoxGeometry(0.4, 0.014, 0.07, 2, 0.006), bladeMat),
          `Propeller ${m.n}`, `${m.spin} propeller. Spins at idle (MOT_SPIN_ARM) once armed.`);
        b.position.x = s * 0.2;
        b.rotation.x = s * 0.18;
        prop.add(b);
      }
      const disc = new THREE.Mesh(new THREE.CircleGeometry(0.42, 48),
        new THREE.MeshBasicMaterial({ color: 0xd8dce3, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide }));
      disc.rotation.x = -Math.PI / 2;
      prop.add(disc);
      d.add(prop);
      this.props.push({ group: prop, dir: m.dir, bladeMat, disc, angle: Math.random() * 6 });

      // orientation LED under each arm tip: front green, rear red
      const color = m.z < 0 ? 0x2bff6a : 0xff3030;
      const ledMat = new THREE.MeshStandardMaterial({ color: 0x222222, emissive: color, emissiveIntensity: 0 });
      const led = this.part(new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 12), ledMat),
        'Orientation LED', m.z < 0 ? 'Front (green) arm light — on whenever the vehicle is powered.' : 'Rear (red) arm light — on whenever the vehicle is powered.');
      led.position.set(m.x, 0.005, m.z);
      d.add(led);
      const glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow ??= glowTexture(), color, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
      glow.scale.set(0.42, 0.42, 1);
      glow.position.copy(led.position);
      d.add(glow);
      this.armLeds.push({ mat: ledMat, glow });
    }

    // status RGB LED (ArduPilot notify LED) on canopy, rear
    this.statusMat = new THREE.MeshStandardMaterial({ color: 0x111111, emissive: 0xffffff, emissiveIntensity: 0 });
    const sled = this.part(new THREE.Mesh(new RoundedBoxGeometry(0.16, 0.03, 0.07, 2, 0.012), this.statusMat),
      'Status LED (RGB)', 'ArduPilot notify LED. Flashing blue: disarmed, no GPS • flashing green: disarmed, GPS lock • solid green: armed • flashing yellow: failsafe / pre-arm fail.');
    sled.position.set(0, 0.19, 0.4);
    d.add(sled);
    this.statusGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
    this.statusGlow.scale.set(0.6, 0.6, 1);
    this.statusGlow.position.set(0, 0.22, 0.4);
    d.add(this.statusGlow);

    // GPS / compass mast
    const mast = this.part(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.34, 12), mat.metal),
      'GPS / compass mast', 'u-blox GNSS + compass, raised away from power wiring. Feeds GPS_RAW_INT and the EKF position estimate.');
    mast.position.set(0, 0.35, 0.22);
    d.add(mast);
    const puck = this.part(new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.05, 32), mat.orange),
      'GPS receiver', 'Satellite count / fix type are reported in GPS_RAW_INT (fix_type 3 = 3D, 6 = RTK fixed).');
    puck.position.set(0, 0.53, 0.22);
    d.add(puck);
    const puckTop = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.012, 32), mat.body);
    puckTop.position.set(0, 0.56, 0.22);
    d.add(puckTop);
    this.gpsLed = new THREE.MeshStandardMaterial({ color: 0x111111, emissive: 0x3b82ff, emissiveIntensity: 0 });
    const gl = new THREE.Mesh(new THREE.SphereGeometry(0.018, 10, 8), this.gpsLed);
    gl.position.set(0, 0.568, 0.18);
    d.add(gl);

    // landing gear
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const leg = this.part(new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.4, 10), mat.dark),
          'Landing gear', 'Skid landing gear. Touch-down is detected by the autopilot (land_complete), then motors auto-disarm.');
        leg.position.set(sx * 0.34, -0.31, sz * 0.3);
        leg.rotation.z = sx * 0.18;
        d.add(leg);
      }
      const skid = this.part(new THREE.Mesh(new THREE.CapsuleGeometry(0.03, 0.95, 6, 12), mat.body),
        'Landing gear', 'Skid landing gear. Touch-down is detected by the autopilot (land_complete), then motors auto-disarm.');
      skid.rotation.x = Math.PI / 2;
      skid.position.set(sx * 0.375, -0.49, 0);
      d.add(skid);
    }

    // camera gimbal
    const bracket = new THREE.Mesh(new RoundedBoxGeometry(0.2, 0.08, 0.14, 2, 0.03), mat.dark);
    bracket.position.set(0, -0.18, -0.36);
    d.add(bracket);
    const cam = this.part(new THREE.Mesh(new THREE.SphereGeometry(0.1, 32, 24), mat.darker),
      'Camera gimbal', 'Payload camera. MAV_CMD_DO_DIGICAM_CONTROL triggers a shot; the autopilot geotags it with CAMERA_FEEDBACK.');
    cam.position.set(0, -0.28, -0.38);
    d.add(cam);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.055, 0.012, 10, 32), mat.orange);
    ring.position.set(0, -0.28, -0.475);
    d.add(ring);
    const lens = new THREE.Mesh(new THREE.CircleGeometry(0.05, 32), mat.glass);
    lens.position.set(0, -0.28, -0.479);
    lens.rotation.y = Math.PI;
    d.add(lens);
    this.flashLight = new THREE.PointLight(0xffffff, 0, 6);
    this.flashLight.position.set(0, -0.3, -0.6);
    d.add(this.flashLight);
    this.flashSprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
    this.flashSprite.scale.set(0.9, 0.9, 1);
    this.flashSprite.position.set(0, -0.28, -0.5);
    d.add(this.flashSprite);

    d.traverse((o) => { if (o.isMesh && !o.material.transparent) o.castShadow = true; });
    d.position.y = GROUND_OFF;
  }

  buildTrail() {
    const max = 4000;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
    geo.setDrawRange(0, 0);
    this.trail = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 }));
    this.trail.frustumCulled = false;
    this.trailMax = max;
    this.trailN = 0;
    this.scene.add(this.trail);
  }

  clearTrail() { this.trailN = 0; this.trail.geometry.setDrawRange(0, 0); }

  buildMission() {
    const g = this.missionGroup;
    while (g.children.length) g.remove(g.children[0]);
    this.wpMarkers = [];
    const items = this.sim.mission;
    if (!items.length) return;
    const pts = [new THREE.Vector3(0, GROUND_OFF, 0)];
    const lineMat = new THREE.LineDashedMaterial({ color: ORANGE, dashSize: 0.18, gapSize: 0.12, transparent: true, opacity: 0.8 });
    const poleMat = new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.06, gapSize: 0.08, transparent: true, opacity: 0.25 });
    items.forEach((it, seq) => {
      if (seq === 0) return;
      let p;
      if (it.command === 20) p = new THREE.Vector3(0, items[1].u * S + GROUND_OFF, 0);
      else if (it.command === 22) p = new THREE.Vector3(0, it.u * S + GROUND_OFF, 0);
      else p = new THREE.Vector3(it.e * S, Math.max(it.u, 0) * S + GROUND_OFF, -it.n * S);
      pts.push(p);
      if (it.command === 16 || it.command === 21) {
        const pole = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(p.x, 0, p.z), p]), poleMat);
        pole.computeLineDistances();
        g.add(pole);
        const dot = new THREE.Mesh(new THREE.RingGeometry(0.12, 0.17, 32), new THREE.MeshBasicMaterial({ color: ORANGE, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
        dot.rotation.x = -Math.PI / 2;
        dot.position.set(p.x, 0.01, p.z);
        g.add(dot);
        const label = labelSprite(String(seq));
        label.position.copy(p).add(new THREE.Vector3(0, 0.32, 0));
        g.add(label);
        this.wpMarkers.push({ seq, label, dot });
      }
    });
    if (items[items.length - 1].command === 20) pts.push(new THREE.Vector3(0, GROUND_OFF, 0));
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), lineMat);
    line.computeLineDistances();
    g.add(line);
  }

  // ---------------------------------------------------------------- interaction
  onPointer(e) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.set(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
    this.mousePx = { x: e.clientX - rect.left, y: e.clientY - rect.top };
  }

  updateTooltip() {
    if (this.mouse.x > 1) return;
    this.raycaster.setFromCamera(this.mouse, this.camera);
    const hit = this.raycaster.intersectObjects(this.parts, false)[0];
    const tt = this.tooltip;
    if (!hit) { tt.hidden = true; return; }
    const { name, info } = hit.object.userData.part;
    let live = '';
    const sim = this.sim;
    if (hit.object.userData.motor !== undefined) live = `servo${hit.object.userData.motor + 1}_raw = ${sim.motors[hit.object.userData.motor]} µs`;
    if (name.startsWith('Status LED')) live = sim.led.label;
    if (name.startsWith('GPS')) live = sim.powered ? `fix_type ${sim.gps.fix} · ${sim.gps.sats} sats` : 'unpowered';
    tt.innerHTML = `<b>${name}</b><span>${info}</span>${live ? `<code>${live}</code>` : ''}`;
    tt.hidden = false;
    const w = this.container.clientWidth;
    tt.style.left = Math.min(this.mousePx.x + 16, w - 290) + 'px';
    tt.style.top = (this.mousePx.y + 16) + 'px';
  }

  resetView() {
    this.controls.target.copy(this.drone.position).setY(Math.max(0.5, this.drone.position.y));
    this.camera.position.copy(this.controls.target).add(new THREE.Vector3(4.6, 2.9, -5.6));
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = w + 'px';
    this.renderer.domElement.style.height = h + 'px';
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
  }

  // ---------------------------------------------------------------- per frame
  frame() {
    const dt = Math.min(0.05, this.clock.getDelta());
    const t = this.clock.elapsedTime;
    const sim = this.sim, d = this.drone;

    if (sim.planKey !== this.shownPlan || sim.mission.length !== this.shownLen) {
      this.shownPlan = sim.planKey;
      this.shownLen = sim.mission.length;
      this.buildMission();
    }
    if (this.wpMarkers) for (const m of this.wpMarkers) {
      const active = sim.missionActive && m.seq === sim.missionSeq;
      const done = sim.missionActive && m.seq < sim.missionSeq;
      m.label.material.opacity = done ? 0.35 : 1;
      m.dot.material.opacity = active ? 0.6 + 0.4 * Math.sin(t * 6) : done ? 0.2 : 0.6;
      m.label.scale.setScalar(active ? 0.5 : 0.42);
    }

    // pose
    const target = new THREE.Vector3(sim.pos.e * S, sim.pos.u * S + GROUND_OFF, -sim.pos.n * S);
    d.position.lerp(target, 1 - Math.exp(-dt * 10));
    const air = THREE.MathUtils.clamp(sim.pos.u / 0.6, 0, 1) * (sim.armed ? 1 : 0);
    const rockR = air * (0.05 * Math.sin(t * 1.7) + 0.02 * Math.sin(t * 4.1));
    const rockP = air * (0.03 * Math.sin(t * 1.1 + 1));
    d.position.y += air * 0.05 * Math.sin(t * 2.3);
    const want = new THREE.Euler(sim.pitch + rockP, -sim.yaw, -sim.roll + rockR, 'YXZ');
    const k = 1 - Math.exp(-dt * 8);
    let dy = want.y - d.rotation.y;
    dy = Math.atan2(Math.sin(dy), Math.cos(dy));
    d.rotation.set(d.rotation.x + (want.x - d.rotation.x) * k, d.rotation.y + dy * k, d.rotation.z + (want.z - d.rotation.z) * k);

    // props: speed from motor PWM
    this.props.forEach((p, i) => {
      const pwm = sim.motors[i] ?? 1000;
      const targetRate = Math.max(0, pwm - 1000) / 1000 * 140;
      p.rate = (p.rate ?? 0) + (targetRate - (p.rate ?? 0)) * Math.min(1, dt * (targetRate > (p.rate ?? 0) ? 2.5 : 1.2));
      p.angle += p.rate * p.dir * dt;
      p.group.rotation.y = p.angle;
      const blur = THREE.MathUtils.clamp((p.rate - 18) / 40, 0, 1);
      p.bladeMat.opacity = 1 - blur * 0.7;
      p.disc.material.opacity = blur * 0.13;
    });

    // LEDs
    const led = sim.led;
    const since = (Date.now() - sim.bootAt) / 1000;
    this.armLeds.forEach((l, i) => {
      let on = sim.powered ? 1 : 0;
      if (sim.powered && !sim.booted) on = Math.floor(since * 6) % 4 === i ? 1 : 0.1;  // boot chase
      l.on = (l.on ?? 0) + (on - (l.on ?? 0)) * Math.min(1, dt * 14);
      l.mat.emissiveIntensity = l.on * 3;
      l.glow.material.opacity = l.on * 0.85;
    });
    let sCol = new THREE.Color(0x000000), sOn = 0;
    if (led.pattern !== 'off') {
      if (led.pattern === 'cycle') {
        sCol.set(['#ff5050', '#35e06f', '#3b82ff'][Math.floor(t * 5) % 3]);
        sOn = 1;
      } else {
        sCol.set(led.color);
        sOn = led.pattern === 'solid' ? 1 : (Math.sin(t * Math.PI * 4) > 0 ? 1 : 0.08);
      }
    }
    this.statusMat.emissive.copy(sCol);
    this.statusMat.emissiveIntensity = sOn * 3.2;
    this.statusGlow.material.color.copy(sCol);
    this.statusGlow.material.opacity = sOn * 0.9;
    this.gpsLed.emissiveIntensity = sim.powered ? (sim.gps.fix >= 3 ? (Math.sin(t * 6) > 0 ? 3 : 0.2) : 1.5) : 0;
    this.gpsLed.emissive.set(sim.gps.fix >= 3 ? 0x35e06f : 0x3b82ff);

    // camera flash
    this.flash = Math.max(0, this.flash - dt * 3.2);
    this.flashLight.intensity = this.flash * 25;
    this.flashSprite.material.opacity = this.flash;

    // trail
    if (sim.armed && !sim.landed) {
      const last = this.trailLast;
      if (!last || last.distanceTo(d.position) > 0.05) {
        if (this.trailN >= this.trailMax) this.clearTrail();
        this.trail.geometry.attributes.position.setXYZ(this.trailN++, d.position.x, d.position.y - 0.45, d.position.z);
        this.trail.geometry.attributes.position.needsUpdate = true;
        this.trail.geometry.setDrawRange(0, this.trailN);
        this.trailLast = d.position.clone();
      }
    }
    if (sim.powered !== this.wasPowered) { if (sim.powered) this.clearTrail(); this.wasPowered = sim.powered; }

    // follow cam + shadow camera follows drone
    if (this.follow) {
      const tgt = new THREE.Vector3(d.position.x, Math.max(0.5, d.position.y - 0.1), d.position.z);
      const move = tgt.sub(this.controls.target).multiplyScalar(1 - Math.exp(-dt * 4));
      this.controls.target.add(move);
      this.camera.position.add(move);
    }
    this.key.position.set(d.position.x + 4, 9, d.position.z - 3);
    this.key.target.position.set(d.position.x, 0, d.position.z);

    this.controls.update();
    this.updateTooltip();
    this.renderer.render(this.scene, this.camera);
  }
}
