import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from './vendor/mujoco/mujoco.js';

const root = document.querySelector('[data-spider-artifact]');
const stage = root.querySelector('[data-spider-stage]');
const canvas = root.querySelector('[data-spider-canvas]');
const playButton = root.querySelector('[data-play]');
const resetButton = root.querySelector('[data-reset]');
const cameraResetButton = root.querySelector('[data-camera-reset]');
const cameraFollowButton = root.querySelector('[data-camera-follow]');
const status = root.querySelector('[data-status]');
const chartCanvases = Object.fromEntries([...root.querySelectorAll('[data-chart]')].map((chart) => [chart.dataset.chart, chart]));
const glyphCanvases = Object.fromEntries([...root.querySelectorAll('[data-glyph]')].map((glyph) => [glyph.dataset.glyph, glyph]));
const glyphValues = Object.fromEntries([...root.querySelectorAll('[data-glyph-value]')].map((value) => [value.dataset.glyphValue, value]));
const reducedMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

const FOOT_NAMES = ['front_left', 'front_right', 'middle_left', 'middle_right', 'rear_left', 'rear_right'];
const TRIPOD_A = new Set([0, 3, 4]);
const COLORS = Array(6).fill('#030304');
const FOOT_COLOR = '#d90508';
const STEP_SECONDS = 0.002;
const MAX_FRAME_SECONDS = 0.05;
const TELEMETRY_WINDOW_SECONDS = 20;
const TELEMETRY_SAMPLE_SECONDS = 0.05;
const CHART_SURFACE = '#f2f5f8';
const CHART_GRID = 'rgba(47,105,173,.16)';
const CHART_AXIS = 'rgba(33,75,120,.38)';
const GLYPH_BASELINE = 'rgba(33,75,120,.5)';
const CHART_TEXT = '#4d6884';
const CHART_BLUE = '#2f69ad';
const CHART_INK = '#214b78';
const CHART_RED = '#b5433f';
const CAMERA_POSITION = new THREE.Vector3(1.1, -1.25, 0.92);
const CAMERA_TARGET = new THREE.Vector3(0, 0, 0.22);
const LOCAL_Y = new THREE.Vector3(0, 1, 0);

let mujoco;
let model;
let data;
let bodyAccessors;
let footAccessors;
let groundGeomId;
let footGeomIds;
let animation;
let previousWallTime;
let accumulator = 0;
let phase = 0;
let lastSimulationTime = 0;
let running = false;
let renderer;
let scene;
let camera;
let controls;
let resizeObserver;
let robotVisual;
let cameraFollow = false;
let telemetryHistory = [];
let lastTelemetrySampleTime = -Infinity;

function readout(name) {
  return root.querySelector(`[data-${name}]`);
}

function smoothstep(value) {
  const bounded = Math.max(0, Math.min(1, value));
  return bounded * bounded * (3 - 2 * bounded);
}

function gaitTarget(currentPhase, leg) {
  const offset = TRIPOD_A.has(leg) ? 0 : 0.5;
  const cycle = (currentPhase + offset) % 1;
  if (cycle < 0.5) {
    const progress = cycle / 0.5;
    return [0.32 * (2 * smoothstep(progress) - 1), 0.8];
  }
  const progress = (cycle - 0.5) / 0.5;
  return [0.32 * (1 - 2 * smoothstep(progress)), 0.8 - 0.55 * Math.sin(Math.PI * progress)];
}

function contactStates() {
  const contacts = Array(FOOT_NAMES.length).fill(false);
  const contactVector = data.contact;
  try {
    for (let index = 0; index < contactVector.size(); index += 1) {
      const contact = contactVector.get(index);
      const otherGeom = contact.geom1 === groundGeomId ? contact.geom2 : contact.geom2 === groundGeomId ? contact.geom1 : -1;
      const foot = footGeomIds.indexOf(otherGeom);
      if (foot !== -1) contacts[foot] = true;
      contact.delete();
    }
  } finally {
    contactVector.delete();
  }
  return contacts;
}

function bodyErrors() {
  const w = data.qpos[3], x = data.qpos[4], y = data.qpos[5], z = data.qpos[6];
  // MuJoCo's body-frame gravity is R^T * [0, 0, -9.81]. Only the
  // third row of the body-to-world rotation contributes here.
  const r20 = 2 * (x * z - y * w);
  const r21 = 2 * (y * z + x * w);
  const r22 = 1 - 2 * (x * x + y * y);
  const gravityX = -9.81 * r20;
  const gravityY = -9.81 * r21;
  const gravityZ = -9.81 * r22;
  return [
    Math.atan2(gravityY, -gravityZ),
    Math.atan2(-gravityX, -gravityZ),
    data.qvel[3],
    data.qvel[4],
  ];
}

function applyGaitControl() {
  const contacts = contactStates();
  const simulationTime = data.time;
  const starting = simulationTime < 1.0;
  const expectedStance = FOOT_NAMES.map((_, leg) => starting || ((phase + (TRIPOD_A.has(leg) ? 0 : 0.5)) % 1) < 0.5);
  const deltaTime = Math.max(0, simulationTime - lastSimulationTime);
  lastSimulationTime = simulationTime;
  if (!starting) phase = (phase + 0.65 * deltaTime) % 1;

  const [roll, pitch, rollRate, pitchRate] = bodyErrors();
  for (let leg = 0; leg < FOOT_NAMES.length; leg += 1) {
    let [hip, knee] = starting ? [0, 0.8] : gaitTarget(phase, leg);
    const side = leg % 2 === 0 ? 1 : -1;
    const foreAft = leg < 2 ? 1 : leg >= 4 ? -1 : 0;
    hip += -0.10 * pitch - 0.01 * pitchRate * foreAft;
    knee += 0.06 * (pitch * foreAft + roll * side);
    if (expectedStance[leg] && !contacts[leg]) knee += 0.08;
    if (!expectedStance[leg] && contacts[leg]) knee -= 0.08;
    data.ctrl[2 * leg] = Math.max(-0.8, Math.min(0.8, hip));
    data.ctrl[2 * leg + 1] = Math.max(-1.4, Math.min(1.4, knee));
  }
  return { contacts, roll, pitch };
}

function setStandingPose() {
  data.qpos.set([0, 0, 0.5, 1, 0, 0, 0]);
  for (let leg = 0; leg < FOOT_NAMES.length; leg += 1) {
    data.qpos[7 + 2 * leg] = 0;
    data.qpos[7 + 2 * leg + 1] = 0.8;
    data.ctrl[2 * leg] = 0;
    data.ctrl[2 * leg + 1] = 0.8;
  }
  mujoco.mj_forward(model, data);
}

function createSegment(color, radius) {
  const material = new THREE.MeshStandardMaterial({ color, metalness: 0.22, roughness: 0.45 });
  const segment = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 12), material);
  segment.castShadow = true;
  segment.receiveShadow = true;
  scene.add(segment);
  return segment;
}

function createRobotVisual() {
  const torso = new THREE.Group();
  const torsoMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.44, 0.32, 0.14),
    new THREE.MeshStandardMaterial({ color: 0x030304, metalness: 0.38, roughness: 0.34 }),
  );
  const torsoEdges = new THREE.LineSegments(
    new THREE.EdgesGeometry(torsoMesh.geometry),
    new THREE.LineBasicMaterial({ color: 0x333337, transparent: true, opacity: 0.58 }),
  );
  torsoMesh.castShadow = true;
  torsoMesh.receiveShadow = true;
  torso.add(torsoMesh, torsoEdges);
  scene.add(torso);

  const legs = FOOT_NAMES.map((_, index) => {
    const color = COLORS[index];
    const foot = new THREE.Mesh(
      new THREE.SphereGeometry(0.045, 18, 12),
      new THREE.MeshStandardMaterial({ color: FOOT_COLOR, emissive: 0x000000, metalness: 0.26, roughness: 0.34 }),
    );
    foot.castShadow = true;
    foot.receiveShadow = true;
    scene.add(foot);
    return { thigh: createSegment(color, 0.035), shin: createSegment(color, 0.029), foot };
  });

  robotVisual = {
    torso,
    legs,
    matrix: new THREE.Matrix4(),
    start: new THREE.Vector3(),
    end: new THREE.Vector3(),
    direction: new THREE.Vector3(),
  };
}

function positionFrom(accessor, target) {
  return target.set(accessor.xpos[0], accessor.xpos[1], accessor.xpos[2]);
}

function setMuJoCoRotation(object, matrixValues) {
  robotVisual.matrix.set(
    matrixValues[0], matrixValues[1], matrixValues[2], 0,
    matrixValues[3], matrixValues[4], matrixValues[5], 0,
    matrixValues[6], matrixValues[7], matrixValues[8], 0,
    0, 0, 0, 1,
  );
  object.quaternion.setFromRotationMatrix(robotVisual.matrix);
}

function setSegment(segment, start, end) {
  const direction = robotVisual.direction.subVectors(end, start);
  const length = direction.length();
  segment.position.addVectors(start, end).multiplyScalar(0.5);
  segment.scale.set(1, Math.max(length, 0.0001), 1);
  segment.quaternion.setFromUnitVectors(LOCAL_Y, direction.multiplyScalar(1 / Math.max(length, 0.0001)));
}

function updateRobotVisual(contacts) {
  const torso = bodyAccessors.torso;
  positionFrom(torso, robotVisual.torso.position);
  setMuJoCoRotation(robotVisual.torso, torso.xmat);

  FOOT_NAMES.forEach((name, index) => {
    const visual = robotVisual.legs[index];
    const hip = positionFrom(bodyAccessors[name], robotVisual.start);
    const knee = positionFrom(bodyAccessors[`${name}_shin`], robotVisual.end);
    setSegment(visual.thigh, hip, knee);
    const foot = positionFrom(footAccessors[name], robotVisual.start);
    setSegment(visual.shin, knee, foot);
    visual.foot.position.copy(foot);
    visual.foot.material.color.set(FOOT_COLOR);
    visual.foot.material.emissive.set(contacts[index] ? 0x260000 : 0x000000);
  });
}

function renderScene() {
  if (renderer && scene && camera) renderer.render(scene, camera);
}

function resizeRenderer() {
  if (!renderer || !camera) return;
  const { width, height } = stage.getBoundingClientRect();
  if (!width || !height) return;
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderScene();
}

function resetCamera() {
  setCameraFollow(false);
  camera.position.copy(CAMERA_POSITION);
  controls.target.copy(CAMERA_TARGET);
  controls.update();
  renderScene();
}

function setCameraFollow(enabled) {
  cameraFollow = enabled;
  cameraFollowButton.textContent = enabled ? 'Following Spider' : 'Follow Spider';
  cameraFollowButton.setAttribute('aria-pressed', String(enabled));
  if (!enabled || !bodyAccessors) return;

  const target = positionFrom(bodyAccessors.torso, robotVisual.start);
  target.z -= 0.25;
  robotVisual.direction.subVectors(camera.position, controls.target);
  controls.target.copy(target);
  camera.position.copy(target).add(robotVisual.direction);
  controls.update();
  renderScene();
}

function updateFollowCamera() {
  if (!cameraFollow) return;
  const target = positionFrom(bodyAccessors.torso, robotVisual.start);
  target.z -= 0.25;
  robotVisual.direction.subVectors(target, controls.target);
  camera.position.add(robotVisual.direction);
  controls.target.copy(target);
}

function setupRenderer() {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.setClearColor(0xe8ecf1, 1);

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xe8ecf1);
  scene.fog = new THREE.Fog(0xe8ecf1, 5.5, 20);
  scene.up.set(0, 0, 1);

  camera = new THREE.PerspectiveCamera(42, 1, 0.05, 25);
  camera.up.set(0, 0, 1);
  camera.position.copy(CAMERA_POSITION);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.copy(CAMERA_TARGET);
  controls.enableDamping = true;
  controls.dampingFactor = 0.07;
  controls.enablePan = true;
  controls.minDistance = 0.45;
  controls.maxDistance = 4.5;
  controls.minPolarAngle = Math.PI * 0.1;
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
  controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };
  controls.addEventListener('change', renderScene);
  controls.addEventListener('start', () => setCameraFollow(false));
  controls.update();

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(24, 24),
    new THREE.MeshStandardMaterial({ color: 0xe8ecf1, metalness: 0.02, roughness: 0.9 }),
  );
  ground.position.z = -0.012;
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.GridHelper(24, 48, 0x7fa5c7, 0xc8d6e3);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.005;
  grid.material.transparent = true;
  grid.material.opacity = 0.36;
  scene.add(grid);

  scene.add(new THREE.HemisphereLight(0xffffff, 0xb7cbe0, 1.6));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2);
  keyLight.position.set(1.8, -1.2, 2.6);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(1024, 1024);
  scene.add(keyLight);
  const fillLight = new THREE.DirectionalLight(0x7fa5c7, 0.42);
  fillLight.position.set(-1.5, 1.2, 0.8);
  scene.add(fillLight);
  const rimLight = new THREE.DirectionalLight(0xffffff, 0.55);
  rimLight.position.set(-1.8, -1.4, 1.7);
  scene.add(rimLight);

  createRobotVisual();
  resizeObserver = new ResizeObserver(resizeRenderer);
  resizeObserver.observe(stage);
  resizeRenderer();
}

function updateReadout(state) {
  readout('time').textContent = `${data.time.toFixed(2)} s`;
  readout('x').textContent = `${data.qpos[0].toFixed(3)} m`;
  readout('z').textContent = `${data.qpos[2].toFixed(3)} m`;
  readout('contacts').textContent = `${state.contacts.filter(Boolean).length} / 6`;
  readout('attitude').textContent = `${state.roll.toFixed(3)} / ${state.pitch.toFixed(3)} rad`;
  readout('torque').textContent = `${data.qfrc_actuator[0].toFixed(3)} N·m`;
}

function recordTelemetry(state) {
  if (telemetryHistory.length && data.time - lastTelemetrySampleTime < TELEMETRY_SAMPLE_SECONDS) return;
  telemetryHistory.push({
    time: data.time,
    position: data.qpos[0],
    height: data.qpos[2],
    velocity: data.qvel[0],
    roll: state.roll,
    pitch: state.pitch,
  });
  lastTelemetrySampleTime = data.time;
  const firstVisibleTime = data.time - TELEMETRY_WINDOW_SECONDS;
  while (telemetryHistory.length > 1 && telemetryHistory[0].time < firstVisibleTime) telemetryHistory.shift();
  drawTelemetryCharts();
  drawTelemetryGlyphs();
}

function chartBounds(keys, { includeZero = false, minimum, minimumSpan = 0.1 } = {}) {
  const values = telemetryHistory.flatMap((sample) => keys.map((key) => sample[key]));
  let lower = Math.min(...values);
  let upper = Math.max(...values);
  if (includeZero) {
    lower = Math.min(lower, 0);
    upper = Math.max(upper, 0);
  }
  if (minimum !== undefined) lower = Math.min(lower, minimum);
  const span = Math.max(upper - lower, minimumSpan);
  const padding = span * 0.13;
  return { lower: minimum === undefined ? lower - padding : lower, upper: upper + padding };
}

function prepareChart(chartCanvas) {
  const bounds = chartCanvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(bounds.width));
  const height = Math.max(1, Math.round(bounds.height));
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  if (chartCanvas.width !== width * pixelRatio || chartCanvas.height !== height * pixelRatio) {
    chartCanvas.width = width * pixelRatio;
    chartCanvas.height = height * pixelRatio;
  }
  const context = chartCanvas.getContext('2d');
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  context.clearRect(0, 0, width, height);
  return { context, width, height };
}

function drawChart(chartName, series, bounds, unit, label) {
  const chartCanvas = chartCanvases[chartName];
  if (!chartCanvas || !telemetryHistory.length) return;
  const { context, width, height } = prepareChart(chartCanvas);
  const plot = { left: 40, top: 15, right: 10, bottom: 24 };
  const plotWidth = width - plot.left - plot.right;
  const plotHeight = height - plot.top - plot.bottom;
  const startTime = telemetryHistory[0].time;
  const endTime = Math.max(telemetryHistory.at(-1).time, startTime + 1);
  const timeRange = endTime - startTime;
  const valueRange = Math.max(bounds.upper - bounds.lower, 0.0001);
  const x = (time) => plot.left + (time - startTime) / timeRange * plotWidth;
  const y = (value) => plot.top + (1 - (value - bounds.lower) / valueRange) * plotHeight;

  context.fillStyle = CHART_SURFACE;
  context.fillRect(0, 0, width, height);
  context.strokeStyle = CHART_GRID;
  context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const guideY = plot.top + plotHeight * index / 4;
    context.beginPath();
    context.moveTo(plot.left, guideY);
    context.lineTo(width - plot.right, guideY);
    context.stroke();
  }
  context.strokeStyle = CHART_AXIS;
  context.beginPath();
  context.moveTo(plot.left, plot.top);
  context.lineTo(plot.left, height - plot.bottom);
  context.lineTo(width - plot.right, height - plot.bottom);
  context.stroke();

  context.fillStyle = CHART_TEXT;
  context.font = '10px JetBrains Mono, monospace';
  context.textBaseline = 'middle';
  context.fillText(`${bounds.upper.toFixed(2)} ${unit}`, 1, plot.top + 2);
  context.fillText(`${bounds.lower.toFixed(2)} ${unit}`, 1, height - plot.bottom - 2);
  context.textBaseline = 'alphabetic';
  context.fillText(`${startTime.toFixed(1)} s`, plot.left, height - 5);
  const endLabel = `${endTime.toFixed(1)} s`;
  context.fillText(endLabel, width - plot.right - context.measureText(endLabel).width, height - 5);

  context.save();
  context.beginPath();
  context.rect(plot.left, plot.top, plotWidth, plotHeight);
  context.clip();
  series.forEach(({ key, color }) => {
    context.strokeStyle = color;
    context.lineWidth = 2;
    context.lineCap = 'round';
    context.lineJoin = 'round';
    context.beginPath();
    telemetryHistory.forEach((sample, index) => {
      if (index === 0) context.moveTo(x(sample.time), y(sample[key]));
      else context.lineTo(x(sample.time), y(sample[key]));
    });
    context.stroke();
  });
  context.restore();

  const latest = telemetryHistory.at(-1);
  const summary = series.map(({ key }) => `${key} ${latest[key].toFixed(3)} ${unit}`).join(', ');
  chartCanvas.setAttribute('aria-label', `${label} over the current simulation run. The horizontal axis is time. Latest: ${summary}.`);
}

function drawTelemetryCharts() {
  if (!telemetryHistory.length) return;
  drawChart('position', [{ key: 'position', color: CHART_BLUE }], chartBounds(['position'], { includeZero: true, minimumSpan: 0.1 }), 'm', 'Torso x position');
  drawChart('height', [{ key: 'height', color: CHART_INK }], chartBounds(['height'], { minimum: 0, minimumSpan: 0.25 }), 'm', 'Torso height');
  drawChart('attitude', [{ key: 'roll', color: CHART_BLUE }, { key: 'pitch', color: CHART_RED }], chartBounds(['roll', 'pitch'], { includeZero: true, minimumSpan: 0.2 }), 'rad', 'Body roll and pitch');
}

function drawTelemetryGlyph(name, series, bounds, unit, label) {
  const glyphCanvas = glyphCanvases[name];
  if (!glyphCanvas || !telemetryHistory.length) return;
  const { context, width, height } = prepareChart(glyphCanvas);
  const inset = 3;
  const startTime = telemetryHistory[0].time;
  const endTime = Math.max(telemetryHistory.at(-1).time, startTime + 1);
  const timeRange = endTime - startTime;
  const valueRange = Math.max(bounds.upper - bounds.lower, 0.0001);
  const x = (time) => inset + (time - startTime) / timeRange * (width - inset * 2);
  const y = (value) => inset + (1 - (value - bounds.lower) / valueRange) * (height - inset * 2);

  context.strokeStyle = GLYPH_BASELINE;
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(inset, height / 2);
  context.lineTo(width - inset, height / 2);
  context.stroke();

  const latest = telemetryHistory.at(-1);
  series.forEach(({ key, color }) => {
    context.strokeStyle = color;
    context.lineWidth = 1.6;
    context.lineCap = 'round';
    context.lineJoin = 'round';
    context.beginPath();
    telemetryHistory.forEach((sample, index) => {
      if (index === 0) context.moveTo(x(sample.time), y(sample[key]));
      else context.lineTo(x(sample.time), y(sample[key]));
    });
    context.stroke();
    context.fillStyle = color;
    context.beginPath();
    context.arc(x(latest.time), y(latest[key]), 1.8, 0, Math.PI * 2);
    context.fill();
  });

  const latestValue = series.length === 1
    ? `${latest[series[0].key].toFixed(2)} ${unit}`
    : `r ${latest.roll.toFixed(2)} · p ${latest.pitch.toFixed(2)}`;
  glyphValues[name].textContent = latestValue;
  glyphCanvas.setAttribute('aria-label', `${label} live trace over the current simulation run. The horizontal axis is time. Latest: ${latestValue}.`);
}

function drawTelemetryGlyphs() {
  if (!telemetryHistory.length) return;
  drawTelemetryGlyph('position', [{ key: 'position', color: CHART_BLUE }], chartBounds(['position'], { includeZero: true, minimumSpan: 0.1 }), 'm', 'Torso x position');
  drawTelemetryGlyph('velocity', [{ key: 'velocity', color: CHART_INK }], chartBounds(['velocity'], { includeZero: true, minimumSpan: 0.1 }), 'm/s', 'Forward velocity');
  drawTelemetryGlyph('attitude', [{ key: 'roll', color: CHART_BLUE }, { key: 'pitch', color: CHART_RED }], chartBounds(['roll', 'pitch'], { includeZero: true, minimumSpan: 0.2 }), 'rad', 'Body attitude');
}

function render() {
  const state = applyGaitControl();
  updateRobotVisual(state.contacts);
  updateFollowCamera();
  controls.update();
  renderScene();
  updateReadout(state);
  recordTelemetry(state);
}

function restart() {
  releaseAccessors();
  if (data) data.delete();
  data = new mujoco.MjData(model);
  phase = 0;
  lastSimulationTime = 0;
  accumulator = 0;
  telemetryHistory = [];
  lastTelemetrySampleTime = -Infinity;
  setStandingPose();
  cacheAccessors();
  render();
}

function stop() {
  running = false;
  playButton.textContent = 'Resume';
  playButton.setAttribute('aria-pressed', 'false');
}

function frame(now) {
  if (!running) return;
  const elapsed = Math.min((now - previousWallTime) / 1000, MAX_FRAME_SECONDS);
  previousWallTime = now;
  accumulator += elapsed;
  while (accumulator >= STEP_SECONDS) {
    applyGaitControl();
    mujoco.mj_step(model, data);
    accumulator -= STEP_SECONDS;
  }
  render();
  animation = requestAnimationFrame(frame);
}

function start() {
  if (running) return;
  running = true;
  previousWallTime = performance.now();
  playButton.textContent = 'Pause';
  playButton.setAttribute('aria-pressed', 'true');
  animation = requestAnimationFrame(frame);
}

function toggle() {
  if (running) stop(); else start();
}

function releaseAccessors() {
  if (bodyAccessors) Object.values(bodyAccessors).forEach((accessor) => accessor.delete());
  if (footAccessors) Object.values(footAccessors).forEach((accessor) => accessor.delete());
  bodyAccessors = undefined;
  footAccessors = undefined;
}

function cacheAccessors() {
  bodyAccessors = {};
  ['torso', ...FOOT_NAMES, ...FOOT_NAMES.map((name) => `${name}_shin`)].forEach((name) => {
    bodyAccessors[name] = data.body(name);
  });
  footAccessors = {};
  FOOT_NAMES.forEach((name) => {
    footAccessors[name] = data.geom(`${name}_foot`);
  });
}

function disposeRenderer() {
  if (resizeObserver) resizeObserver.disconnect();
  if (controls) controls.dispose();
  if (scene) {
    scene.traverse((object) => {
      if (object.geometry) object.geometry.dispose();
      if (object.material) {
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        materials.forEach((material) => material.dispose());
      }
    });
  }
  if (renderer) renderer.dispose();
}

async function initialise() {
  try {
    setupRenderer();
    const [loadedMujoco, modelXml, manifest] = await Promise.all([
      loadMujoco(),
      fetch('./model/spider.xml').then((response) => {
        if (!response.ok) throw new Error('The Spider model could not load.');
        return response.text();
      }),
      fetch('./manifest.json').then((response) => {
        if (!response.ok) throw new Error('The Spider release manifest could not load.');
        return response.json();
      }),
    ]);
    mujoco = loadedMujoco;
    model = mujoco.MjModel.from_xml_string(modelXml);
    const ground = model.geom('ground');
    groundGeomId = ground.id;
    ground.delete();
    footGeomIds = FOOT_NAMES.map((name) => {
      const geom = model.geom(`${name}_foot`);
      const id = geom.id;
      geom.delete();
      return id;
    });
    restart();
    status.textContent = `Live 3D · ${manifest.release} · MuJoCo ${mujoco.mj_versionString()} · ${manifest.spider_commit.slice(0, 8)}`;
    playButton.disabled = false;
    resetButton.disabled = false;
    cameraResetButton.disabled = false;
    cameraFollowButton.disabled = false;
    playButton.textContent = reducedMotion ? 'Start simulation' : 'Pause';
    playButton.addEventListener('click', toggle);
    resetButton.addEventListener('click', restart);
    cameraResetButton.addEventListener('click', resetCamera);
    cameraFollowButton.addEventListener('click', () => setCameraFollow(!cameraFollow));
    if (!reducedMotion) start();
  } catch (error) {
    disposeRenderer();
    status.textContent = 'Live simulation unavailable';
    stage.innerHTML = `<p class="explorer-error">${error.message} See the canonical Spider repository for the native simulation.</p>`;
  }
}

window.addEventListener('pagehide', () => {
  if (animation) cancelAnimationFrame(animation);
  if (data) data.delete();
  releaseAccessors();
  if (model) model.delete();
  disposeRenderer();
});

window.addEventListener('resize', drawTelemetryCharts);

initialise();
