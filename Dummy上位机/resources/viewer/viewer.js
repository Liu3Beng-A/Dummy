import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';
import URDFLoader from './vendor/urdf-loader/URDFLoader.js';
import { advanceRgbDisplayFrame, createAngularUv, createDiffuserKernel, DIFFUSER_SAMPLES, DIFFUSER_STYLE,
  normalizeRgbFrame, patchDiffuserShader, sameRgbFrame, writeDiffuserTexture } from './model_rgb.js';

const root = document.getElementById('canvas');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xedf2f8);
const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 15);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.2;
root.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.09;
controls.minDistance = 0.12;
controls.maxDistance = 4;
controls.target.set(0, 0, 0.28);

const hemi = new THREE.HemisphereLight(0xe9f1ff, 0x626c7c, 2.0);
hemi.position.set(0, 0, 1);
scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 3.0);
key.position.set(2, -3, 4);
scene.add(key);
const rim = new THREE.DirectionalLight(0xcbdfff, 1.5);
rim.position.set(-2, 1, 2);
scene.add(rim);
const grid = new THREE.GridHelper(2, 20, 0xb7c4d5, 0xd9e2ed);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.003;
scene.add(grid);
const axes = new THREE.AxesHelper(0.15);
// The world origin lies inside the rail/base meshes. Draw the axes as an
// overlay, after both solid and ghost robots, without moving the origin.
axes.material.depthTest = false;
axes.material.depthWrite = false;
axes.material.transparent = true;
axes.renderOrder = 10;
scene.add(axes);
const marker = new THREE.Group();
const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.009, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b }));
marker.add(sphere, new THREE.AxesHelper(0.055));
marker.visible = false;
scene.add(marker);

let actual = null;
let target = null;
let currentState = null;
let previousCameraSeq = -1;
let previousVisibilitySeq = -1;
let disposed = false;
const meshMaterials = new Map();
const diffuserMeshes = [];
const diffuserKernel = createDiffuserKernel();
// The split STL is registered to the existing base_link coordinate frame (m).
// Source zero points along +Y; increasing angles turn toward +X. This is the
// 2D ring's simulated clockwise order, not a claim about physical LED wiring.
const DIFFUSER_CENTER = [0, 0, 0.18996717780828476];
const DIFFUSER_U = [0, 1, 0];
const DIFFUSER_V = [1, 0, 0];
let rgbFrame = Array.from({ length: 24 }, () => [0, 0, 0]);
let rgbFrameUpdates = 0;
let displayedRgbFrame = null;
let rgbLightOn = false;
let rgbLightOnUpdates = 0;
const rgbCoverReflection = { value: DIFFUSER_STYLE.coverReflection };
let diffuserTexture = null;
let diffuserPixels = null;
let cameraName = 'isometric';
// Display-only damping. Keep currentState.actual as the unmodified feedback.
const ACTUAL_SMOOTHING_SECONDS = 0.05;
const FEEDBACK_JOINTS = new Set(['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'rail']);
let displayedActual = null;
let feedbackContinuous = false;
let previousFrameTime = null;

function updateActualState(state) {
  if (!state.actualKnown) {
    displayedActual = null;
    feedbackContinuous = false;
    return;
  }
  const values = state.actual || {};
  const continuous = !state.stale && !state.feedbackPaused && !document.hidden;
  // Never interpolate from an invented pose or replay a paused/expired session.
  if (!displayedActual || !feedbackContinuous || !continuous) {
    displayedActual = { ...values };
    actual.setJointValues(values);
  } else {
    // Fingers may illustrate a commanded opening rather than real feedback.
    actual.setJointValues(Object.fromEntries(Object.entries(values)
      .filter(([name]) => !FEEDBACK_JOINTS.has(name))));
  }
  feedbackContinuous = continuous;
}

function animateActual(dt) {
  if (!actual || !currentState?.actualKnown || !displayedActual) return;
  const values = currentState.actual || {};
  if (!feedbackContinuous || document.hidden || dt > 0.25) {
    displayedActual = { ...values };
    actual.setJointValues(values);
    return;
  }
  const updates = {};
  for (const name of FEEDBACK_JOINTS) {
    const goal = values[name];
    if (!Number.isFinite(goal)) continue;
    const previous = displayedActual[name] ?? goal;
    const damped = THREE.MathUtils.damp(previous, goal, 1 / ACTUAL_SMOOTHING_SECONDS, dt);
    const epsilon = name === 'rail' ? 1e-7 : 1e-6;
    const next = Math.abs(goal - damped) <= epsilon ? goal : damped;
    if (next !== previous) updates[name] = next;
    displayedActual[name] = next;
  }
  actual.setJointValues(updates);
}

function isLocalRgbPreview(state) {
  return state?.rgbPreview === true && state.actualOnly !== true && !state.actualKnown;
}

function setCamera(name) {
  const points = {
    front: [1.5, 0, 0.3], side: [0, -1.5, 0.3], right: [0, 1.5, 0.3],
    top: [0.001, -0.001, 1.7], isometric: [1.02, -1.2, 0.9],
  };
  cameraName = name;
  const jointPreview = currentState?.mode === 'preview'
    && currentState?.actualOnly !== true && currentState?.targetEnabled !== false && !currentState?.marker;
  const reference = isLocalRgbPreview(currentState) ? actual
    : jointPreview ? target : currentState?.actualKnown ? actual : null;
  const center = new THREE.Vector3(0, 0, 0.28);
  const outward = new THREE.Vector3(...(points[name] || points.isometric)).sub(center).normalize();
  let distance = 1.6;
  if (reference) {
    reference.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(reference);
    if (!box.isEmpty()) {
      box.getCenter(center);
      camera.position.copy(center).add(outward);
      camera.lookAt(center);
      camera.updateMatrixWorld(true);
      const right = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);
      const tanV = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2));
      const tanH = tanV * camera.aspect;
      distance = 0.35;
      for (const x of [box.min.x, box.max.x]) {
        for (const y of [box.min.y, box.max.y]) {
          for (const z of [box.min.z, box.max.z]) {
            const corner = new THREE.Vector3(x, y, z).sub(center);
            const depth = corner.dot(outward);
            distance = Math.max(distance,
              Math.abs(corner.dot(right)) / (tanH * 0.85) + depth,
              Math.abs(corner.dot(up)) / (tanV * 0.85) + depth);
          }
        }
      }
    }
  }
  camera.position.copy(center).addScaledVector(outward, distance);
  controls.target.copy(center);
  controls.update();
}
setCamera('isometric');

function prepareRobot(robot) {
  // Round-off in the exported radian limits otherwise clips 180 degree states.
  // Preview is bounded by firmware limits in Python; actual state is unbounded.
  Object.values(robot.joints).forEach(joint => { joint.ignoreLimits = true; });
  robot.traverse(object => {
    if (!object.isMesh) return;
    const originals = Array.isArray(object.material) ? object.material : [object.material];
    object.material = originals.map(material => {
      const own = material.clone();
      own.side = THREE.DoubleSide;
      meshMaterials.set(own, { color: own.color?.clone(), emissive: own.emissive?.clone() });
      return own;
    });
    if (object.material.length === 1) object.material = object.material[0];
  });
}

function isDiffuserMesh(object) {
  let node = object;
  while (node && !node.isURDFLink) {
    if (node.isURDFVisual && node.name === 'rgb_diffuser') return true;
    node = node.parent;
  }
  return false;
}

function prepareDiffuser(robot) {
  const pixels = new Uint16Array(DIFFUSER_SAMPLES * 4);
  diffuserPixels = new Float32Array(DIFFUSER_SAMPLES * 4);
  diffuserTexture = new THREE.DataTexture(pixels, DIFFUSER_SAMPLES, 1,
    THREE.RGBAFormat, THREE.HalfFloatType);
  // RGB stores bounded display emission in linear output space; alpha stores
  // PWM activity for diagnostics only. RGBA16F preserves low-light interpolation
  // and supports linear filtering in WebGL2 without float32 filter extensions.
  diffuserTexture.colorSpace = THREE.LinearSRGBColorSpace;
  diffuserTexture.wrapS = THREE.RepeatWrapping;
  diffuserTexture.wrapT = THREE.ClampToEdgeWrapping;
  diffuserTexture.minFilter = THREE.LinearFilter;
  diffuserTexture.magFilter = THREE.LinearFilter;
  diffuserTexture.generateMipmaps = false;
  robot.traverse(object => {
    if (!object.isMesh || !isDiffuserMesh(object)) return;
    // The ghost keeps the original geometry and its own cloned translucent material.
    const geometry = object.geometry.index ? object.geometry.toNonIndexed()
      : object.geometry.clone();
    geometry.setAttribute('uv', new THREE.BufferAttribute(createAngularUv(
      geometry.getAttribute('position').array, DIFFUSER_CENTER, DIFFUSER_U, DIFFUSER_V), 2));
    object.geometry = geometry;
    object.userData.rgbDiffuser = true;
    diffuserMeshes.push(object);
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) {
      // The neutral pigment/unlit baseline is fixed. The shader continuously
      // tints/fades its reflection using local RGB emission, never power metadata.
      material.color.setRGB(DIFFUSER_STYLE.coverGray, DIFFUSER_STYLE.coverGray, DIFFUSER_STYLE.coverGray);
      material.specular?.setHex(0x20252a);
      material.shininess = 24;
      material.emissive.setRGB(1, 1, 1);
      material.emissiveIntensity = DIFFUSER_STYLE.emissionIntensity;
      material.emissiveMap = diffuserTexture;
      material.onBeforeCompile = shader => {
        shader.uniforms.rgbCoverReflection = rgbCoverReflection;
        shader.uniforms.rgbHighlightKnee = { value: DIFFUSER_STYLE.highlightKnee };
        shader.uniforms.rgbCoverTintStrength = { value: DIFFUSER_STYLE.coverTintStrength };
        shader.uniforms.rgbCoverFadeStrength = { value: DIFFUSER_STYLE.coverFadeStrength };
        shader.uniforms.rgbCoverFadePeak = { value: DIFFUSER_STYLE.coverFadePeak };
        shader.fragmentShader = patchDiffuserShader(shader.fragmentShader);
      };
      material.customProgramCacheKey = () => 'rgb-opal-cover-v6-chroma-reflection';
      material.userData.rgbFixedCover = true;
      material.needsUpdate = true;
    }
  });
  updateDiffuserTexture();
}

function updateDiffuserTexture() {
  if (!diffuserTexture || !diffuserPixels) return;
  writeDiffuserTexture(displayedRgbFrame || rgbFrame, diffuserKernel, diffuserPixels);
  const halfPixels = diffuserTexture.image.data;
  for (let index = 0; index < diffuserPixels.length; index++) {
    halfPixels[index] = THREE.DataUtils.toHalfFloat(diffuserPixels[index]);
  }
  diffuserTexture.needsUpdate = true;
}

function setRgbFrame(frame, lightOn = undefined) {
  if (disposed) return false;
  const normalized = normalizeRgbFrame(frame);
  if (!normalized || (lightOn !== undefined && typeof lightOn !== 'boolean')) return false;
  const first = displayedRgbFrame === null;
  // Legacy arrays preserve the last known power state. Their first frame
  // defaults to powered; zero PWM alone cannot identify an overall switch-off.
  const nextLightOn = lightOn === undefined ? (first ? true : rgbLightOn) : lightOn;
  if (rgbLightOn !== nextLightOn) rgbLightOnUpdates++;
  rgbLightOn = nextLightOn;
  if (!sameRgbFrame(rgbFrame, normalized)) {
    rgbFrame = normalized;
    rgbFrameUpdates++;
  }
  if (first || document.hidden) {
    displayedRgbFrame = rgbFrame.map(color => color.slice());
    updateDiffuserTexture();
  }
  return true;
}

function animateRgb(dt) {
  if (!displayedRgbFrame) return;
  // Only an actual rendering pause snaps. A long interval between deduplicated
  // input messages is normal during a breathing trough and never resets damping.
  const snap = document.hidden || dt > 0.25;
  const changed = advanceRgbDisplayFrame(displayedRgbFrame, rgbFrame,
    snap ? 0.26 : dt, DIFFUSER_STYLE.smoothingSeconds);
  if (changed) updateDiffuserTexture();
}

function styleRobot(robot, ghost, highlight) {
  const highlightedLinks = {
    Joint1: 'Link1', Joint2: 'Link2', Joint3: 'Link3', Joint4: 'Link4',
    Joint5: 'Link5', Joint6: 'Link6', rail: 'base_link',
  };
  robot.traverse(object => {
    if (!object.isMesh) return;
    const diffuser = isDiffuserMesh(object);
    // Preserve the actual ring's continuously updated emission even when a
    // joint/state/highlight update restyles the remainder of the robot.
    if (!ghost && diffuser) return;
    let link = object;
    while (link.parent && !link.isURDFLink) link = link.parent;
    const selected = link.name === highlightedLinks[highlight]
      || (highlight === 'gripper' && ['Hand_R', 'Hand_L'].includes(link.name));
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) {
      const base = meshMaterials.get(material);
      if (base?.color) material.color.copy(base.color);
      if (base?.emissive) material.emissive.copy(base.emissive);
      material.opacity = ghost ? (selected ? 0.22 : 0.12) : 1;
      material.transparent = ghost;
      material.depthWrite = !ghost;
      if (ghost && material.color) material.color.setHex(0xe8edf3);
      if (ghost && diffuser) material.emissive?.setRGB(0, 0, 0);
      if (selected && !diffuser) {
        if (!ghost) material.color?.lerp(new THREE.Color(0x4389ff), 0.55);
        material.emissive?.setHex(ghost ? 0x454545 : 0x16386d);
      }
    }
    object.renderOrder = ghost ? 2 : 0;
  });
}

function setState(state) {
  currentState = state;
  const dark = state.theme === 'dark';
  document.body.classList.toggle('dark', dark);
  scene.background.setHex(dark ? 0x141e2c : 0xedf2f8);
  const actualOnly = state.actualOnly === true;
  const feedbackPaused = state.feedbackPaused === true;
  const localRgbPreview = isLocalRgbPreview(state);
  const preview = !actualOnly && state.mode === 'preview';
  const markerVisible = !actualOnly && Boolean(state.marker);
  const jointPreview = preview && state.targetEnabled !== false && !markerVisible && !localRgbPreview;
  document.getElementById('mode').textContent = localRgbPreview ? '灯效预览 · 本地示意'
    : actualOnly ? feedbackPaused
    ? '最近实际姿态 · 回读已暂停' : '实际关节 · 串口回读'
    : preview ? '目标预览 · 不发送运动' : '实际位置 · 串口回读';
  const telemetry = document.getElementById('telemetry');
  telemetry.textContent = !state.actualKnown ? '尚无位置回读'
    : feedbackPaused ? '保留最近姿态 · 非实时'
    : state.stale ? '回读已过期 · 保留最近姿态' : '位置数据有效';
  telemetry.classList.toggle('warning', state.actualKnown && (state.stale || feedbackPaused));
  const solidLabel = document.getElementById('solid-label');
  solidLabel.hidden = !state.actualKnown && !localRgbPreview;
  solidLabel.querySelector('span').textContent = localRgbPreview ? '灯效预览 · 本地示意'
    : feedbackPaused || state.stale
    ? '最近回读 · 实体' : '实际回读 · 实体';
  document.getElementById('ghost-label').hidden = !jointPreview;
  document.getElementById('gripper-status').textContent = localRgbPreview
    ? '夹爪：本地目标示意（无实际回读）'
    : state.actualKnown && state.actualGripperKnown
    ? jointPreview ? '夹爪：实体回读 · 虚体目标' : '夹爪：实际回读'
    : actualOnly ? state.commandedGripper != null
      ? `夹爪：已发送目标 ${state.commandedGripper}%（无实际回读）`
      : '夹爪：固定示意（无实际回读）'
    : '夹爪：目标开度（无实际回读）';
  if (target && actual) {
    target.setJointValues(state.target || {});
    updateActualState(state);
    // This uses the solid material only to preview its lamp. It never populates
    // feedback state; the first real readback still seeds actual without damping
    // from this locally posed model. No IK/TCP marker is derived from the pose.
    if (localRgbPreview) actual.setJointValues(state.target || {});
    target.visible = jointPreview;
    actual.visible = Boolean(state.actualKnown) || localRgbPreview;
    styleRobot(target, true, state.highlight);
    styleRobot(actual, false, preview || localRgbPreview ? null : state.highlight);
  }
  marker.visible = markerVisible;
  if (markerVisible) {
    marker.position.set(...state.marker.position);
    // This is a target-position marker; it does not solve IK or certify a path.
    marker.rotation.set(...state.marker.orientation, 'ZYX');
  }
  document.getElementById('marker-label').hidden = !markerVisible;
  document.querySelector('.legend').hidden = !state.actualKnown && !jointPreview
    && !markerVisible && !localRgbPreview;
  if (previousCameraSeq !== state.cameraSeq) {
    previousCameraSeq = state.cameraSeq;
    setCamera(state.camera);
  }
  if (previousVisibilitySeq !== state.visibilitySeq) {
    previousVisibilitySeq = state.visibilitySeq;
    grid.visible = state.grid !== false;
    axes.visible = state.axes !== false;
  }
}

window.robotViewer = {
  loaded: false, setState, setCamera, setRgbFrame,
  diagnostics: () => ({
    loaded: Boolean(actual && target), state: currentState,
    joints: actual ? Object.keys(actual.joints) : [],
    visuals: actual ? Object.keys(actual.visual).length : 0,
    renderedMeshes: actual ? countMeshes(actual) : 0,
    gridVisible: grid.visible, axesVisible: axes.visible,
    targetVisible: Boolean(target?.visible), actualVisible: Boolean(actual?.visible),
    rgbPreviewVisible: Boolean(actual?.visible && isLocalRgbPreview(currentState)),
    markerVisible: marker.visible,
    actualJointValues: actual ? Object.fromEntries(Object.entries(actual.joints)
      .map(([name, joint]) => [name, joint.jointValue])) : {},
    rgb: diffuserDiagnostics(),
  }),
};

function diffuserDiagnostics() {
  actual?.updateMatrixWorld(true);
  const bounds = new THREE.Box3();
  diffuserMeshes.forEach(mesh => bounds.union(new THREE.Box3().setFromObject(mesh)));
  const summarize = material => ({
    emissive: material.emissive?.toArray(), emissiveIntensity: material.emissiveIntensity,
    hasEmissiveMap: Boolean(material.emissiveMap), opacity: material.opacity,
    color: material.color?.toArray(), fixedCover: material.userData.rgbFixedCover === true,
  });
  const actualMaterials = diffuserMeshes.flatMap(object =>
    (Array.isArray(object.material) ? object.material : [object.material]).map(summarize));
  const targetMaterials = [];
  target?.traverse(object => {
    if (!object.isMesh || !isDiffuserMesh(object)) return;
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.forEach(material => targetMaterials.push(summarize(material)));
  });
  const linearSample = index => diffuserTexture ? Array.from(
    diffuserTexture.image.data.slice(index * 4, index * 4 + 3),
    value => THREE.DataUtils.fromHalfFloat(value)) : null;
  const sample = index => linearSample(index)?.map(value => Math.round(value * 255)) || null;
  const activity = index => diffuserTexture
    ? THREE.DataUtils.fromHalfFloat(diffuserTexture.image.data[index * 4 + 3]) : null;
  return {
    meshCount: diffuserMeshes.length,
    frame: rgbFrame.map(color => color.slice()), frameUpdates: rgbFrameUpdates,
    displayFrame: displayedRgbFrame?.map(color => color.slice()) || null,
    lightOn: rgbLightOn, lightOnUpdates: rgbLightOnUpdates,
    coverReflection: rgbCoverReflection.value, coverReflectionTarget: DIFFUSER_STYLE.coverReflection,
    displayStyle: { ...DIFFUSER_STYLE },
    textureFormat: 'rgba16f',
    localCenter: DIFFUSER_CENTER.slice(), sourceZeroDirection: DIFFUSER_U.slice(),
    worldBounds: bounds.isEmpty() ? null : { min: bounds.min.toArray(), max: bounds.max.toArray() },
    worldCenter: bounds.isEmpty() ? null : bounds.getCenter(new THREE.Vector3()).toArray(),
    textureSamples: { source0: sample(0), source6: sample(72), source12: sample(144), source18: sample(216) },
    textureLinearSamples: { source0: linearSample(0), source6: linearSample(72),
      source12: linearSample(144), source18: linearSample(216) },
    textureActivitySamples: { source0: activity(0), source6: activity(72),
      source12: activity(144), source18: activity(216) },
    textureLit: Boolean(diffuserTexture && diffuserTexture.image.data.some(
      (value, index) => index % 4 !== 3 && value > 0)),
    actualMaterials, targetMaterials,
  };
}

function countMeshes(object) {
  let count = 0;
  object.traverse(child => { if (child.isMesh) count++; });
  return count;
}

const manager = new THREE.LoadingManager();
let sourceRobot = null;
let failedAsset = false;
manager.onError = url => {
  failedAsset = true;
  window.reportRobotError('模型资源无法读取：' + url.split('/').pop());
};
manager.onLoad = () => {
  if (disposed || failedAsset || !sourceRobot) return;
  if (countMeshes(sourceRobot) !== 11) {
    window.reportRobotError('模型网格不完整，预期 11 个可视网格。');
    return;
  }
  actual = sourceRobot;
  target = sourceRobot.clone(true);
  prepareRobot(actual);
  prepareRobot(target);
  prepareDiffuser(actual);
  if (diffuserMeshes.length !== 1) {
    window.reportRobotError('RGB 散光罩模型不完整，预期 1 个灯环网格。');
    return;
  }
  styleRobot(actual, false, null);
  styleRobot(target, true, null);
  scene.add(actual, target);
  actual.visible = false;
  target.setJointValues({ Joint2: -75 * Math.PI / 180, Joint3: Math.PI });
  document.getElementById('error').hidden = true;
  window.robotViewer.loaded = true;
  if (window.pendingRobotState) setState(window.pendingRobotState);
  if (window.pendingRgbFrame) {
    const pending = window.pendingRgbFrame;
    if (Array.isArray(pending)) setRgbFrame(pending);
    else setRgbFrame(pending.colors, pending.light_on);
  }
  window.robotBridge?.viewerReady();
};
const loader = new URDFLoader(manager);
loader.packages = { DummyRobot_URDF: new URL('../models/DummyRobot_URDF', location.href).href };
loader.parseVisual = true;
loader.parseCollision = false;
loader.load(new URL('../models/DummyRobot_URDF/urdf/DummyRobot_URDF.urdf', location.href).href,
  robot => { sourceRobot = robot; }, undefined,
  error => window.reportRobotError(error?.message || String(error)));

renderer.domElement.addEventListener('webglcontextlost', event => {
  event.preventDefault();
  window.reportRobotError('显卡上下文丢失，请重新启动三维视图。');
});
const resize = new ResizeObserver(() => {
  const width = root.clientWidth;
  const height = root.clientHeight;
  if (!width || !height) return;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
  if (window.robotViewer?.loaded) setCamera(cameraName);
});
resize.observe(root);
function animate(time = performance.now()) {
  if (disposed) return;
  requestAnimationFrame(animate);
  const dt = previousFrameTime == null ? 0 : Math.max(0, (time - previousFrameTime) / 1000);
  previousFrameTime = time;
  animateActual(dt);
  animateRgb(dt);
  controls.update();
  // An obscured/tabbed view can skip expensive GL drawing while preserving state.
  if (!document.hidden) renderer.render(scene, camera);
}
animate();
window.addEventListener('pagehide', () => {
  disposed = true;
  resize.disconnect();
  controls.dispose();
  const geometries = new Set();
  scene.traverse(object => {
    if (object.geometry) geometries.add(object.geometry);
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.filter(Boolean).forEach(material => material.dispose());
  });
  geometries.forEach(geometry => geometry.dispose());
  diffuserTexture?.dispose();
  diffuserTexture = null;
  diffuserPixels = null;
  meshMaterials.clear();
  diffuserMeshes.length = 0;
  renderer.dispose();
});
