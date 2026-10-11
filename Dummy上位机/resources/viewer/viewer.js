import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';
import URDFLoader from './vendor/urdf-loader/URDFLoader.js';

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

function setCamera(name) {
  const points = {
    front: [1.5, 0, 0.3], side: [0, -1.5, 0.3], right: [0, 1.5, 0.3],
    top: [0.001, -0.001, 1.7], isometric: [1.02, -1.2, 0.9],
  };
  cameraName = name;
  const jointPreview = currentState?.mode === 'preview'
    && currentState?.actualOnly !== true && !currentState?.marker;
  const reference = jointPreview ? target : currentState?.actualKnown ? actual : null;
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

function styleRobot(robot, ghost, highlight) {
  const highlightedLinks = {
    Joint1: 'Link1', Joint2: 'Link2', Joint3: 'Link3', Joint4: 'Link4',
    Joint5: 'Link5', Joint6: 'Link6', rail: 'base_link',
  };
  robot.traverse(object => {
    if (!object.isMesh) return;
    let link = object;
    while (link.parent && !link.isURDFLink) link = link.parent;
    const selected = link.name === highlightedLinks[highlight]
      || (highlight === 'gripper' && ['Hand_R', 'Hand_L'].includes(link.name));
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) {
      const base = meshMaterials.get(material);
      if (base?.color) material.color.copy(base.color);
      if (base?.emissive) material.emissive.copy(base.emissive);
      material.opacity = ghost ? (selected ? 0.42 : 0.28) : 1;
      material.transparent = ghost;
      material.depthWrite = !ghost;
      if (ghost && material.color) material.color.setHex(0x4389ff);
      if (selected) {
        if (!ghost) material.color?.lerp(new THREE.Color(0x4389ff), 0.55);
        material.emissive?.setHex(0x16386d);
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
  const preview = !actualOnly && state.mode === 'preview';
  const markerVisible = !actualOnly && Boolean(state.marker);
  const jointPreview = preview && !markerVisible;
  document.getElementById('mode').textContent = actualOnly ? feedbackPaused
    ? '最近实际姿态 · 回读已暂停' : '实际关节 · 串口回读'
    : preview ? '目标预览 · 不发送运动' : '实际位置 · 串口回读';
  const telemetry = document.getElementById('telemetry');
  telemetry.textContent = !state.actualKnown ? '尚无位置回读'
    : feedbackPaused ? '保留最近姿态 · 非实时'
    : state.stale ? '回读已过期 · 保留最近姿态' : '位置数据有效';
  telemetry.classList.toggle('warning', state.actualKnown && (state.stale || feedbackPaused));
  const solidLabel = document.getElementById('solid-label');
  solidLabel.hidden = !state.actualKnown;
  solidLabel.querySelector('span').textContent = feedbackPaused || state.stale
    ? '最近回读 · 实体' : '实际回读 · 实体';
  document.getElementById('ghost-label').hidden = !jointPreview;
  document.getElementById('gripper-status').textContent = state.actualKnown && state.actualGripperKnown
    ? jointPreview ? '夹爪：实体回读 · 虚体目标' : '夹爪：实际回读'
    : actualOnly ? state.commandedGripper != null
      ? `夹爪：已发送目标 ${state.commandedGripper}%（无实际回读）`
      : '夹爪：固定示意（无实际回读）'
    : '夹爪：目标开度（无实际回读）';
  if (target && actual) {
    target.setJointValues(state.target || {});
    updateActualState(state);
    target.visible = jointPreview;
    actual.visible = Boolean(state.actualKnown);
    styleRobot(target, true, state.highlight);
    styleRobot(actual, false, preview ? null : state.highlight);
  }
  marker.visible = markerVisible;
  if (markerVisible) {
    marker.position.set(...state.marker.position);
    // This is a target-position marker; it does not solve IK or certify a path.
    marker.rotation.set(...state.marker.orientation, 'ZYX');
  }
  document.getElementById('marker-label').hidden = !markerVisible;
  document.querySelector('.legend').hidden = !state.actualKnown && !jointPreview && !markerVisible;
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
  loaded: false, setState, setCamera,
  diagnostics: () => ({
    loaded: Boolean(actual && target), state: currentState,
    joints: actual ? Object.keys(actual.joints) : [],
    visuals: actual ? Object.keys(actual.visual).length : 0,
    renderedMeshes: actual ? countMeshes(actual) : 0,
    gridVisible: grid.visible, axesVisible: axes.visible,
    targetVisible: Boolean(target?.visible), actualVisible: Boolean(actual?.visible),
    markerVisible: marker.visible,
    actualJointValues: actual ? Object.fromEntries(Object.entries(actual.joints)
      .map(([name, joint]) => [name, joint.jointValue])) : {},
  }),
};

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
  if (failedAsset || !sourceRobot) return;
  if (countMeshes(sourceRobot) !== 10) {
    window.reportRobotError('模型网格不完整，预期 10 个可视网格。');
    return;
  }
  actual = sourceRobot;
  target = sourceRobot.clone(true);
  prepareRobot(actual);
  prepareRobot(target);
  styleRobot(actual, false, null);
  styleRobot(target, true, null);
  scene.add(actual, target);
  actual.visible = false;
  target.setJointValues({ Joint2: -75 * Math.PI / 180, Joint3: Math.PI });
  document.getElementById('error').hidden = true;
  window.robotViewer.loaded = true;
  if (window.pendingRobotState) setState(window.pendingRobotState);
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
  renderer.dispose();
});
