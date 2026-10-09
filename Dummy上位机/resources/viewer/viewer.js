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

function setCamera(name) {
  const points = {
    front: [1.5, 0, 0.3], side: [0, -1.5, 0.3], right: [0, 1.5, 0.3],
    top: [0.001, -0.001, 1.7], isometric: [1.02, -1.2, 0.9],
  };
  cameraName = name;
  const reference = currentState?.mode === 'actual' && currentState?.actualKnown ? actual : target;
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
      material.opacity = ghost ? 0.18 : 1;
      material.transparent = ghost;
      material.depthWrite = !ghost;
      if (ghost && material.color) material.color.setHex(0x6481a6);
      if (selected && !ghost) {
        material.color?.lerp(new THREE.Color(0x4389ff), 0.55);
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
  const preview = state.mode === 'preview';
  document.getElementById('mode').textContent = preview ? '目标预览 · 不发送运动' : '实际位置 · 串口回读';
  const telemetry = document.getElementById('telemetry');
  telemetry.textContent = !state.actualKnown ? '尚无位置回读'
    : state.stale ? '回读已过期 · 保留最近姿态' : '位置数据有效';
  telemetry.classList.toggle('warning', state.actualKnown && state.stale);
  document.getElementById('solid-label').textContent = preview ? '目标预览' : '实际回读';
  document.getElementById('ghost-label').hidden = !preview || !state.actualKnown;
  document.getElementById('gripper-status').textContent = !preview && state.actualGripperKnown
    ? '夹爪：实际回读' : '夹爪：目标开度（无实际回读）';
  if (target && actual) {
    target.setJointValues(state.target || {});
    actual.setJointValues(state.actual || {});
    target.visible = preview;
    actual.visible = Boolean(state.actualKnown);
    styleRobot(target, false, state.highlight);
    styleRobot(actual, preview, state.highlight);
  }
  marker.visible = Boolean(state.marker);
  if (state.marker) {
    marker.position.set(...state.marker.position);
    // This is a target-position marker; it does not solve IK or certify a path.
    marker.rotation.set(...state.marker.orientation, 'ZYX');
  }
  document.getElementById('marker-label').hidden = !state.marker;
  if (previousCameraSeq !== state.cameraSeq) {
    previousCameraSeq = state.cameraSeq;
    setCamera(state.camera);
  }
  if (previousVisibilitySeq !== state.visibilitySeq) {
    previousVisibilitySeq = state.visibilitySeq;
    grid.visible = state.grid !== false;
    axes.visible = state.axes !== false;
    document.getElementById('grid').classList.toggle('active', grid.visible);
    document.getElementById('axes').classList.toggle('active', axes.visible);
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

document.querySelectorAll('[data-view]').forEach(button => {
  button.addEventListener('click', () => setCamera(button.dataset.view));
});
document.getElementById('grid').addEventListener('click', event => {
  grid.visible = !grid.visible;
  event.currentTarget.classList.toggle('active', grid.visible);
});
document.getElementById('axes').addEventListener('click', event => {
  axes.visible = !axes.visible;
  event.currentTarget.classList.toggle('active', axes.visible);
});
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
function animate() {
  if (disposed) return;
  requestAnimationFrame(animate);
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
