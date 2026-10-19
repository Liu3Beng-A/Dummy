import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import vm from 'node:vm';
import * as THREE from '../resources/viewer/vendor/three/three.module.js';

// Exercise the real framing and ResizeObserver code with Three's camera math.
// Native Qt/WebEngine QA separately covers real OrbitControls mouse input.
async function viewerCamera() {
  const source = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  const camera = new THREE.PerspectiveCamera(38, 1, .005, 15);
  camera.up.set(0, 0, 1);
  const controls = { target: new THREE.Vector3(), update: () => camera.lookAt(controls.target) };
  const robot = new THREE.Mesh(new THREE.BoxGeometry(.5, .25, .65));
  robot.position.set(.04, -.02, .32);
  const root = { clientWidth: 760, clientHeight: 450 };
  const sizes = [];
  let onResize;
  const context = vm.createContext({
    THREE, camera, controls, root, actual: robot, target: robot,
    currentState: { actualKnown: true }, cameraName: 'isometric',
    isLocalRgbPreview: () => false, jointTargetVisible: () => false,
    window: { robotViewer: { loaded: true } },
    renderer: { setSize: (width, height) => sizes.push([width, height]) },
    drawSchedule: { request: () => {} },
    ResizeObserver: class {
      constructor(callback) { onResize = callback; }
      observe(element) { assert.equal(element, root); }
    },
  });
  vm.runInContext(source.slice(source.indexOf('function setCamera('),
    source.indexOf('function prepareRobot(')), context);
  const resizeStart = source.indexOf('const resize = new ResizeObserver(');
  vm.runInContext(source.slice(resizeStart, source.indexOf('function animate(', resizeStart)), context);
  const pose = () => [...camera.position.toArray(), ...camera.quaternion.toArray(), ...controls.target.toArray()];
  function manualView() {
    camera.position.set(-.7, 1.1, 1.2);
    controls.target.set(.07, -.12, .31);
    controls.update();
  }
  return { camera, controls, root, sizes, pose, manualView,
    resize: (width, height) => { root.clientWidth = width; root.clientHeight = height; onResize(); },
    reset: () => context.setCamera('isometric') };
}

test('viewport resizing changes projection without losing orbit, pan or zoom', async () => {
  const view = await viewerCamera();
  view.manualView();
  const pose = view.pose();
  const projection = view.camera.projectionMatrix.toArray();
  for (const [width, height] of [[760, 450], [638, 345], [300, 450], [760, 450]]) {
    view.resize(width, height);
    assert.deepEqual(view.pose(), pose);
    assert.equal(view.camera.aspect, width / height);
    assert.deepEqual(view.sizes.at(-1), [width, height]);
    assert.notDeepEqual(view.camera.projectionMatrix.toArray(), projection);
  }
  const count = view.sizes.length;
  for (const [width, height] of [[0, 450], [760, 0], [0, 0]]) view.resize(width, height);
  assert.equal(view.sizes.length, count, 'Hidden views must not produce an invalid aspect ratio');
  assert.deepEqual(view.pose(), pose);
});

test('explicit reset still frames the model using the current viewport', async () => {
  const view = await viewerCamera();
  view.manualView();
  const manual = view.pose();
  view.resize(760, 450);
  view.reset();
  assert.notDeepEqual(view.pose(), manual);
  const wideDistance = view.camera.position.distanceTo(view.controls.target);
  const widePose = view.pose();
  view.resize(300, 450);
  assert.deepEqual(view.pose(), widePose);
  view.reset();
  assert(view.camera.position.distanceTo(view.controls.target) > wideDistance,
    'Only an explicit reset should move the camera back to fit a narrower viewport');
});
