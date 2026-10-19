import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import vm from 'node:vm';
import * as THREE from '../resources/viewer/vendor/three/three.module.js';

// Import the real dependency-free production decision; no DOM/WebGL/device.
const source = await readFile(new URL('../resources/viewer/target_visibility.js', import.meta.url), 'utf8');
const { jointTargetVisible, TARGET_FADE_MS, TargetVisibilityFade } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
const preview = overrides => ({ mode: 'preview', actualOnly: false,
  targetEnabled: true, targetMatched: false, actualKnown: true,
  rgbPreview: false, marker: null, ...overrides });

test('matching feedback hides target and a later difference restores it', () => {
  assert.equal(jointTargetVisible(preview()), true);
  assert.equal(jointTargetVisible(preview({ targetMatched: true })), false);
  assert.equal(jointTargetVisible(preview({ targetMatched: false })), true);
});

test('manual off remains off regardless of feedback and defaults remain compatible', () => {
  for (const targetMatched of [true, false]) {
    assert.equal(jointTargetVisible(preview({ targetEnabled: false, targetMatched })), false);
  }
  assert.equal(jointTargetVisible({ mode: 'preview' }), true);
  assert.equal(jointTargetVisible(), false);
  assert.equal(jointTargetVisible(null), false);
});

test('actual-only and actual display never reveal the joint target', () => {
  for (const targetMatched of [true, false]) {
    assert.equal(jointTargetVisible(preview({ mode: 'actual', targetMatched })), false);
    assert.equal(jointTargetVisible(preview({ actualOnly: true, targetMatched })), false);
  }
});

test('MoveL keeps its marker rather than showing a second joint model', () => {
  const marker = Object.freeze({ position: Object.freeze([.1, .2, .3]), orientation: Object.freeze([0, 0, 0]) });
  for (const targetMatched of [true, false]) {
    assert.equal(jointTargetVisible(preview({ marker, targetMatched })), false);
  }
});

test('unknown and stale feedback show the preview when Python clears targetMatched', () => {
  for (const flags of [{ actualKnown: false }, { stale: true }, { feedbackPaused: true }]) {
    assert.equal(jointTargetVisible(preview({ ...flags, targetMatched: false })), true);
  }
});

test('RGB preview with no known pose still suppresses a separate target model', () => {
  assert.equal(jointTargetVisible(preview({ rgbPreview: true, actualKnown: false })), false);
  assert.equal(jointTargetVisible(preview({ rgbPreview: true, actualKnown: true })), true);
  assert.equal(jointTargetVisible(preview({ rgbPreview: true, actualKnown: true, targetMatched: true })), false);
});

test('visibility decision never changes preferences, telemetry, RGB, or marker', () => {
  const state = preview({ actual: Object.freeze({ Joint1: .5 }), target: Object.freeze({ Joint1: .5 }),
    rgbFrame: Object.freeze([Object.freeze([0, 128, 64])]), targetMatched: true });
  const before = JSON.stringify(state);
  Object.freeze(state);
  assert.equal(jointTargetVisible(state), false);
  assert.equal(JSON.stringify(state), before);
});

const close = (actual, expected) => assert(Math.abs(actual - expected) < 1e-12, `${actual} != ${expected}`);

test('automatic target visibility fades over 500 ms with a smooth midpoint', () => {
  assert.equal(TARGET_FADE_MS, 500);
  const fade = new TargetVisibilityFade();
  assert.equal(fade.setState(preview(), 100), 0);
  close(fade.sample(225), .15625);
  close(fade.sample(350), .5);
  close(fade.sample(475), .84375);
  assert.equal(fade.sample(600), 1);
  fade.setState(preview({ targetMatched: true }), 1000);
  assert.equal(fade.sample(1000), 1);
  close(fade.sample(1250), .5);
  assert.equal(fade.sample(1500), 0);
});

test('fade duration follows elapsed time at different rendering frame rates', () => {
  for (const fps of [24, 30, 60, 144]) {
    const fade = new TargetVisibilityFade();
    fade.setState(preview(), 0);
    for (let time = 1000 / fps; time < 250; time += 1000 / fps) fade.sample(time);
    close(fade.sample(250), .5);
    for (let time = 250 + 1000 / fps; time < 500; time += 1000 / fps) fade.sample(time);
    assert.equal(fade.sample(500), 1);
  }
});

test('repeated 30 Hz feedback does not restart or stretch either transition', () => {
  const fade = new TargetVisibilityFade();
  fade.setState(preview(), 0);
  for (let tick = 1; tick <= 15; tick++) fade.setState(preview(), tick * 1000 / 30);
  assert.equal(fade.startedAt, 0);
  assert.equal(fade.sample(500), 1);
  fade.setState(preview({ targetMatched: true }), 1000);
  for (let tick = 1; tick <= 15; tick++) fade.setState(preview({ targetMatched: true }), 1000 + tick * 1000 / 30);
  assert.equal(fade.startedAt, 1000);
  assert.equal(fade.sample(1500), 0);
});

test('reversing a fade is continuous and uses 500 ms from its current opacity', () => {
  const fade = new TargetVisibilityFade();
  fade.setState(preview(), 0);
  const first = fade.sample(250);
  assert.equal(fade.setState(preview({ targetMatched: true }), 250), first);
  assert.equal(fade.from, first);
  close(fade.sample(500), .25);
  const second = fade.sample(550);
  assert.equal(fade.setState(preview(), 550), second);
  assert.equal(fade.from, second);
  assert.equal(fade.sample(550), second);
  close(fade.sample(800), second + (1 - second) * .5);
  assert.equal(fade.sample(1050), 1);
});

test('a long hidden-tab interval completes the transition without replaying it', () => {
  const fade = new TargetVisibilityFade();
  fade.setState(preview(), 0);
  assert.equal(fade.sample(100000), 1);
  fade.setState(preview({ targetMatched: true }), 100000);
  assert.equal(fade.sample(100001), fade.value);
  assert.equal(fade.sample(900000), 0);
});

test('master, actual-only, actual-mode, MoveL and local RGB gates clear immediately', () => {
  const gates = [undefined, null, preview({ targetEnabled: false }), preview({ actualOnly: true }),
    preview({ mode: 'actual' }), preview({ marker: { position: [.1, .2, .3] } }),
    preview({ rgbPreview: true, actualKnown: false })];
  for (const gate of gates) {
    const fade = new TargetVisibilityFade();
    fade.setState(preview(), 0);
    assert.equal(fade.sample(500), 1);
    assert.equal(fade.setState(gate, 501), 0);
    assert.equal(fade.value, 0);
    assert.equal(fade.from, 0);
    assert.equal(fade.to, 0);
    assert.equal(fade.sample(1001), 0);
    assert.equal(fade.setState(preview(), 1002), 0);
    close(fade.sample(1252), .5);
    assert.equal(fade.sample(1502), 1);
  }
});

test('rapid matching changes retain a bounded finite opacity', () => {
  const fade = new TargetVisibilityFade();
  for (let tick = 0; tick < 600; tick++) {
    const now = tick * 7;
    const before = fade.sample(now);
    const after = fade.setState(preview({ targetMatched: tick % 11 < 5 }), now);
    close(after, before);
    for (const value of [before, after, fade.sample(now + 3)]) {
      assert(Number.isFinite(value) && value >= 0 && value <= 1, `invalid opacity ${value}`);
    }
  }
});

async function realViewerStylingContext() {
  const viewer = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  const labels = { ghost: { hidden: false, style: {} }, legend: { hidden: false },
    gripper: { textContent: '' } };
  const target = new THREE.Group();
  const meshMaterials = new Map();
  const ghostMaterials = new Set();
  const targetFade = new TargetVisibilityFade();
  const context = vm.createContext({ THREE, meshMaterials, ghostMaterials, targetFade, target,
    appliedTargetOpacity: -1, currentState: preview({ actualGripperKnown: true, theme: 'dark' }), marker: { visible: false },
    isLocalRgbPreview: () => false,
    document: { getElementById: name => name === 'ghost-label' ? labels.ghost
      : name === 'gripper-status' ? labels.gripper : null,
      querySelector: name => name === '.legend' ? labels.legend : null } });
  vm.runInContext(viewer.slice(viewer.indexOf('function isDiffuserMesh('), viewer.indexOf('function prepareDiffuser(')), context);
  vm.runInContext(viewer.slice(viewer.indexOf('function styleRobot('), viewer.indexOf('function setState(')), context);
  function addMesh(robot, linkName, { diffuser = false, array = false } = {}) {
    const link = new THREE.Group();
    link.name = linkName;
    link.isURDFLink = true;
    const visual = new THREE.Group();
    visual.name = diffuser ? 'rgb_diffuser' : `${linkName}_visual`;
    visual.isURDFVisual = true;
    const materials = [new THREE.MeshPhongMaterial({ color: 0x111111, emissive: 0x002233 })];
    if (array) materials.push(new THREE.MeshPhongMaterial({ color: 0x555555, emissive: 0x332211 }));
    for (const material of materials) meshMaterials.set(material, {
      color: material.color.clone(), emissive: material.emissive.clone(),
    });
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(), array ? materials : materials[0]);
    visual.add(mesh);
    link.add(visual);
    robot.add(link);
    return { mesh, materials };
  }
  return { context, target, targetFade, meshMaterials, ghostMaterials, labels, addMesh };
}

test('real viewer styling and RAF updates preserve base-opacity times fade and selected ratio', async () => {
  const { context, target, targetFade, ghostMaterials, labels, addMesh } = await realViewerStylingContext();
  const normal = addMesh(target, 'Link2', { array: true });
  const selected = addMesh(target, 'Link1');
  const ring = addMesh(target, 'base_link', { diffuser: true });
  targetFade.setState(preview(), 0);
  targetFade.sample(250);
  context.styleRobot(target, true, 'Joint1');
  assert.equal(ghostMaterials.size, 4);
  for (const material of [...normal.materials, ...ring.materials]) {
    close(material.userData.targetBaseOpacity, .12);
    close(material.opacity, .06);
  }
  close(selected.materials[0].userData.targetBaseOpacity, .22);
  close(selected.materials[0].opacity, .11);
  close(selected.materials[0].opacity / normal.materials[0].opacity, .22 / .12);
  for (const time of [300, 400, 500]) {
    context.animateTarget(time);
    for (const material of ghostMaterials) close(material.opacity, material.userData.targetBaseOpacity * targetFade.value);
  }
  // Repeated state styling must retain the current alpha rather than reset it.
  targetFade.setState(preview({ targetMatched: true }), 500);
  context.animateTarget(750);
  assert.equal(labels.ghost.hidden, false, 'The label remains present during fade-out');
  assert.equal(labels.ghost.style.opacity, '0.5');
  assert.equal(labels.gripper.textContent, '夹爪：实体回读 · 虚体目标');
  context.styleRobot(target, true, 'Joint2');
  for (const material of normal.materials) close(material.opacity, .11);
  close(selected.materials[0].opacity, .06);
  assert.equal(ghostMaterials.size, 4);
  context.animateTarget(1000);
  assert.equal(target.visible, false);
  assert.equal(labels.ghost.hidden, true);
  assert.equal(labels.ghost.style.opacity, '0');
  assert.equal(labels.gripper.textContent, '夹爪：实际回读');
  assert.equal(labels.legend.hidden, false, 'Known actual telemetry still needs its legend');
  for (const material of ghostMaterials) assert.equal(material.opacity, 0);
  context.animateTarget(2000);
  assert.equal(target.visible, false);
  // With no actual telemetry the legend also survives the final fade-out frame.
  context.currentState = preview({ actualKnown: false });
  targetFade.setState(context.currentState, 2000);
  context.animateTarget(2250);
  assert.equal(labels.legend.hidden, false);
  assert.equal(labels.ghost.hidden, false);
  targetFade.setState(preview({ actualKnown: false, targetMatched: true }), 2250);
  context.animateTarget(2500);
  assert.equal(labels.legend.hidden, false);
  assert.equal(labels.ghost.hidden, false);
  context.animateTarget(2750);
  assert.equal(labels.legend.hidden, true);
  assert.equal(labels.ghost.hidden, true);
});

test('light theme strengthens targets while a theme change preserves the running visibility fade', async () => {
  const { context, target, targetFade, addMesh } = await realViewerStylingContext();
  const normal = addMesh(target, 'Link2', { array: true });
  const selected = addMesh(target, 'Link1');
  targetFade.setState(preview(), 0);
  targetFade.sample(250);
  context.currentState.theme = 'light';
  context.styleRobot(target, true, 'Joint1');
  for (const material of normal.materials) close(material.opacity, .32 * .5);
  close(selected.materials[0].opacity, .46 * .5);
  close(targetFade.value, .5);
  context.currentState.theme = 'dark';
  context.styleRobot(target, true, 'Joint1');
  for (const material of normal.materials) close(material.opacity, .12 * .5);
  close(selected.materials[0].opacity, .22 * .5);
  context.animateTarget(500);
  close(normal.materials[0].opacity, .12);
  close(selected.materials[0].opacity, .22);
});

test('real viewer fading touches ghost materials while actual and its diffuser remain unchanged', async () => {
  const { context, target, targetFade, ghostMaterials, addMesh } = await realViewerStylingContext();
  addMesh(target, 'Link2');
  const actual = new THREE.Group();
  const solid = addMesh(actual, 'Link2');
  const ring = addMesh(actual, 'base_link', { diffuser: true });
  const ringMaterial = ring.materials[0];
  ringMaterial.emissive.setRGB(.03, 3, 4);
  ringMaterial.userData.diffuserMarker = 'live-rgb';
  const ringBefore = ringMaterial.toJSON();
  context.styleRobot(actual, false, null);
  const solidBefore = solid.materials[0].toJSON();
  targetFade.setState(preview(), 0);
  context.styleRobot(target, true, null);
  assert.equal(ghostMaterials.size, 1);
  for (const time of [100, 250, 500, 5000]) context.animateTarget(time);
  targetFade.setState(preview({ targetMatched: true }), 5000);
  for (const time of [5100, 5250, 5500]) context.animateTarget(time);
  assert.deepEqual(ringMaterial.toJSON(), ringBefore);
  assert.deepEqual(solid.materials[0].toJSON(), solidBefore);
  assert.equal(solid.materials[0].opacity, 1);
  assert.equal(solid.materials[0].transparent, false);
  assert.equal(solid.materials[0].depthWrite, true);
});
