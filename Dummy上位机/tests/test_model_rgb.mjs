import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import vm from 'node:vm';
import * as THREE from '../resources/viewer/vendor/three/three.module.js';

// This production module has no Three/DOM dependencies. Loading it by a data
// URL also works when the Python application's directory has no package.json.
const source = await readFile(new URL('../resources/viewer/model_rgb.js', import.meta.url), 'utf8');
const { advanceRgbDisplayFrame, createAngularUv, createDiffuserKernel, dimLightWeight, DIFFUSER_SAMPLES, DIFFUSER_STYLE, normalizeRgbFrame, patchDiffuserShader, sameRgbFrame,
  writeDiffuserTexture } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
const frame = color => Array.from({ length: 24 }, () => color.slice());
const kernel = createDiffuserKernel();
const texture = colors => writeDiffuserTexture(colors, kernel, new Uint8Array(288 * 4));

async function bundledPhongShader() {
  const threeSource = await readFile(new URL('../resources/viewer/vendor/three/three.module.js', import.meta.url), 'utf8');
  const encodedPhong = threeSource.match(/const fragment\$6 = ("(?:[^"\\]|\\.)*");/);
  assert(encodedPhong, 'Expected the bundled MeshPhong fragment shader');
  return JSON.parse(encodedPhong[1]);
}

// Run the injected GLSL itself with scalar JS math and equivalent vector
// primitives. No cover/response/shoulder formula is copied into this test.
function evaluateProductionLight(patched, spin = false) {
  const names = ['rgbHighlightGain', 'rgbSoftHighlight', 'rgbLightResponse', 'rgbTintedCover', 'rgbSpinHighlight', 'rgbCompositeHighlight'];
  const declarations = names.map(name => {
    const helper = patched.match(new RegExp(`float\\s+${name}\\s*\\(([^)]*)\\)\\s*\\{([^{}]*)\\}`));
    assert(helper, `Expected the production ${name} helper`);
    const parameters = helper[1].replace(/\bfloat\s+/g, '');
    const body = helper[2].replace(/\bfloat\s+([A-Za-z_]\w*)\s*=/g, 'const $1 =')
      .replace(/\b(min|max|exp)\s*\(/g, 'Math.$1(');
    return `function ${name}(${parameters}) {${body}}`;
  }).join('\n');
  const uniforms = ['rgbHighlightKnee', 'rgbCoverTintStrength', 'rgbCoverFadeStrength', 'rgbCoverFadePeak',
    'rgbHighlightBoostStart', 'rgbHighlightBoostStrength', 'rgbSpinCurve', 'rgbSpinHighlightWhite'];
  const values = [DIFFUSER_STYLE.highlightKnee, DIFFUSER_STYLE.coverTintStrength,
    DIFFUSER_STYLE.coverFadeStrength, DIFFUSER_STYLE.coverFadePeak,
    DIFFUSER_STYLE.highlightBoostStart, DIFFUSER_STYLE.highlightBoostStrength,
    spin ? 1 : 0, DIFFUSER_STYLE.spinHighlightWhite];
  const helpers = new Function(...uniforms, 'mix', 'clamp', `${declarations}\nreturn {${names.join(',')}};`)(
    ...values, (first, second, amount) => first * (1 - amount) + second * amount,
    (value, lower, upper) => Math.min(Math.max(value, lower), upper));

  const emissionInjection = patched.match(/vec3 rgbDisplayEmission = totalEmissiveRadiance;[\s\S]*?totalEmissiveRadiance = vec3\(0\.0\);/);
  assert(emissionInjection, 'Expected the cached emission and selective gain injection');
  const emissionBody = emissionInjection[0]
    .replace('vec3 rgbDisplayEmission = totalEmissiveRadiance;', 'let rgbDisplayEmission = totalEmissiveRadiance.slice();')
    .replace('#ifdef USE_EMISSIVEMAP', 'if (useEmissiveMap) {')
    .replace('#endif', '}')
    .replace(/rgbDisplayEmission\s*\*=\s*([^;]+);/g,
      'rgbDisplayEmission = rgbDisplayEmission.map(value => value * ($1));')
    .replace('totalEmissiveRadiance = vec3(0.0);', 'totalEmissiveRadiance = [0, 0, 0];');
  const emit = new Function('rgbHighlightGain', 'totalEmissiveRadiance', 'emissiveColor', 'useEmissiveMap',
    `${emissionBody}\nreturn {emission: rgbDisplayEmission, removed: totalEmissiveRadiance, activity: emissiveColor.a};`);

  const injection = patched.match(/#include <tonemapping_fragment>([\s\S]*?)#include <colorspace_fragment>/);
  assert(injection, 'Expected the production post-tone-map composition');
  const indices = { r: 0, g: 1, b: 2 };
  const body = injection[1]
    .replace(/\bmax\((rgbDisplayEmission|gl_FragColor\.rgb), vec3\(0\.0\)\)/g, 'positiveRgb($1)')
    .replace(/vec3\s+([A-Za-z_]\w*)\s*=\s*vec3\(([^;]*)\);/g, 'const $1 = [$2];')
    .replace('vec3 rgbLightSum = rgbCover + rgbEmission;',
      'const rgbLightSum = rgbCover.map((value, channel) => value + rgbEmission[channel]);')
    .replace(/gl_FragColor\.rgb\s*=\s*vec3\(([^;]*)\);/g, 'const output = [$1];')
    .replace(/\b(?:float|vec3)\s+([A-Za-z_]\w*)\s*=/g, 'const $1 =')
    .replace(/\b(rgbEmission|rgbReflection|rgbLightSum)\.([rgb])/g,
      (_, name, channel) => `${name}[${indices[channel]}]`)
    .replace(/\bmax\s*\(/g, 'Math.max(');
  const compose = new Function(...names, 'positiveRgb', 'rgbDisplayEmission', 'gl_FragColor',
    `${body}\nreturn {response: rgbResponse, cover: rgbCover, light: rgbLightSum, output};`);
  return {
    gain: helpers.rgbHighlightGain,
    emit: (emission, activity, useEmissiveMap = true) => emit(helpers.rgbHighlightGain,
      emission, { a: activity }, useEmissiveMap),
    highlight: helpers.rgbSoftHighlight,
    response: helpers.rgbLightResponse,
    tint: helpers.rgbTintedCover,
    composite: helpers.rgbCompositeHighlight,
    spinHighlight: helpers.rgbSpinHighlight,
    compose: (reflection, emission) => compose(...names.map(name => helpers[name]),
      rgb => rgb.map(value => Math.max(value, 0)), emission, { rgb: reflection }),
  };
}

test('display interpolation smooths low PWM steps and never mutates its integer target', () => {
  const displayed = frame([1, 1, 1]);
  const target = frame([2, 2, 2]);
  const supplied = target.map(color => color.slice());
  assert(advanceRgbDisplayFrame(displayed, target, 1 / 60));
  assert(displayed[0][0] > 1 && displayed[0][0] < 2);
  let previous = displayed[0][0];
  for (let tick = 0; tick < 50; tick++) {
    advanceRgbDisplayFrame(displayed, target, 1 / 60);
    assert(displayed[0][0] >= previous && displayed[0][0] <= 2);
    previous = displayed[0][0];
  }
  assert.equal(displayed[0][0], 2);
  assert.deepEqual(target, supplied);
  assert(!advanceRgbDisplayFrame(displayed, target, 1 / 60));
  // An actual long drawing pause catches up once instead of replaying old light.
  const off = frame([0, 0, 0]);
  advanceRgbDisplayFrame(displayed, off, .4);
  assert(displayed.every(color => color.every(value => value === 0)));
  assert.throws(() => advanceRgbDisplayFrame(displayed, target, NaN), RangeError);
  assert.throws(() => advanceRgbDisplayFrame(displayed, target, -1), RangeError);
});

test('rotation-only shoulder opens middle levels while preserving the off cover and full head', async () => {
  const shader = patchDiffuserShader(await bundledPhongShader());
  const previous = evaluateProductionLight(shader);
  const spin = evaluateProductionLight(shader, true);
  for (const cover of [[0, 0, 0], [.1, .2, .3], [2, .05, 1.1]]) {
    assert.deepEqual(spin.compose(cover, [0, 0, 0]).output,
      previous.compose(cover, [0, 0, 0]).output);
    let last = 0;
    for (let value = 0; value <= 255; value++) {
      const pixels = new Float32Array(288 * 4);
      writeDiffuserTexture(frame([0, value, value]), kernel, pixels);
      const emission = previous.emit([...pixels.slice(0, 3)].map(v => v * DIFFUSER_STYLE.emissionIntensity), pixels[3]).emission;
      const result = spin.compose(cover, emission).output;
      assert(result.every(v => Number.isFinite(v) && v >= 0 && v <= 1));
      assert(result[1] >= last - 1e-10, `${value}: dominant channel should be monotonic`);
      last = result[1];
      if (value === 128 && cover.every(v => v <= 1)) {
        assert(result[1] < previous.compose(cover, emission).output[1] - .12);
      }
      if (value === 255) {
        assert(Math.abs(result[1] - previous.compose(cover, emission).output[1]) < .001);
      }
    }
  }
  const endpoint = DIFFUSER_STYLE.spinHighlightWhite;
  assert.equal(spin.spinHighlight(0), 0);
  assert.equal(spin.spinHighlight(endpoint), 1);
  assert.equal(spin.spinHighlight(endpoint * 2), 1);
  assert((1 - spin.spinHighlight(endpoint - 1e-5)) / 1e-5 < 1e-5);
});

test('RGB bridge validates and caches explicit modes, including changes with identical colors', async () => {
  const html = await readFile(new URL('../resources/viewer/index.html', import.meta.url), 'utf8');
  const inline = html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
  const context = vm.createContext({ window: { addEventListener() {} } });
  vm.runInContext(inline, context);
  const window = context.window;
  const supplied = frame([0, 128, 128]);
  assert(window.receiveRobotRgbPayload({ colors: supplied, light_on: true, effect_mode: 8 }));
  assert.equal(window.pendingRgbFrame.effect_mode, 8);
  const calls = [];
  window.robotViewer = { setRgbFrame: (...args) => { calls.push(args); return true; } };
  for (const mode of [3, 8, 7, null, 0, 12, 13, 14, 15, 16]) {
    assert(window.receiveRobotRgbPayload({ colors: supplied, light_on: true, effect_mode: mode }));
    assert.equal(calls.at(-1)[2], mode);
  }
  const pending = window.pendingRgbFrame;
  const count = calls.length;
  for (const mode of [-1, 17, 8.5, true, '8', NaN]) {
    assert.equal(window.receiveRobotRgbPayload({ colors: supplied, light_on: true, effect_mode: mode }), false);
    assert.equal(window.pendingRgbFrame, pending);
    assert.equal(calls.length, count);
  }
  assert(window.receiveRobotRgbPayload(supplied));
  assert.equal(calls.at(-1)[2], null, 'Legacy data must not retain a stale rotation profile');
});

test('independent automatic modes share the existing rotation contrast curve and reject invalid frames atomically', async () => {
  const viewer = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  const setter = viewer.slice(viewer.indexOf('function setRgbFrame('), viewer.indexOf('function animateRgb('));
  const context = vm.createContext({
    disposed: false, normalizeRgbFrame, sameRgbFrame, rgbEffectMode: null,
    diffuserTexture: { offset: { x: 0 } },
    rgbSpinCurve: { value: 0 }, displayedRgbFrame: frame([0, 0, 0]),
    document: { hidden: false }, rgbLightOn: true, rgbLightOnUpdates: 0,
    rgbFrame: frame([0, 0, 0]), rgbFrameUpdates: 0,
  });
  vm.runInContext(setter, context);
  const supplied = frame([0, 128, 128]);
  for (const mode of [13, 14, 15, 16, ...Array.from({ length: 13 }, (_, i) => i), null]) {
    assert(context.setRgbFrame(supplied, true, mode));
    assert.equal(context.rgbEffectMode, mode);
    assert.equal(context.diffuserTexture.offset.x, 0, 'Mode selection must not add an installation-angle compensation');
    assert.equal(context.rgbSpinCurve.value, mode === 8 || (mode >= 13 && mode <= 16) ? 1 : 0);
  }
  assert(context.setRgbFrame(supplied, true, 16));
  const previous = context.rgbFrame;
  const count = context.rgbFrameUpdates;
  for (const invalid of [17, -1, 13.1, true, '13']) {
    assert.equal(context.setRgbFrame(frame([255, 0, 0]), false, invalid), false);
    assert.equal(context.rgbEffectMode, 16);
    assert.equal(context.rgbFrame, previous);
    assert.equal(context.rgbFrameUpdates, count);
    assert.equal(context.rgbLightOn, true);
    assert.equal(context.rgbSpinCurve.value, 1);
    assert.equal(context.diffuserTexture.offset.x, 0);
  }
});

test('global registration survives deferred texture creation and mode changes without a two-peak offset', async () => {
  const viewer = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  const supplied = frame([0, 0, 0]);
  supplied[0] = [80, 160, 240]; // Asymmetric probe: two equal peaks would hide the rotation sign.
  const context = vm.createContext({
    disposed: false, normalizeRgbFrame, sameRgbFrame, rgbEffectMode: null,
    rgbSpinCurve: { value: 0 }, diffuserTexture: null, diffuserPixels: null,
    displayedRgbFrame: supplied.map(color => color.slice()), rgbFrame: supplied,
    document: { hidden: false }, rgbLightOn: true, rgbLightOnUpdates: 0, rgbFrameUpdates: 0,
    DIFFUSER_SAMPLES: 288, DIFFUSER_STYLE, diffuserKernel: kernel, writeDiffuserTexture,
    Float32Array, Uint16Array,
    THREE: {
      DataTexture: class {
        constructor(data) { this.image = { data }; this.offset = { x: 0, y: 0 }; }
      },
      DataUtils: { toHalfFloat: value => value },
    },
  });
  vm.runInContext(viewer.slice(viewer.indexOf('const DIFFUSER_CENTER'), viewer.indexOf('let rgbFrame')), context);
  vm.runInContext(viewer.slice(viewer.indexOf('function prepareDiffuser('), viewer.indexOf('function animateRgb(')), context);
  assert(context.setRgbFrame(supplied, true, 15));
  context.prepareDiffuser({ traverse() {} });
  assert.equal(context.diffuserTexture.offset.x, 0);
  const uv = createAngularUv([-.04, 0, 0, -.04, .001, 0, -.04, -.001, .001],
    [0, 0, 0], [-1, 0, 0], [0, 1, 0]);
  // -X is 90 degrees counterclockwise from the old +Y source-zero position.
  assert.equal((uv[0] + context.diffuserTexture.offset.x) % 1, 0);
  const originalTarget = context.rgbFrame;
  const originalDisplay = context.displayedRgbFrame;
  for (const mode of [13, 15, 14, 15, 16, 8, 0, null]) {
    assert(context.setRgbFrame(supplied, true, mode));
    assert.equal(context.diffuserTexture.offset.x, 0);
    assert.equal(context.rgbFrame, originalTarget);
    assert.equal(context.displayedRgbFrame, originalDisplay);
    assert.equal(context.rgbFrameUpdates, 0);
  }
  assert(context.setRgbFrame(supplied, true, 15));
  assert.equal(context.setRgbFrame(supplied, true, 17), false);
  assert.equal(context.diffuserTexture.offset.x, 0);
  assert.equal(context.rgbEffectMode, 15);
  assert.deepEqual(supplied[0], [80, 160, 240]);
});

test('production diffuser geometry and real Three texture map every mode to the same front-back axis', async () => {
  const viewer = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  const supplied = frame([0, 0, 0]);
  supplied[0] = [0, 128, 128];
  const context = vm.createContext({
    THREE, Float32Array, Uint16Array, normalizeRgbFrame, sameRgbFrame, createAngularUv,
    DIFFUSER_SAMPLES, DIFFUSER_STYLE, diffuserKernel: kernel, writeDiffuserTexture, patchDiffuserShader,
    disposed: false, rgbEffectMode: null, rgbSpinCurve: { value: 0 },
    rgbCoverReflection: { value: DIFFUSER_STYLE.coverReflection },
    diffuserTexture: null, diffuserPixels: null, diffuserMeshes: [], isDiffuserMesh: () => true,
    displayedRgbFrame: supplied.map(color => color.slice()), rgbFrame: supplied,
    document: { hidden: false }, rgbLightOn: true, rgbLightOnUpdates: 0, rgbFrameUpdates: 0,
  });
  vm.runInContext(viewer.slice(viewer.indexOf('const DIFFUSER_CENTER'), viewer.indexOf('let rgbFrame')), context);
  vm.runInContext(viewer.slice(viewer.indexOf('function prepareDiffuser('), viewer.indexOf('function animateRgb(')), context);
  const centerZ = .18996717780828476;
  const directions = [[.04, 0, centerZ], [0, .04, centerZ], [-.04, 0, centerZ], [0, -.04, centerZ]];
  const positions = directions.flatMap(point => [...point, ...point, ...point]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  const material = new THREE.MeshPhongMaterial();
  const mesh = new THREE.Mesh(geometry, material);
  context.prepareDiffuser({ traverse: callback => callback(mesh) });
  const map = material.emissiveMap;
  assert(map.isDataTexture);
  const shader = { uniforms: {}, fragmentShader: THREE.ShaderLib.phong.fragmentShader };
  material.onBeforeCompile(shader);
  assert(shader.fragmentShader.includes('#include <emissivemap_fragment>'));
  for (const mode of [15, 16, 0, 3, 8, 13, 14, null, 15]) {
    assert(context.setRgbFrame(supplied, true, mode));
    map.updateMatrix();
    const uv = mesh.geometry.getAttribute('uv');
    const indices = directions.map((_, index) => {
      const sampled = new THREE.Vector2(uv.getX(index * 3), uv.getY(index * 3)).applyMatrix3(map.matrix);
      return Math.round(((sampled.x % 1 + 1) % 1) * 24) % 24;
    });
    assert.deepEqual(indices, [12, 6, 0, 18], 'Front +X must emit LED12; rear -X must emit LED0 in every mode');
    assert.equal(map.offset.x, 0);
  }
  const oldUv = createAngularUv(positions, [0, 0, centerZ], [0, 1, 0], [1, 0, 0]);
  const newUv = mesh.geometry.getAttribute('uv');
  for (let vertex = 0; vertex < newUv.count; vertex++) {
    const previousTwoPeak = ((oldUv[vertex * 2] + .25) % 1 + 1) % 1;
    assert.equal((newUv.getX(vertex) % 1 + 1) % 1, previousTwoPeak, 'Two-peak placement must retain its former total 90-degree correction');
  }
  material.dispose();
  geometry.dispose();
  mesh.geometry.dispose();
  map.dispose();
});

test('opal shader adds independent light after ACES and softens highlights without changing opacity/output conversion', async () => {
  const original = await bundledPhongShader();
  const patched = patchDiffuserShader(original);
  const reflection = patched.indexOf('diffuseColor.rgb *= rgbCoverReflection;');
  const viewer = await readFile(new URL('../resources/viewer/viewer.js', import.meta.url), 'utf8');
  for (const uniform of ['rgbCoverReflection', 'rgbHighlightKnee', 'rgbCoverTintStrength',
    'rgbCoverFadeStrength', 'rgbCoverFadePeak', 'rgbHighlightBoostStart', 'rgbHighlightBoostStrength']) {
    assert(patched.includes(`uniform float ${uniform};`));
    const binding = viewer.indexOf(`shader.uniforms.${uniform} =`);
    assert(binding >= 0 && binding < viewer.indexOf('shader.fragmentShader = patchDiffuserShader('));
  }
  for (const [uniform, setting] of [['rgbHighlightKnee', 'highlightKnee'], ['rgbCoverTintStrength', 'coverTintStrength'],
    ['rgbCoverFadeStrength', 'coverFadeStrength'], ['rgbCoverFadePeak', 'coverFadePeak'],
    ['rgbHighlightBoostStart', 'highlightBoostStart'], ['rgbHighlightBoostStrength', 'highlightBoostStrength']]) {
    assert(viewer.includes(`shader.uniforms.${uniform} = { value: DIFFUSER_STYLE.${setting} };`));
  }
  for (const name of ['rgbHighlightGain', 'rgbSoftHighlight', 'rgbLightResponse', 'rgbTintedCover', 'rgbCompositeHighlight']) {
    assert(patched.indexOf(`float ${name}(`) < patched.indexOf('void main()'));
    assert.equal((patched.match(new RegExp(`float ${name}\\(`, 'g')) || []).length, 1);
  }
  assert.equal(DIFFUSER_STYLE.emissionIntensity, 2.5);
  assert.equal(DIFFUSER_STYLE.highlightBoostStart, .35);
  assert.equal(DIFFUSER_STYLE.highlightBoostStrength, 4);
  assert(viewer.includes('material.emissiveIntensity = DIFFUSER_STYLE.emissionIntensity;'));
  assert(viewer.includes("material.customProgramCacheKey = () => 'rgb-opal-cover-v8-spin-contrast';"));
  assert(viewer.includes('THREE.RGBAFormat, THREE.HalfFloatType'));
  assert(reflection > patched.indexOf('#include <emissivemap_fragment>'));
  assert(reflection < patched.indexOf('#include <lights_phong_fragment>'));
  assert(patched.indexOf('#include <specularmap_fragment>') < reflection);
  assert(patched.includes('diffuseColor.rgb *= rgbCoverReflection;'));
  assert(patched.includes('specularStrength *= rgbCoverReflection;'));
  const cached = patched.indexOf('vec3 rgbDisplayEmission = totalEmissiveRadiance;');
  const boosted = patched.indexOf('rgbDisplayEmission *= rgbHighlightGain(emissiveColor.a);');
  const removed = patched.indexOf('totalEmissiveRadiance = vec3(0.0);');
  const toneMapped = patched.indexOf('#include <tonemapping_fragment>');
  const emitted = patched.indexOf('vec3 rgbEmission = max(rgbDisplayEmission, vec3(0.0));');
  const response = patched.indexOf('float rgbResponse = rgbLightResponse(rgbEmissionPeak);');
  const tinted = patched.indexOf('vec3 rgbCover = vec3(');
  const restored = patched.indexOf('vec3 rgbLightSum = rgbCover + rgbEmission;');
  const peak = patched.indexOf('float rgbLightPeak = max(rgbLightSum.r, max(rgbLightSum.g, rgbLightSum.b));');
  const compressed = patched.indexOf('gl_FragColor.rgb = vec3(');
  assert(cached > reflection && boosted > cached && removed > boosted);
  assert(patched.includes('#ifdef USE_EMISSIVEMAP\nrgbDisplayEmission *= rgbHighlightGain(emissiveColor.a);\n#endif'));
  assert(removed < patched.indexOf('#include <lights_phong_fragment>'));
  assert(emitted > toneMapped && response > emitted && tinted > response);
  assert(restored > tinted && peak > restored && compressed > peak);
  assert(restored > toneMapped && restored < patched.indexOf('#include <colorspace_fragment>'));
  assert(compressed > restored && compressed < patched.indexOf('#include <colorspace_fragment>'));
  for (const channel of ['r', 'g', 'b']) {
    assert(patched.includes(`rgbTintedCover(rgbReflection.${channel}, rgbEmission.${channel}, rgbEmissionPeak, rgbResponse)`));
    assert(patched.includes(`rgbCompositeHighlight(rgbLightSum.${channel}, rgbLightPeak, rgbResponse)`));
  }
  assert(patched.indexOf('#include <colorspace_fragment>') < patched.indexOf('#include <fog_fragment>'));
  assert(patched.indexOf('#include <fog_fragment>') < patched.indexOf('#include <premultiplied_alpha_fragment>'));
  assert(patched.indexOf('#include <premultiplied_alpha_fragment>') < patched.indexOf('#include <dithering_fragment>'));
  assert.equal((patched.match(/#include <tonemapping_fragment>/g) || []).length, 1);
  assert.equal((patched.match(/vec3 rgbLightSum =/g) || []).length, 1);
  assert.equal((patched.match(/gl_FragColor\.rgb = vec3\(\s*rgbCompositeHighlight/g) || []).length, 1);
  assert(!/\bclamp\s*\(\s*rgbDisplayEmission/.test(patched), 'Emission intensity must not clip before the soft shoulder');
  assert(!patched.includes('vec3(1.0) - clamp(gl_FragColor.rgb'), 'Cover brightness must not attenuate independent emission');
  assert.equal((patched.match(/emissiveColor\.a/g) || []).length, 1, 'Activity alpha is read only once for the gain');
  assert(!/emissiveColor\.a\s*=/.test(patched));
  assert(!patched.includes('smoothstep('));
  assert(!patched.includes('diffuseColor.a ='));
  assert.equal(patched.slice(patched.indexOf('#include <colorspace_fragment>')),
    original.slice(original.indexOf('#include <colorspace_fragment>')), 'Color conversion, fog and alpha must remain unchanged');
  // Reuse one emission sample: alpha is activity, never material transparency.
  assert(!patched.includes('texture2D('));
  assert.throws(() => patchDiffuserShader('void main() {}'), /shader hook/);
  assert.throws(() => patchDiffuserShader(patched), /shader hook/);
  assert.throws(() => patchDiffuserShader(original.replace(
    '#include <tonemapping_fragment>', '')), /shader hook/);
  assert.throws(() => patchDiffuserShader(original +
    '\n#include <emissivemap_fragment>'), /shader hook/);
});

test('production highlight gain leaves the low range exact and rises C1 monotonically to a bounded fivefold peak', async () => {
  const { gain } = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const start = DIFFUSER_STYLE.highlightBoostStart;
  for (const activity of [-10, 0, 1 / 255, .01, .1, .2, start - 1e-8, start]) {
    assert.equal(gain(activity), 1, `Low activity ${activity} must stay unchanged`);
  }
  const epsilon = 1e-6;
  const leftSlope = (gain(start) - gain(start - epsilon)) / epsilon;
  const rightSlope = (gain(start + epsilon) - gain(start)) / epsilon;
  assert.equal(leftSlope, 0);
  assert(rightSlope >= 0 && Math.abs(rightSlope - leftSlope) < 1e-4, 'The threshold must not create a slope jump');
  assert(gain(start + .01) > 1 && gain(start + .01) < 1.01, 'Gain should enter gradually above the threshold');
  let previous = 1;
  for (let step = 0; step <= 2000; step++) {
    const value = gain(step / 2000);
    assert(Number.isFinite(value) && value >= previous && value >= 1 && value <= 5);
    previous = value;
  }
  assert.equal(gain(1), 5);
  assert.equal(gain(10), 5, 'Activity outside the valid upper range must not amplify past the cap');
});

test('production emission hook keeps PWM 0 through 89 exact and boosts only bright activity without changing alpha', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const cover = [.2, .21, .19];
  for (let pwm = 0; pwm <= 89; pwm++) {
    const pixels = writeDiffuserTexture(frame([0, pwm, pwm / 2]), kernel, new Float32Array(288 * 4));
    const emission = Array.from(pixels.slice(0, 3), value => value * DIFFUSER_STYLE.emissionIntensity);
    const original = emission.slice();
    const boosted = light.emit(emission, pixels[3]);
    assert.equal(light.gain(pixels[3]), 1);
    assert.deepEqual(boosted.emission, emission, `PWM ${pwm} was boosted below the activity threshold`);
    assert.deepEqual(light.compose(cover, boosted.emission), light.compose(cover, emission));
    assert.deepEqual(boosted.removed, [0, 0, 0]);
    assert.equal(boosted.activity, pixels[3]);
    assert.deepEqual(emission, original, 'The cached shader path must not mutate its input');
  }
  for (const pwm of [90, 128, 180, 255]) {
    const pixels = writeDiffuserTexture(frame([0, pwm, pwm / 2]), kernel, new Float32Array(288 * 4));
    const emission = Array.from(pixels.slice(0, 3), value => value * DIFFUSER_STYLE.emissionIntensity);
    const boosted = light.emit(emission, pixels[3]);
    assert(boosted.emission[1] > emission[1], `PWM ${pwm} should enter the bright-region gain`);
    assert.equal(boosted.activity, pixels[3]);
    assert.deepEqual(light.emit(emission, pixels[3], false).emission, emission,
      'A material without an emissive map must not access texture activity or apply its gain');
    const rendered = light.compose(cover, boosted.emission);
    assert(rendered.output.every(value => Number.isFinite(value) && value >= 0 && value <= 1));
    if (pwm === 255) assert.equal(boosted.emission[1], emission[1] * 5);
  }
  assert.deepEqual(light.emit([0, 0, 0], 1).emission, [0, 0, 0], 'Activity cannot create light from zero emission');
});

test('selective gain preserves mixed-color ratios dim spatial background and existing sixty millisecond display damping', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const colors = frame([0, 10, 5]);
  colors[0] = [255, 180, 64];
  const pixels = writeDiffuserTexture(colors, kernel, new Float32Array(288 * 4));
  const original = pixels.slice();
  let dimSamples = 0;
  let brightSamples = 0;
  for (let sample = 0; sample < 288; sample++) {
    const offset = sample * 4;
    const emission = Array.from(pixels.slice(offset, offset + 3), value => value * DIFFUSER_STYLE.emissionIntensity);
    const activity = pixels[offset + 3];
    const boosted = light.emit(emission, activity).emission;
    if (activity <= DIFFUSER_STYLE.highlightBoostStart) {
      assert.deepEqual(boosted, emission, `Dim spatial sample ${sample} changed`);
      dimSamples++;
    } else {
      assert(boosted[0] > emission[0]);
      const dominant = emission.indexOf(Math.max(...emission));
      for (let channel = 0; channel < 3; channel++) {
        assert(Math.abs(boosted[channel] / boosted[dominant] - emission[channel] / emission[dominant]) < 1e-12,
          'The shared gain must not change emitted RGB ratios');
      }
      brightSamples++;
    }
  }
  assert(dimSamples > 0 && brightSamples > 0, 'The rotating head fixture must exercise both untouched and boosted regions');
  assert.deepEqual(pixels, original, 'RGB and original PWM alpha remain unchanged in the texture');
  assert.equal(DIFFUSER_STYLE.smoothingSeconds, .06);
  const displayed = frame([0, 10, 0]);
  const target = frame([0, 255, 128]);
  const supplied = target.map(color => color.slice());
  let previous = 1;
  for (let tick = 0; tick < 40; tick++) {
    advanceRgbDisplayFrame(displayed, target, 1 / 60);
    const current = writeDiffuserTexture(displayed, kernel, new Float32Array(288 * 4));
    const gain = light.gain(current[3]);
    assert(gain >= previous && gain <= 5, 'Gain must follow the already-smoothed display activity');
    previous = gain;
  }
  assert(previous > 4);
  assert.deepEqual(target, supplied);
});

test('production highlight shoulder is linear below its knee, C1 at the knee and monotonic into HDR', async () => {
  const { highlight } = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const knee = DIFFUSER_STYLE.highlightKnee;
  const range = 1 - knee;
  assert(knee >= 0 && knee < 1, 'The highlight shoulder must have a positive finite range');
  assert.equal(highlight(0), 0);
  for (let step = 0; step <= 100; step++) {
    const value = knee * step / 100;
    assert.equal(highlight(value), value, 'The shoulder must leave the dim range unchanged');
  }
  const epsilon = 1e-6;
  const leftSlope = (highlight(knee) - highlight(knee - epsilon)) / epsilon;
  const rightSlope = (highlight(knee + epsilon) - highlight(knee)) / epsilon;
  assert(Math.abs(leftSlope - 1) < 1e-8);
  assert(Math.abs(rightSlope - leftSlope) < 1e-5, 'A slope discontinuity can show as a brightness step');
  assert(Math.abs(highlight(knee + range) - (knee + range * (1 - Math.E ** -1))) < 1e-12);
  let previous = 0;
  for (let step = 0; step <= 4000; step++) {
    const value = step / 500;
    const output = highlight(value);
    assert(Number.isFinite(output) && output >= previous && output <= 1, `Invalid HDR response at ${value}`);
    previous = output;
  }
  // Higher intensities retain distinguishable headroom instead of flattening
  // at the old hard 1.0 emission clamp; finite precision may reach 1 far above it.
  assert(highlight(1) < highlight(1.2) && highlight(1.2) < highlight(2));
  assert(highlight(2) < 1);
  assert.equal(highlight(1000), 1);
});

test('tinted reflection fades dark channels while independent production emission remains additive and monotonic', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  // These are post-ACES reflection values, not the material's linear albedo.
  const cover = [.2, .21, .19];
  let previous = cover[1];
  let previousDark = cover[0];
  for (let pwm = 0; pwm <= 255; pwm++) {
    const pixels = writeDiffuserTexture(frame([0, pwm, 0]), kernel, new Float32Array(288 * 4));
    const emission = Array.from(pixels.slice(0, 3), value => value * DIFFUSER_STYLE.emissionIntensity);
    const rendered = light.compose(cover, emission);
    for (let channel = 0; channel < 3; channel++) {
      assert(Math.abs(rendered.light[channel] - rendered.cover[channel] - emission[channel]) < 1e-12,
        'Tint/fade may change cover reflection, never independent emission');
    }
    assert(rendered.cover[0] <= previousDark && rendered.cover[2] <= cover[2]);
    assert(rendered.output[1] >= previous && rendered.output[1] <= 1, `Composition reversed at PWM ${pwm}`);
    if (pwm > 0) assert(rendered.output[1] > previous, `Composition lost a green step at PWM ${pwm}`);
    previous = rendered.output[1];
    previousDark = rendered.cover[0];
  }
  assert(light.compose(cover, [0, .7 * 3, 0]).output[1] > light.compose(cover, [0, .7 * 2, 0]).output[1],
    'Emission above 1 must reach the shoulder without early clipping');
});

test('production light response has a smooth zero toe and gradual bounded transition', async () => {
  const { response } = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  assert.equal(response(0), 0);
  assert.equal(response(DIFFUSER_STYLE.coverFadePeak), .5);
  let previous = 0;
  for (const peak of [1e-12, 1e-10, 1e-8, 1e-6, .0001, .001, .01, .1, .3, 1, 10, 1000, 1e12]) {
    const value = response(peak);
    assert(Number.isFinite(value) && value > previous && value <= 1, `Invalid cover response at ${peak}`);
    previous = value;
  }
  for (const epsilon of [1e-8, 1e-7, 1e-6]) {
    assert(response(epsilon) / epsilon < 4e-5, 'Cover tint and common-peak response must approach zero with zero slope');
  }
});

test('off preserves the old per-channel result even for unequal HDR reflections and approaches it continuously', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  for (const cover of [[0, 0, 0], [.2, .21, .19], [.05, 1.3, 3.2], [4, .7, 12], [-.2, .8, 1.4]]) {
    const baseline = cover.map(value => light.highlight(Math.max(value, 0)));
    const off = light.compose(cover, [0, 0, 0]);
    assert.deepEqual(off.output, baseline);
    assert.deepEqual(off.cover, cover.map(value => Math.max(value, 0)));
    assert.equal(off.response, 0);
    for (const epsilon of [1e-12, 1e-10, 1e-8, 1e-6, .0001]) {
      const lit = light.compose(cover, [epsilon, epsilon * .4, 0]);
      for (let channel = 0; channel < 3; channel++) {
        assert(Math.abs(lit.output[channel] - baseline[channel]) <= 2 * epsilon,
          `Zero discontinuity for ${cover} at emission ${epsilon}, channel ${channel}`);
      }
    }
  }
});

test('strong green orange and purple gain saturation and preserve color better than the old white-cover shoulder', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const cover = [.2, .21, .19];
  const normalized = rgb => rgb.map(value => value / Math.max(...rgb));
  const distance = (first, second) => first.reduce((sum, value, index) => sum + Math.abs(value - second[index]), 0);
  const saturation = rgb => (Math.max(...rgb) - Math.min(...rgb)) / Math.max(...rgb);
  for (const color of [[0, 255, 0], [255, 180, 0], [180, 0, 255]]) {
    const pixels = writeDiffuserTexture(frame(color), kernel, new Float32Array(288 * 4));
    const emission = Array.from(pixels.slice(0, 3), value => value * DIFFUSER_STYLE.emissionIntensity);
    const rendered = light.compose(cover, emission);
    const previous = cover.map((value, channel) => light.highlight(value + emission[channel]));
    assert(rendered.response > .9, 'Strong light must substantially enter the shared-peak shoulder');
    assert(saturation(rendered.output) > saturation(previous), `Saturation did not improve for ${color}`);
    assert(distance(normalized(rendered.output), normalized(color)) < distance(normalized(previous), normalized(color)),
      `Shared-peak composition did not improve ${color} channel ratios`);
    assert(rendered.output[color.indexOf(0)] < previous[color.indexOf(0)], 'The reflected white floor must fade under colored light');
  }
});

test('production composition remains finite and bounded across HDR and its dominant channel stays monotonic', async () => {
  const light = evaluateProductionLight(patchDiffuserShader(await bundledPhongShader()));
  const colors = [[0, 1, 0], [1, .7, 0], [.7, 0, 1], [1, 1, 1]];
  const levels = [0, 1e-12, 1e-8, .0001, .001, .01, .1, .5, 1, 1.75, 10, 1000, 1e12];
  for (const cover of [[0, 0, 0], [.2, .21, .19], [.7, 1.1, .5], [4, .7, 12]]) {
    for (const color of colors) {
      for (const level of levels) {
        const rendered = light.compose(cover, color.map(value => value * level));
        assert(rendered.output.every(value => Number.isFinite(value) && value >= 0 && value <= 1));
        assert(rendered.cover.every(Number.isFinite) && rendered.light.every(Number.isFinite));
      }
    }
  }
  for (const cover of [[0, 0, 0], [.2, .21, .19], [.7, 1.1, .5]]) {
    for (const color of colors) {
      const dominant = color.indexOf(1);
      let previous = 0;
      for (let step = 0; step <= 2000; step++) {
        const output = light.compose(cover, color.map(value => value * step / 400)).output[dominant];
        assert(output >= previous, `Dominant channel reversed for ${cover}/${color} at step ${step}`);
        previous = output;
      }
    }
  }
});

test('RGB input is atomic, finite, bounded and independent of its caller', () => {
  const supplied = frame([255, 180, 0]);
  const accepted = normalizeRgbFrame(supplied);
  assert(sameRgbFrame(supplied, accepted));
  supplied[0][0] = 0;
  assert.equal(accepted[0][0], 255);
  for (const bad of [null, [], frame([0, 0]), frame([256, 0, 0]),
    frame([true, 0, 0]), frame([NaN, 0, 0]), frame([1.1, 0, 0])]) {
    assert.equal(normalizeRgbFrame(bad), null);
  }
});

test('circular normalized scattering keeps uniform light even and zero truly unlit', () => {
  for (let sample = 0; sample < 288; sample++) {
    const weights = kernel.slice(sample * 24, (sample + 1) * 24);
    assert(Math.abs(weights.reduce((sum, value) => sum + value, 0) - 1) < 1e-6);
  }
  const off = texture(frame([0, 0, 0]));
  assert(off.every(value => value === 0));
  const white = texture(frame([255, 255, 255]));
  // The cap lies at 178.5 byte values; Float32 kernel weights may round either way.
  assert(white.every((value, index) => index % 4 === 3 ? value === 255 : value >= 178 && value <= 179));
  const gray = texture(frame([128, 128, 128]));
  // RGB is bounded display light; alpha carries the original PWM activity.
  assert(gray.every((value, index) => value === (index % 4 === 3 ? 128 : 137)));
});

test('weak light is reduced but remains nonzero, brightness is monotonic and highlights stay bounded', () => {
  let previous = 0;
  for (let level = 0; level <= 255; level++) {
    const pixels = texture(frame([0, level, 0]));
    const displayed = pixels[1];
    assert(displayed >= previous, `Brightness reversed at PWM ${level}`);
    assert(displayed <= 179, 'Highlights must leave room for cover reflection');
    assert.equal(pixels[0], 0);
    assert.equal(pixels[2], 0);
    assert.equal(pixels[3], level, 'Activity must describe input, not lifted display light');
    if (level > 0) assert(displayed > 0);
    previous = displayed;
  }
  const background = texture(frame([0, 10, 0]))[1];
  assert(background > 0 && background < 41 / 2, 'Firmware 4% green background should be darker than its previous byte value 41');
  assert(texture(frame([0, 64, 0]))[1] > background, 'Higher PWM must remain distinguishable from the dim background');
});

test('dim emission falls below the frozen response while bright PWM and its cap remain exactly unchanged', () => {
  // Measured Float32 values from the frozen selective-highlight production module.
  // Fixtures keep this permanent test independent of ignored QA snapshots.
  let previousWeight = DIFFUSER_STYLE.dimLightFloor;
  for (const [pwm, previous] of [[1, .03273744136095047], [2, .05600852891802788],
    [10, .16127800941467285], [38, .32607316970825195]]) {
    const pixels = writeDiffuserTexture(frame([0, pwm, 0]), kernel, new Float32Array(288 * 4));
    assert(pixels[1] > 0 && pixels[1] < previous, `Dim PWM ${pwm} should be reduced without becoming black`);
    const weight = pixels[1] / previous;
    assert(weight >= previousWeight && weight < 1, 'Attenuation must fade out monotonically as PWM rises');
    previousWeight = weight;
    assert.equal(pixels[0], 0);
    assert.equal(pixels[2], 0);
    assert(Math.abs(pixels[3] - pwm / 255) < 1e-7, 'Activity must retain the original PWM');
  }
  for (const [pwm, previous] of [[90, .4711666405200958], [128, .5365010499954224],
    [180, .6061309576034546], [255, .699999988079071]]) {
    const pixels = writeDiffuserTexture(frame([0, pwm, pwm / 2]), kernel, new Float32Array(288 * 4));
    assert.equal(pixels[1], previous, `Bright PWM ${pwm} changed from the frozen response`);
    assert.equal(pixels[2], Math.fround(previous / 2), 'One common dim weight must preserve the RGB ratio');
    assert(Math.abs(pixels[3] - pwm / 255) < 1e-7);
  }
  const full = writeDiffuserTexture(frame([255, 255, 255]), kernel, new Float32Array(288 * 4));
  assert(Math.abs(full[0] - DIFFUSER_STYLE.emissionLimit) < 1e-6);
});

test('production dim weight has a bounded monotonic C1 transition through its half and end thresholds', () => {
  assert.equal(DIFFUSER_STYLE.dimLightFloor, .25);
  assert.equal(DIFFUSER_STYLE.dimLightEnd, .35);
  const floor = DIFFUSER_STYLE.dimLightFloor;
  const end = DIFFUSER_STYLE.dimLightEnd;
  assert.equal(dimLightWeight(-1), floor);
  assert.equal(dimLightWeight(0), floor);
  assert.equal(dimLightWeight(end / 2), .625);
  for (const peak of [end, end + 1e-8, .5, 1, 100]) assert.equal(dimLightWeight(peak), 1);
  const epsilon = 1e-6;
  const toeLeftSlope = (dimLightWeight(0) - dimLightWeight(-epsilon)) / epsilon;
  const toeRightSlope = (dimLightWeight(epsilon) - dimLightWeight(0)) / epsilon;
  const endLeftSlope = (dimLightWeight(end) - dimLightWeight(end - epsilon)) / epsilon;
  const endRightSlope = (dimLightWeight(end + epsilon) - dimLightWeight(end)) / epsilon;
  assert.equal(toeLeftSlope, 0);
  assert.equal(endRightSlope, 0);
  assert(Math.abs(toeRightSlope - toeLeftSlope) < 1e-4, 'The dim toe must not create a slope jump');
  assert(Math.abs(endRightSlope - endLeftSlope) < 1e-4, 'The unchanged bright range must join with zero slope');
  let previous = floor;
  for (let step = 0; step <= 1000; step++) {
    const weight = dimLightWeight(step / 1000);
    assert(Number.isFinite(weight) && weight >= previous && weight >= floor && weight <= 1);
    previous = weight;
  }
});

test('float display texture retains fractional low PWM and zero has a finite smooth toe', () => {
  const values = [0, 0.0001, 0.001, 0.01, 0.1, 0.5, 1, 2, 3, 4, 5];
  let previous = 0;
  for (const level of values) {
    const pixels = writeDiffuserTexture(frame([0, level, level / 2]), kernel, new Float32Array(288 * 4));
    assert(Number.isFinite(pixels[1]) && pixels[1] >= previous);
    assert(pixels[1] <= 0.71);
    assert(Math.abs(pixels[2] - pixels[1] / 2) < 1e-7);
    assert(Math.abs(pixels[3] - level / 255) < 1e-7);
    if (level > 0) {
      assert(pixels[1] > previous, 'Fractional PWM must not quantize back to an 8-bit step');
      assert(pixels[1] / (level / 255) < 12, 'Near zero the response must have bounded gain');
    } else assert(pixels.every(value => value === 0));
    previous = pixels[1];
  }
});

test('one shared response preserves RGB ratios rather than whitening weak colors', () => {
  for (const color of [[255, 128, 64], [64, 32, 16], [0, 10, 5], [255, 180, 0]]) {
    const pixels = texture(frame(color));
    const dominant = color.indexOf(Math.max(...color));
    for (let component = 0; component < 3; component++) {
      assert(Math.abs(pixels[component] - pixels[dominant] * color[component] / color[dominant]) <= 1);
    }
  }
});

test('rotating green retains its same-color background with a brighter distinct head', () => {
  const colors = frame([0, 10, 0]);
  colors[0] = [0, 255, 0];
  colors[23] = [0, 180, 0];
  const pixels = texture(colors);
  const greens = Array.from({ length: 288 }, (_, sample) => pixels[sample * 4 + 1]);
  const darkest = Math.min(...greens);
  const brightest = Math.max(...greens);
  assert(darkest > 0 && darkest < 41 / 2, 'Dim ring must remain green but darker than the frozen background');
  assert.equal(brightest, 155, 'The frozen bright moving head must remain unchanged');
  const floats = writeDiffuserTexture(colors, kernel, new Float32Array(288 * 4));
  const floatingGreens = Array.from({ length: 288 }, (_, sample) => floats[sample * 4 + 1]);
  const dim = Math.min(...floatingGreens);
  const head = Math.max(...floatingGreens);
  assert(dim > 0 && dim < .16127800941467285 / 2);
  assert.equal(head, .6085164546966553);
  assert(head / dim > 3.7730900629611397 * 3, 'Head/background contrast should improve substantially from the frozen ratio');
  assert(pixels.every((value, index) => index % 4 === 1 || index % 4 === 3 || value === 0));
});

test('one source scatters smoothly across the 23/0 circular seam', () => {
  const colors = frame([0, 0, 0]);
  colors[0] = [255, 0, 0];
  const pixels = texture(colors);
  assert(pixels[0] > pixels[12 * 4]);
  assert(pixels[12 * 4] > pixels[24 * 4]);
  assert(pixels[24 * 4] > 0);
  assert.equal(pixels[12 * 4], pixels[(288 - 12) * 4]);
  assert.equal(pixels[24 * 4], pixels[(288 - 24) * 4]);
  assert.equal(pixels[144 * 4], 0);
  assert(pixels.every((value, index) => index % 4 === 0 || index % 4 === 3 || value === 0));
});

test('source order rotates the spatial texture without changing emitted power', () => {
  const first = frame([0, 0, 0]);
  first[0] = [20, 210, 80];
  const second = frame([0, 0, 0]);
  second[6] = first[0];
  const before = texture(first);
  const after = texture(second);
  for (let sample = 0; sample < 288; sample++) {
    assert.deepEqual(after.slice(((sample + 72) % 288) * 4, ((sample + 72) % 288 + 1) * 4),
      before.slice(sample * 4, (sample + 1) * 4));
  }
});

test('ring UV uses verified local axes and closes the seam per triangle', () => {
  const center = [0, 0, 0.18996717780828476];
  const angles = [-0.02, 0.01, 0.03];
  const positions = angles.flatMap((angle, index) =>
    [0.04 * Math.sin(angle), 0.04 * Math.cos(angle), center[2] + index * 0.003]);
  const uv = createAngularUv(positions, center, [0, 1, 0], [1, 0, 0]);
  const u = [uv[0], uv[2], uv[4]];
  assert(Math.max(...u) - Math.min(...u) < 0.01);
  assert(u[1] > u[0] && u[2] > u[1]);
  assert.deepEqual([uv[1], uv[3], uv[5]], [0.5, 0.5, 0.5]);
  // Translate the entire base/rail; the local mapping has no world dependency.
  const offset = [1, -0.25, 0.5];
  const moved = positions.map((value, index) => value + offset[index % 3]);
  const translated = createAngularUv(moved, center.map((value, index) => value + offset[index]),
    [0, 1, 0], [1, 0, 0]);
  assert.deepEqual(translated, uv);
});

test('invalid texture/triangle dimensions are rejected', () => {
  assert.throws(() => createDiffuserKernel(10), RangeError);
  assert.throws(() => writeDiffuserTexture(frame([0, 0, 0]), kernel, new Uint8Array(3)), RangeError);
  assert.throws(() => createAngularUv([1, 2, 3], [0, 0, 0], [1, 0, 0], [0, 1, 0]), RangeError);
});

test('registered production STL has finite seam-safe UVs throughout the diffuser', async () => {
  const bytes = await readFile(new URL('../resources/models/DummyRobot_URDF/meshes/light.stl', import.meta.url));
  const faces = bytes.readUInt32LE(80);
  assert.equal(bytes.length, 84 + faces * 50);
  const positions = new Float32Array(faces * 9);
  for (let face = 0; face < faces; face++) {
    for (let component = 0; component < 9; component++) {
      positions[face * 9 + component] = bytes.readFloatLE(84 + face * 50 + 12 + component * 4);
    }
  }
  const uv = createAngularUv(positions, [0, 0, 0.18996717780828476], [-1, 0, 0], [0, 1, 0]);
  assert(uv.every(Number.isFinite));
  for (let face = 0; face < faces; face++) {
    const angles = [uv[face * 6], uv[face * 6 + 2], uv[face * 6 + 4]];
    assert(Math.max(...angles) - Math.min(...angles) <= 0.5, `Stretched UV at triangle ${face}`);
  }
  const radii = [];
  for (let vertex = 0; vertex < positions.length / 3; vertex++) {
    radii.push(Math.hypot(positions[vertex * 3], positions[vertex * 3 + 1]));
  }
  assert(Math.min(...radii) > 0.034 && Math.max(...radii) < 0.045);
});
