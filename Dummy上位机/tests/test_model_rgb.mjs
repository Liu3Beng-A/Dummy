import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

// This production module has no Three/DOM dependencies. Loading it by a data
// URL also works when the Python application's directory has no package.json.
const source = await readFile(new URL('../resources/viewer/model_rgb.js', import.meta.url), 'utf8');
const { advanceRgbDisplayFrame, createAngularUv, createDiffuserKernel, normalizeRgbFrame, patchDiffuserShader, sameRgbFrame,
  writeDiffuserTexture } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
const frame = color => Array.from({ length: 24 }, () => color.slice());
const kernel = createDiffuserKernel();
const texture = colors => writeDiffuserTexture(colors, kernel, new Uint8Array(288 * 4));

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

test('opal shader separates bounded emission from ACES and leaves opacity/output conversion intact', async () => {
  const threeSource = await readFile(new URL('../resources/viewer/vendor/three/three.module.js', import.meta.url), 'utf8');
  const encodedPhong = threeSource.match(/const fragment\$6 = ("(?:[^"\\]|\\.)*");/);
  assert(encodedPhong, 'Expected the bundled MeshPhong fragment shader');
  const patched = patchDiffuserShader(JSON.parse(encodedPhong[1]));
  const reflection = patched.indexOf('diffuseColor.rgb *= rgbCoverReflection;');
  assert(patched.includes('uniform float rgbCoverReflection;'));
  assert(reflection > patched.indexOf('#include <emissivemap_fragment>'));
  assert(reflection < patched.indexOf('#include <lights_phong_fragment>'));
  assert(patched.indexOf('#include <specularmap_fragment>') < reflection);
  assert(patched.includes('diffuseColor.rgb *= rgbCoverReflection;'));
  assert(patched.includes('specularStrength *= rgbCoverReflection;'));
  const cached = patched.indexOf('vec3 rgbDisplayEmission = totalEmissiveRadiance;');
  const removed = patched.indexOf('totalEmissiveRadiance = vec3(0.0);');
  const toneMapped = patched.indexOf('#include <tonemapping_fragment>');
  const restored = patched.indexOf('gl_FragColor.rgb += rgbDisplayEmission;');
  assert(cached > reflection && removed > cached);
  assert(removed < patched.indexOf('#include <lights_phong_fragment>'));
  assert(restored > toneMapped && restored < patched.indexOf('#include <colorspace_fragment>'));
  assert(patched.indexOf('#include <colorspace_fragment>') < patched.indexOf('#include <fog_fragment>'));
  assert.equal((patched.match(/#include <tonemapping_fragment>/g) || []).length, 1);
  assert.equal((patched.match(/gl_FragColor\.rgb \+= rgbDisplayEmission;/g) || []).length, 1);
  assert(!patched.includes('emissiveColor.a'));
  assert(!patched.includes('smoothstep('));
  assert(!patched.includes('diffuseColor.a ='));
  // Reuse one emission sample: alpha is activity, never material transparency.
  assert(!patched.includes('texture2D('));
  assert.throws(() => patchDiffuserShader('void main() {}'), /shader hook/);
  assert.throws(() => patchDiffuserShader(patched), /shader hook/);
  assert.throws(() => patchDiffuserShader(JSON.parse(encodedPhong[1]).replace(
    '#include <tonemapping_fragment>', '')), /shader hook/);
  assert.throws(() => patchDiffuserShader(JSON.parse(encodedPhong[1]) +
    '\n#include <emissivemap_fragment>'), /shader hook/);
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
  assert(gray.every((value, index) => value === (index % 4 === 3 ? 128 : 127)));
});

test('weak light remains visible, brightness is monotonic and highlights stay bounded', () => {
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
  assert(texture(frame([0, 10, 0]))[1] >= 30, 'Firmware 4% green background must be visible');
  assert(texture(frame([0, 64, 0]))[1] >= 85, 'READY green must not disappear under the cover');
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
      assert(pixels[1] / (level / 255) < 10, 'Near zero the response must have bounded gain');
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
  assert(darkest >= 30, 'Dim ring must remain green all around');
  assert(brightest >= darkest * 2, 'Moving head must remain distinguishable');
  assert(brightest <= darkest * 5, 'The response must compress excessive head/background contrast');
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
  const uv = createAngularUv(positions, [0, 0, 0.18996717780828476], [0, 1, 0], [1, 0, 0]);
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
