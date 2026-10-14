// Shared-frame rendering only: animation and serial polling remain in Python.
export const RGB_CHANNELS = 24;
export const DIFFUSER_SAMPLES = 288;
const TAU = 2 * Math.PI;
const DIFFUSER_TOE = 0.003;
const DIFFUSER_EXPONENT = 0.45;
const DIFFUSER_CURVE_BASE = DIFFUSER_TOE ** DIFFUSER_EXPONENT;
const DIFFUSER_CURVE_RANGE = (1 + DIFFUSER_TOE) ** DIFFUSER_EXPONENT - DIFFUSER_CURVE_BASE;

export function normalizeRgbFrame(frame) {
  if (!Array.isArray(frame) || frame.length !== RGB_CHANNELS) return null;
  for (const color of frame) {
    if (!Array.isArray(color) || color.length !== 3) return null;
    for (const value of color) {
      if (!Number.isInteger(value) || value < 0 || value > 255) return null;
    }
  }
  return frame.map(color => color.slice());
}

export function sameRgbFrame(first, second) {
  return first && second && first.every((color, index) =>
    color.every((value, channel) => value === second[index][channel]));
}

// Display-only interpolation in PWM units. The integer target frame is never
// mutated, and message arrival intervals do not affect interpolation continuity.
export function advanceRgbDisplayFrame(displayed, target, dt, seconds = 0.06) {
  if (!Number.isFinite(dt) || dt < 0 || !Number.isFinite(seconds) || seconds <= 0) {
    throw new RangeError('RGB display damping requires a finite nonnegative time step.');
  }
  const factor = dt > 0.25 ? 1 : 1 - Math.exp(-dt / seconds);
  let changed = false;
  for (let source = 0; source < RGB_CHANNELS; source++) {
    for (let channel = 0; channel < 3; channel++) {
      const previous = displayed[source][channel];
      const goal = target[source][channel];
      const damped = previous + (goal - previous) * factor;
      const next = Math.abs(goal - damped) <= 0.002 ? goal : damped;
      if (next !== previous) changed = true;
      displayed[source][channel] = next;
    }
  }
  return changed;
}

export function patchDiffuserShader(fragmentShader) {
  const emissionHook = '#include <emissivemap_fragment>';
  const toneMapHook = '#include <tonemapping_fragment>';
  const uniformHook = '#include <common>';
  const hooks = [uniformHook, emissionHook, toneMapHook];
  if (!fragmentShader.includes('#define PHONG')
    || hooks.some(hook => fragmentShader.split(hook).length !== 2)
    || !/vec3\s+outgoingLight\s*=\s*[^;]*\+\s*totalEmissiveRadiance\s*;/.test(fragmentShader)
    || fragmentShader.includes('rgbCoverReflection')) {
    throw new Error('The diffuser requires unmodified Phong emission-map and tone-map shader hooks.');
  }
  // A metadata-driven uniform describes the entire powered ring. Breathing or
  // a dark individual arc never toggles its white cover's reflection/opacity.
  const withCover = fragmentShader.replace(uniformHook, `${uniformHook}
uniform float rgbCoverReflection;`).replace(emissionHook, `${emissionHook}
diffuseColor.rgb *= rgbCoverReflection;
specularStrength *= rgbCoverReflection;
vec3 rgbDisplayEmission = totalEmissiveRadiance;
totalEmissiveRadiance = vec3(0.0);`);
  // The vendor Phong shader adds totalEmissiveRadiance to outgoingLight before
  // opaque_fragment. Cache emission and zero that contribution before lighting,
  // tone-map reflection alone, then restore bounded emission once. Avoiding a
  // subtraction also preserves small reflected values. Color/fog/alpha keep
  // their original order; other robot materials keep the global ACES pipeline.
  return withCover.replace(toneMapHook, `${toneMapHook}
gl_FragColor.rgb += rgbDisplayEmission;`);
}

export function createDiffuserKernel(samples = DIFFUSER_SAMPLES) {
  if (!Number.isInteger(samples) || samples < RGB_CHANNELS) {
    throw new RangeError('The diffuser needs at least 24 angular samples.');
  }
  // Match the existing 2D opal cover's circular Gaussian radius (0.8 sources).
  const kernel = new Float32Array(samples * RGB_CHANNELS);
  for (let sample = 0; sample < samples; sample++) {
    const position = sample * RGB_CHANNELS / samples;
    let total = 0;
    for (let channel = 0; channel < RGB_CHANNELS; channel++) {
      const distance = (channel - position + 36) % RGB_CHANNELS - 12;
      const weight = Math.exp(-0.5 * (distance / 0.8) ** 2);
      kernel[sample * RGB_CHANNELS + channel] = weight;
      total += weight;
    }
    for (let channel = 0; channel < RGB_CHANNELS; channel++) {
      kernel[sample * RGB_CHANNELS + channel] /= total;
    }
  }
  return kernel;
}

export function writeDiffuserTexture(frame, kernel, pixels) {
  const samples = kernel.length / RGB_CHANNELS;
  if (!Number.isInteger(samples) || pixels.length !== samples * 4) {
    throw new RangeError('Diffuser texture and kernel dimensions differ.');
  }
  const floating = pixels instanceof Float32Array;
  if (!floating && !(pixels instanceof Uint8Array) && !(pixels instanceof Uint8ClampedArray)) {
    throw new TypeError('Diffuser output must be Float32Array or an 8-bit pixel array.');
  }
  // Firmware RGB bytes are PWM levels. Scatter those levels before applying
  // one shared peak curve, rather than gamma-transforming separate channels.
  const power = frame.map(color => color.map(value => value / 255));
  for (let sample = 0; sample < samples; sample++) {
    const offset = sample * RGB_CHANNELS;
    const scattered = [0, 0, 0];
    let weightTotal = 0;
    for (let channel = 0; channel < RGB_CHANNELS; channel++) {
      weightTotal += kernel[offset + channel];
    }
    for (let component = 0; component < 3; component++) {
      for (let channel = 0; channel < RGB_CHANNELS; channel++) {
        scattered[component] += power[channel][component] * kernel[offset + channel];
      }
      // Correct Float32 kernel round-off before the curve/byte quantization so
      // uniform PWM input stays uniform, including the 0.7*255 half-byte edge.
      scattered[component] /= weightTotal;
    }
    const peak = Math.min(1, Math.max(0, ...scattered));
    // A finite-slope toe keeps small PWM changes smooth instead of applying an
    // infinite-slope power law near zero. One peak scale preserves RGB ratios.
    const toePeak = peak > 0 ? 0.7 * ((peak + DIFFUSER_TOE) ** DIFFUSER_EXPONENT
      - DIFFUSER_CURVE_BASE) / DIFFUSER_CURVE_RANGE : 0;
    // Lift dim colors by up to 30%, smoothly removing that gain at full power.
    // One common peak gain retains channel ratios and the existing 0.7 ceiling.
    const displayPeak = toePeak * (1 + 0.3 * (1 - peak) ** 2);
    const scale = peak > 0 ? displayPeak / peak : 0;
    for (let component = 0; component < 3; component++) {
      const light = Math.min(0.7, Math.max(0, scattered[component] * scale));
      pixels[sample * 4 + component] = floating ? light : Math.round(light * 255);
    }
    // Activity is diagnostic data only; the shader never turns it into opacity.
    pixels[sample * 4 + 3] = floating ? peak : Math.round(peak * 255);
  }
  return pixels;
}

// STL triangles have no UVs. Map the angle around the verified local ring axis;
// no world coordinates are used, so the mapping travels with the rail/base.
export function createAngularUv(positions, center, axisU, axisV) {
  if (positions.length % 9) throw new RangeError('Expected unindexed STL triangles.');
  const uv = new Float32Array(positions.length / 3 * 2);
  for (let vertex = 0; vertex < positions.length / 3; vertex++) {
    let x = 0;
    let y = 0;
    for (let component = 0; component < 3; component++) {
      const relative = positions[vertex * 3 + component] - center[component];
      x += relative * axisU[component];
      y += relative * axisV[component];
    }
    uv[vertex * 2] = (Math.atan2(y, x) / TAU + 1) % 1;
    uv[vertex * 2 + 1] = 0.5;
  }
  // Unwrap each seam-crossing triangle. RepeatWrapping joins source 23 to 0
  // without stretching the whole texture across one narrow STL triangle.
  for (let vertex = 0; vertex < positions.length / 3; vertex += 3) {
    const offsets = [vertex * 2, (vertex + 1) * 2, (vertex + 2) * 2];
    const angles = offsets.map(offset => uv[offset]);
    if (Math.max(...angles) - Math.min(...angles) > 0.5) {
      offsets.forEach(offset => { if (uv[offset] < 0.5) uv[offset] += 1; });
    }
  }
  return uv;
}
