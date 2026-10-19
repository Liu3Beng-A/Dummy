import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
const source = await readFile(new URL('../resources/viewer/performance.js', import.meta.url), 'utf8');
const { FramePerformance } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));

test('FPS measures completed frames at different rates rather than telemetry or a fixed 60', () => {
  for (const fps of [15, 30, 60, 120]) {
    const stats = new FramePerformance();
    for (let i = 0; i <= fps * 3; i++) stats.record(i * 1000 / fps);
    const result = stats.snapshot(3000);
    assert.ok(Math.abs(result.fps - fps) < .001);
    assert.ok(Math.abs(result.frame_ms_p95 - 1000 / fps) < .001);
    assert.ok(result.rendered_frames <= fps * 2 + 1);
  }
});

test('render stalls reduce measured FPS and stale/hidden history is not reused', () => {
  const stats = new FramePerformance();
  for (let i = 0; i <= 60; i++) stats.record(i * 1000 / 60);
  assert.ok(stats.snapshot(1500).fps < 41);
  assert.equal(stats.snapshot(4000).fps, null);
  stats.reset();
  stats.record(10000);
  assert.equal(stats.snapshot(10000).fps, null);
  stats.record(10050);
  assert.equal(stats.snapshot(10050).fps, 20);
});

test('duplicate invalid or reversed frame timestamps cannot inflate FPS', () => {
  const stats = new FramePerformance();
  for (const value of [0, NaN, Infinity, -1, 0, 20, 20, 10, 40]) stats.record(value);
  assert.equal(stats.snapshot(40).fps, 50);
  assert.equal(stats.snapshot(40).rendered_frames, 3);
});
