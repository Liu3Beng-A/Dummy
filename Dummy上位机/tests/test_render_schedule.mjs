import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
const source = await readFile(new URL('../resources/viewer/render_schedule.js', import.meta.url), 'utf8');
const { RenderSchedule } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));

function clock(draw) {
  let now = 0, next = 0;
  const events = new Map();
  const schedule = (callback, delay) => {
    const id = ++next;
    events.set(id, { at: now + delay, callback });
    return id;
  };
  const loop = new RenderSchedule(draw, {
    now: () => now,
    frame: callback => schedule(() => callback(now), 1000 / 60),
    cancelFrame: id => events.delete(id),
    timer: schedule, cancelTimer: id => events.delete(id),
  });
  return { loop, advance(ms) {
    const end = now + ms;
    while (events.size) {
      const [id, event] = [...events].sort((a, b) => a[1].at - b[1].at)[0];
      if (event.at > end) break;
      now = event.at;
      events.delete(id);
      event.callback();
    }
    now = end;
  } };
}

test('bursts coalesce into one draw and static scenes stop requesting frames', () => {
  let latest = 0;
  const displayed = [];
  const { loop, advance } = clock(() => { displayed.push(latest); return false; });
  for (let i = 1; i <= 100; i++) { latest = i; loop.request(); }
  advance(1000);
  assert.deepEqual(displayed, [100]);
  assert.equal(loop.diagnostics().idle, true);
  latest = 101; loop.request(); advance(20);
  assert.deepEqual(displayed, [100, 101]);
});

test('active animation continues, background drawing is capped and hidden drawing stops', () => {
  const { loop, advance } = clock(() => true);
  loop.request(); advance(1000);
  assert(loop.frames >= 59);
  loop.setMode('background');
  const before = loop.frames; advance(1000);
  assert(loop.frames - before >= 10 && loop.frames - before <= 15);
  loop.setMode('hidden');
  const hidden = loop.frames;
  for (let i = 0; i < 100; i++) loop.request();
  advance(3000);
  assert.equal(loop.frames, hidden);
  loop.setMode('active'); advance(17);
  assert.equal(loop.frames, hidden + 1);
});

test('wakeups inside a draw remain bounded and disposal cancels all work', () => {
  let count = 0;
  const { loop, advance } = clock(() => {
    if (++count < 3) loop.request();
    return false;
  });
  loop.request(); advance(1000);
  assert.equal(count, 3);
  assert(loop.diagnostics().idle);
  loop.setMode('background'); loop.request();
  loop.dispose(); advance(3000);
  assert.equal(count, 3);
});
