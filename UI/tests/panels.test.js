'use strict';

// Regression tests for the STATUS instruments + header layout on a phone.
//
// 1. drawAttitude: the pitch ladder rungs/labels must never touch the fixed
//    roll-arc ticks or the bezel, at ANY roll/pitch combination. The original
//    bug was a +40 label landing on the 45deg tick.
// 2. renderMotorBars: firmware sends motor_left / motor_right; the bars and
//    the numbers must follow them (both signs, and the center-zero fill).

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const panelsSource = fs.readFileSync(path.join(__dirname, '..', 'js', 'panels.js'), 'utf8');

function createHarness() {
  // Records every canvas draw call so collisions can be measured geometrically.
  const ops = [];
  const ctx = new Proxy({}, {
    get(_, prop) {
      if (prop === 'canvas') return { width: 180, height: 180 };
      if (prop === 'measureText') return () => ({ width: 14 });
      if (prop === 'save' || prop === 'restore' || prop === 'beginPath' ||
          prop === 'closePath' || prop === 'clip' || prop === 'fill' ||
          prop === 'stroke' || prop === 'fillRect' || prop === 'fillText' ||
          prop === 'arc' || prop === 'moveTo' || prop === 'lineTo' ||
          prop === 'translate' || prop === 'rotate' || prop === 'clearRect') {
        return (...args) => { ops.push([prop, ...args]); };
      }
      // properties (fillStyle, strokeStyle, font, lineWidth, ...)
      return undefined;
    },
    set() { return true; },
  });

  const canvas = { width: 180, height: 180, getContext: () => ctx };
  const elements = {
    attitudeCanvas: canvas,
    compassCanvas: { width: 180, height: 180, getContext: () => ctx },
  };
  // Any other element panels.js touches at load time gets a harmless stub so
  // the module can be evaluated outside a browser.
  const stub = () => ({
    style: {}, dataset: {}, classList: { add() {}, remove() {}, toggle() {} },
    addEventListener() {}, appendChild() {}, querySelector: () => null,
    querySelectorAll: () => [], value: '', textContent: '', checked: false,
    getBoundingClientRect: () => ({ x: 0, y: 0, width: 100, height: 20 }),
  });

  const sandbox = {
    console: { log() {}, warn() {}, error() {} },
    Math, Date, JSON, String, Number, Array, Object, Boolean,
    isFinite, parseFloat, parseInt,
    document: {
      getElementById: (id) => elements[id] || stub(),
      querySelector: () => stub(),
      querySelectorAll: () => [],
      addEventListener() {},
    },
    window: { addEventListener() {} },
    requestAnimationFrame() {},
    setTimeout, clearTimeout, setInterval, clearInterval,
    isConnected: true,
    state: { roll: 0, pitch: 0, yaw: 0, throttle: 0, armed: false },
    updateDisplay() {}, drawStick() {}, renderArm() {}, syncOfflineOverlay() {},
    applyVehicleUI() {},
    espIP: '192.168.100.27',
  };
  vm.createContext(sandbox);
  vm.runInContext(panelsSource, sandbox, { filename: 'UI/js/panels.js' });
  return { sandbox, elements, ops };
}

// Segments are collected alongside items so a moveTo/lineTo pair is one edge.
const ALL_SEGS = [];
function segsPush(items, from, to) {
  ALL_SEGS.push({ x1: from.x, y1: from.y, x2: to.x, y2: to.y });
}

// Replays the recorded ops with a real 2D-transform stack.
function replay(ops) {
  ALL_SEGS.length = 0;
  const SIZE = 180;
  const items = [];
  // Canvas transforms are MUTATING: translate/rotate compose onto the current
  // matrix, while save/restore snapshot it. Pushing a frame per translate
  // (an easy mistake) makes one restore() undo only the rotation and silently
  // leaves every later coordinate translated — which would make the whole
  // collision test meaningless, so the model has to be right here.
  let stack = [{ a: 1, b: 0, c: 0, d: 1, e: 0, f: 0 }];
  // Canvas CTM row form is [a c e; b d f], and a new operation POST-multiplies:
  // combined = current x op. Written out term by term — the compact 2x2
  // shortcut transposes b/c here and silently sends rotated coordinates to
  // infinity, which would make the whole collision test vacuous.
  const mul = (m, n) => ({
    a: m.a * n.a + m.c * n.b,
    b: m.b * n.a + m.d * n.b,
    c: m.a * n.c + m.c * n.d,
    d: m.b * n.c + m.d * n.d,
    e: m.a * n.e + m.c * n.f + m.e,
    f: m.b * n.e + m.d * n.f + m.f,
  });
  const apply = (m, x, y) => ({ x: m.a * x + m.c * y + m.e, y: m.b * x + m.d * y + m.f });
  const cur = () => stack[stack.length - 1];
  const set = (m) => { stack[stack.length - 1] = m; };
  let pen = null;
  for (const [op, ...a] of ops) {
    if (op === 'save') stack.push({ ...cur() });
    else if (op === 'restore') { if (stack.length > 1) stack.pop(); }
    else if (op === 'translate') set(mul(cur(), { a: 1, b: 0, c: 0, d: 1, e: a[0], f: a[1] }));
    else if (op === 'rotate') {
      const cs = Math.cos(a[0]), sn = Math.sin(a[0]);
      set(mul(cur(), { a: cs, b: sn, c: -sn, d: cs, e: 0, f: 0 }));
    } else if (op === 'moveTo') {
      pen = apply(cur(), a[0], a[1]);
      items.push({ kind: 'pt', op, ...pen });
    } else if (op === 'lineTo') {
      const p = apply(cur(), a[0], a[1]);
      items.push({ kind: 'pt', op, ...p });
      if (pen) {
        segsPush(items, pen, p);
        pen = null;
      }
    } else if (op === 'fillText') {
      // fillText(text, x, y) — the text is arg 0, so the coordinates are
      // args 1 and 2. Reading a[0]/a[1] here puts the label at (text, x),
      // which is nowhere near the disc and makes every collision check pass
      // for the wrong reason.
      const p = apply(cur(), a[1], a[2]);
      items.push({ kind: 'label', text: a[0], x: p.x, y: p.y, w: a[0].length * 5.4, h: 9 });
    } else if (op === 'arc') {
      items.push({ kind: 'arc', cx: a[0], cy: a[1], r: a[2] });
    }
  }
  return { items, segs: ALL_SEGS.slice(), SIZE };
}

function segIntersects(a, b, c, d) {
  const s = (x1, y1, x2, y2, x3, y3, x4, y4) => {
    const d1 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    const d2 = (x4 - x3) * (y2 - y3) - (y4 - y3) * (x2 - x3);
    const d3 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x1 - x1);
    const d4 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x3 - x1);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  };
  return s(a.x1, a.y1, a.x2, a.y2, c.x1, c.y1, c.x2, c.y2);
}

function boxHit(b, c, d) {
  // b = {x1,y1,x2,y2} axis-aligned; c/d = segment endpoints
  return segIntersects(
    { x1: b.x1, y1: b.y1, x2: b.x2, y2: b.y1 }, c, d
  ) || segIntersects(
    { x1: b.x1, y1: b.y2, x2: b.x2, y2: b.y2 }, c, d
  ) || segIntersects(
    { x1: b.x1, y1: b.y1, x2: b.x1, y2: b.y2 }, c, d
  ) || segIntersects(
    { x1: b.x2, y1: b.y1, x2: b.x2, y2: b.y2 }, c, d
  );
}

function labelBox(it) {
  return { x1: it.x, y1: it.y - it.h / 2, x2: it.x + it.w, y2: it.y + it.h / 2 };
}

function analyseAttitude(roll, pitch) {
  const h = createHarness();
  h.ops.length = 0;
  h.sandbox.drawAttitude(roll, pitch);
  const { items, segs } = replay(h.ops);
  const size = 180, center = size / 2, radius = size / 2 - 4;

  // The roll-arc ticks are the ONLY segments drawn after the ladder's
  // restore(), i.e. the trailing run of short radial strokes nearest the rim.
  // Classify by radius band so the test does not depend on ordering.
  const RIM = radius - 14; // ticks live at radius-3 .. radius-12
  const ticks = [], ladder = [];
  for (const s of segs) {
    const m = Math.hypot((s.x1 + s.x2) / 2 - center, (s.y1 + s.y2) / 2 - center);
    (m > RIM ? ticks : ladder).push(s);
  }
  const labels = items.filter(i => i.kind === 'label').map(i => ({ text: i.text, box: labelBox(i) }));
  return { segs, ladder, ticks, labels, center, radius };
}

test('the geometry model itself is correct (guards the tests below)', () => {
  // At zero roll and zero pitch the ladder must be symmetric about the disc
  // centre. If the replay transform is wrong (e.g. a transposed matrix) this
  // fails while every collision test still reports "no collision".
  const a = analyseAttitude(0, 0);
  const byText = Object.fromEntries(a.labels.map(l => [l.text, l.box]));
  assert.ok(byText['+10'] && byText['-10'] && byText['+20'] && byText['-20'],
    `expected the level ladder to label +/-10 and +/-20, got ${JSON.stringify(Object.keys(byText))}`);
  for (const [p, m] of [['+10', '-10'], ['+20', '-20'], ['+30', '-30']]) {
    const xp = (byText[p].x1 + byText[p].x2) / 2, xm = (byText[m].x1 + byText[m].x2) / 2;
    const yp = (byText[p].y1 + byText[p].y2) / 2, ym = (byText[m].y1 + byText[m].y2) / 2;
    // Labels are left-aligned right of each rung, so a +/- pair shares the
    // same x and straddles the centre vertically: y-midpoint is 90.
    assert.ok(Math.abs(xp - xm) < 0.5, `${p}/${m} share the same x, got ${xp.toFixed(2)} vs ${xm.toFixed(2)}`);
    assert.ok(Math.abs((yp + ym) / 2 - 90) < 0.5,
      `${p}/${m} y-midpoint should be the disc centre 90, got ${((yp + ym) / 2).toFixed(2)}`);
  }
  // And a positive roll must rotate the ladder, not translate it.
  const rolled = analyseAttitude(45, 0);
  const lvl = analyseAttitude(0, 0);
  const plus = lvl.labels.find(l => l.text === '+10').box;
  const rplus = rolled.labels.find(l => l.text === '+10').box;
  assert.ok(Math.abs(rplus.x1 - plus.x1) > 5 || Math.abs(rplus.y1 - plus.y1) > 5,
    'a 45deg roll should visibly move the +10 rung');
});

test('the pitch ladder never collides with the roll arc ticks', () => {
  const failures = [];
  for (let roll = -45; roll <= 45; roll += 5) {
    for (let pitch = -30; pitch <= 30; pitch += 5) {
      const a = analyseAttitude(roll, pitch);
      for (const lab of a.labels) {
        for (const t of a.ticks) {
          if (boxHit(lab.box, { x1: t.x1, y1: t.y1 }, { x1: t.x2, y1: t.y2 })) {
            failures.push(`roll=${roll} pitch=${pitch}: label ${lab.text} x tick (${t.x1.toFixed(0)},${t.y1.toFixed(0)})`);
          }
        }
      }
    }
  }
  assert.deepEqual(failures, [], 'ladder label/tick collisions:\n' + failures.slice(0, 12).join('\n'));
});

test('the pitch ladder stays inside the bezel with clearance', () => {
  const failures = [];
  for (let roll = -45; roll <= 45; roll += 5) {
    for (let pitch = -30; pitch <= 30; pitch += 5) {
      const a = analyseAttitude(roll, pitch);
      for (const lab of a.labels) {
        for (const [k, v] of Object.entries(lab.box)) void k, v;
        const cx = Math.hypot(lab.box.x1 - a.center, (lab.box.y1 + lab.box.y2) / 2 - a.center);
        if (cx > a.radius - 4) {
          failures.push(`roll=${roll} pitch=${pitch}: label ${lab.text} centre r=${cx.toFixed(1)} > r=${(a.radius - 4).toFixed(1)}`);
        }
      }
    }
  }
  assert.deepEqual(failures, [], 'labels outside the bezel:\n' + failures.slice(0, 12).join('\n'));
});

test('the pitch ladder still labels a readable range', () => {
  // Guards the fix against over-shrinking the ladder into uselessness.
  // Count alone is not enough: collapsing the scale to ~0 keeps all 6 labels
  // but piles them into an unreadable blob, so spacing is asserted too.
  const a = analyseAttitude(0, 0);
  const texts = a.labels.map(l => l.text).sort();
  assert.ok(texts.length >= 4, `expected >=4 labelled rungs at level, got ${JSON.stringify(texts)}`);
  assert.ok(texts.includes('+30') && texts.includes('-30'),
    `expected the +/-30 rungs to be labelled, got ${JSON.stringify(texts)}`);
  const cy = (t) => { const b = a.labels.find(l => l.text === t).box; return (b.y1 + b.y2) / 2; };
  const gap = Math.abs(cy('+20') - cy('+10'));
  assert.ok(gap > 5, `adjacent rungs must be visibly separated, got +10/+20 gap ${gap.toFixed(1)}px`);
  assert.ok(Math.abs(cy('+30') - 90) > 25 && Math.abs(cy('-30') - 90) > 25,
    `the +/-30 rungs must sit well out from the centre, got +30@${cy('+30').toFixed(1)} -30@${cy('-30').toFixed(1)}`);
});

// ===== uPlot x-axis: epoch seconds, never raw milliseconds =====
test('graph x values are seconds and the tooltip renders clock time', () => {
  const src = fs.readFileSync(path.join(__dirname, '..', 'js', 'panels.js'), 'utf8');

  // The scale must be a time scale, or uPlot prints raw numbers as ticks.
  const scaleLine = src.split('\n').find(l => l.includes('scales:'));
  assert.ok(scaleLine && /x:\s*\{\s*time:\s*true/.test(scaleLine),
    `x scale must be a time scale, got: ${JSON.stringify(scaleLine)}`);

  // Timestamps must be stored in seconds. 1.7e12 is milliseconds (the bug:
  // tick labels like 1,791,525,840,000 overflowing the legend on a phone).
  const h = createHarness();
  const realNow = Date.now;
  Date.now = () => 1791525845000;
  try {
    h.ops.length = 0;
    h.sandbox.addGraphDataPoint(1, -2, 0.5);
  } finally { Date.now = realNow; }
  const t = h.sandbox.graphData.labels[h.sandbox.graphData.labels.length - 1];
  assert.ok(t > 1e9 && t < 2e9, `x value must be epoch seconds (~1.79e9), got ${t}`);

  // The cursor tooltip must convert those seconds back to clock time.
  const m = src.match(/rawValue \* 1000\)\.toLocaleTimeString/);
  assert.ok(m, 'tooltip must multiply the seconds value by 1000 for Date');
  const shown = new Date(t * 1000).toLocaleTimeString('en-US', { hour12: false });
  assert.ok(/^\d{1,2}:\d{2}(:\d{2})?/.test(shown), `tooltip should be clock time, got ${shown}`);
  assert.ok(!shown.includes('1791'), 'tooltip must not leak the raw epoch number');
});

// ===== motor bars =====
function motorHarness() {
  const els = {};
  const mk = () => ({ style: {}, textContent: '' });
  for (const id of ['mbarL', 'mbarLNum', 'mbarR', 'mbarRNum']) els[id] = mk();
  const sandbox = {
    Math, Number, Array, Object, String, parseFloat, isFinite,
    document: { getElementById: (id) => els[id] || null },
    window: {},
  };
  vm.createContext(sandbox);
  const src = fs.readFileSync(path.join(__dirname, '..', 'js', 'panels.js'), 'utf8');
  // renderMotorBars only; extract just that function to avoid DOM deps.
  const start = src.indexOf('function renderMotorBars');
  const end = src.indexOf('// Offline gate');
  vm.runInContext('var motorPeak = 255;\n' + src.slice(start, end), sandbox);
  return { sandbox, els };
}

test('motor bars follow the firmware motor_left / motor_right telemetry', () => {
  const { sandbox, els } = motorHarness();

  // Forward on both wheels.
  sandbox.renderMotorBars(1200, 1200);
  assert.equal(els.mbarLNum.textContent, '1200');
  assert.equal(els.mbarRNum.textContent, '1200');
  const fwd = els.mbarL.style.left;
  assert.equal(fwd, '50%', 'forward output fills to the right of centre');
  assert.ok(parseFloat(els.mbarL.style.width) > 0, 'forward output must have width');

  // Reverse on the left only — this is the case that was invisible before.
  sandbox.renderMotorBars(-900, 1500);
  assert.equal(els.mbarLNum.textContent, '-900');
  assert.equal(els.mbarRNum.textContent, '1500');
  assert.ok(els.mbarL.style.left.endsWith('%') && parseFloat(els.mbarL.style.left) < 50,
    `reverse left bar must start left of centre, got ${els.mbarL.style.left}`);
  assert.notEqual(els.mbarL.style.background, els.mbarR.style.background,
    'reverse and forward must not share the same fill');

  // Zero output parks the bars dead centre.
  sandbox.renderMotorBars(0, 0);
  assert.equal(els.mbarL.style.left, '50%');
  assert.equal(els.mbarR.style.left, '50%');
});

test('the motor bar scale tracks the peak instead of clipping', () => {
  const { sandbox, els } = motorHarness();
  sandbox.renderMotorBars(4000, 4000);
  const w = parseFloat(els.mbarL.style.width);
  assert.ok(w > 40 && w <= 50, `peak output should nearly fill the bar, got ${w}%`);
  // A small value after a big one must stay visible, not vanish.
  sandbox.renderMotorBars(20, 20);
  assert.ok(parseFloat(els.mbarL.style.width) > 0, 'small output must still render some width');
});

test('the motor bar scale adapts instead of assuming a fixed 4095 peak', () => {
  // A moderate output must fill a substantial part of the bar. With a fixed
  // 4095 peak it would read 200/4095*50 = 2.4% (a dead-looking bar) while the
  // adaptive peak (~254) gives ~39%. This is what makes small corrections
  // visible on the phone UI.
  const { sandbox, els } = motorHarness();
  sandbox.renderMotorBars(200, 200);
  const w = parseFloat(els.mbarL.style.width);
  assert.ok(w > 20, `moderate output should substantially fill the bar, got ${w}%`);
});
// ===== graph: y locked to +/-20, x ticks can't hit the legend =====
test('graph y is locked to +/-20 with no auto-zoom stretching', () => {
  const src = fs.readFileSync(path.join(__dirname, '..', 'js', 'panels.js'), 'utf8');
  const scaleLine = src.split('\n').find(l => l.includes('scales:'));
  assert.ok(scaleLine && /y:\s*\{\s*auto:\s*false,\s*range:\s*\[\s*-20\s*,\s*20\s*\]/.test(scaleLine),
    `y scale must be locked to [-20,20], got: ${JSON.stringify(scaleLine)}`);
  assert.ok(!src.includes('autoZoomCheck'),
    'auto-zoom code must be gone from panels.js (it rescaled past +/-20)');
  const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
  assert.ok(!html.includes('autoZoomCheck'),
    'Auto Zoom checkbox must be gone from index.html');
});

test('graph height follows the CSS box and x tick labels are gone', () => {
  const src = fs.readFileSync(path.join(__dirname, '..', 'js', 'panels.js'), 'utf8');
  assert.ok(!/height:\s*300\s*[,}]/.test(src),
    'no hardcoded 300px chart height may remain (it overflowed the 260px phone box)');
  assert.ok(src.includes('c.clientHeight') && src.includes('container.clientHeight'),
    'chart init and resize must size from the CSS box (clientHeight)');
  // The x ticks were wall-clock seconds fragments (:10, :15) with no meaning
  // on a 10s rolling window — the axis itself must stay hidden, not thinned.
  assert.ok(/\{\s*show:\s*false\s*\},?\s*\n\s*\{ label: 'Angle/.test(src),
    'x axis must be { show: false } so no tick labels can ever render');
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  assert.ok(/#angleChart\s*\{\s*overflow:\s*hidden/.test(css),
    '#angleChart must clip overflow so nothing can bleed into the legend');
});

// ===== header: one line — battery + link pill + fullscreen =====
test('header is one line with no arm pill or vehicle badge', () => {
  const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
  for (const id of ['armBtn', 'vehicleBadge', 'header-left', 'header-right']) {
    assert.ok(!html.includes(id), `index.html must not contain ${id}`);
  }
  for (const id of ['batteryIndicator', 'connectionStatus', 'fullscreenBtn']) {
    assert.ok(html.includes(`id="${id}"`), `index.html must keep #${id}`);
  }
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  assert.ok(!/#armBtn\s*\{/.test(css), 'dead #armBtn CSS must be gone');
  assert.ok(!/\.vehicle-badge\s*\{/.test(css), 'dead .vehicle-badge CSS must be gone');
  assert.ok(/flex-wrap:\s*nowrap/.test(css), 'phone header must be a single nowrap row');
});

// ===== throttle: quadcopter-era readouts removed, wire value kept =====
test('throttle readouts are gone but the wire value is untouched', () => {
  const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
  for (const id of ['throttle-val', 'telem-throttle', 'kpi-thr', 'kpi-thr-bar']) {
    assert.ok(!html.includes(id), `index.html must not contain ${id}`);
  }
  const stick = fs.readFileSync(path.join(__dirname, '..', 'js', 'stick.js'), 'utf8');
  assert.ok(!stick.includes('throttle-val'), 'stick.js must not write the removed readout');
  assert.ok(/state\.throttle\s*=\s*state\.armed \? 0\.2 : 0/.test(stick),
    'stick.js must still hold the arm throttle value for the wire protocol');
  const link = fs.readFileSync(path.join(__dirname, '..', 'js', 'link.js'), 'utf8');
  assert.ok(!link.includes("getElementById('telem-throttle')"),
    'link.js must not write the removed telemetry row (it would throw on null)');
});

// ===== fullscreen: hole-punch clearance, normal view untouched =====
test('fullscreen pushes the header below the camera cutout', () => {
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  assert.ok(css.includes('html:fullscreen .header'),
    'a :fullscreen header rule must exist for the hole-punch offset');
  assert.ok(/padding-top:\s*max\(28px/.test(css),
    'fullscreen top padding must guarantee >=28px even with no safe-area inset');
});

// ===== landscape: stick left, readout+arm right =====
test('landscape lays control side-by-side instead of stacked', () => {
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'stick.css'), 'utf8');
  const m = css.match(/orientation:\s*landscape\)\s*\{([\s\S]*?)\n\}/);
  assert.ok(m, 'a landscape media block must exist in stick.css');
  assert.ok(/grid-template-columns:[^;]*minmax\(150px,\s*calc\(100dvh - 200px\)\)[^;]*minmax\(190px,\s*1fr\)/.test(m[1]),
    'landscape control-grid must be stick + side columns sized to the short viewport');
  // The landscape block must come AFTER the 1fr portrait collapse it overrides.
  assert.ok(css.indexOf('grid-template-columns: 1fr') < css.indexOf('orientation: landscape'),
    'landscape rule must follow the portrait 1fr collapse or the cascade loses');
});

// ===== portrait control fills the column; landscape pill hugs the icon =====
test('portrait control stretches down to the tab bar', () => {
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  const portrait = css.indexOf('orientation: portrait');
  assert.ok(portrait !== -1, 'a portrait-only media block must exist in panels.css');
  const tail = css.slice(portrait);
  assert.ok(tail.includes('.main-content:has(#panel-control.active)'),
    'portrait block must target the control-active content column');
  assert.ok(tail.includes('flex: 1') && tail.includes('space-evenly'),
    'portrait control card must stretch (flex:1) and spread (space-evenly) so the arm button lands above the tabs');
});

test('landscape link pill hugs the fullscreen icon', () => {
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  const land = css.indexOf('orientation: landscape');
  assert.ok(land !== -1, 'a landscape media block must exist in panels.css');
  const tail = css.slice(land);
  assert.ok(/#connectionStatus\s*\{\s*margin-left:\s*auto/.test(tail),
    'landscape must right-pack #connectionStatus next to the fullscreen icon');
});

// ===== console input is wired in the markup (harness stubs can't see this) =====
test('console input is wired in the markup', () => {
  const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
  assert.ok(html.includes('id="consoleInput"'), 'console text field must exist');
  assert.ok(html.includes("if(event.key==='Enter')sendConsoleLine()"),
    'Enter key must send the console line');
  assert.ok(html.includes('onclick="sendConsoleLine()"'),
    'a Send button must call sendConsoleLine()');
  const link = fs.readFileSync(path.join(__dirname, '..', 'js', 'link.js'), 'utf8');
  assert.ok(link.includes('JSON.stringify({ cli: line })'),
    'Wi-Fi path must send {"cli"} (raw text is ignored by the firmware)');
});
