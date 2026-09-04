// stick.js — single joystick canvas + mappings per vehicle + select-bug fix listeners.
// Depends on state.js (state, vehicle, isConnected, maxRollPitchAngle, droneThrottlePct).
var stickCanvas = document.getElementById('joystick1');
var stickCtx = stickCanvas.getContext('2d');
var stickMax = 100;
var stickX = 0, stickY = 0; // pixel pos of knob
var stickDragging = false;

// Yaw deadzone (balancing): stick angles within yawDeadzoneDeg of the
// fore-aft axis read as pure pitch — lean sideways past it to yaw.
function yawWithDeadzone(normX, normY, fullScale) {
  var mag = Math.sqrt(normX * normX + normY * normY);
  if (mag < 0.02) return 0;
  var dz = (typeof yawDeadzoneDeg === 'number' && isFinite(yawDeadzoneDeg)) ? yawDeadzoneDeg : 10;
  var ang = Math.atan2(Math.abs(normX), Math.abs(normY)) * 180 / Math.PI;
  if (dz <= 0) return normX * fullScale;
  if (ang <= dz) return 0;
  return normX * fullScale * Math.min(1, (ang - dz) / (90 - dz));
}

function sizeStick() {
  var rect = stickCanvas.getBoundingClientRect();
  var side = Math.max(160, Math.min(rect.width || 300, 640));
  stickCanvas.width = side;
  stickCanvas.height = side;
  stickCanvas.style.height = side + 'px';
  stickMax = side / 2 - 32;
  if (!stickDragging) { stickX = side / 2; stickY = side / 2; }
  drawStick();
}
function drawStick() {
  var w = stickCanvas.width, h = stickCanvas.height;
  var cx = w / 2, cy = h / 2;
  var linked = isConnected;
  // NEBULA VOID instrument: naked field, no plate frame
  stickCtx.clearRect(0, 0, w, h);
  // crosshair axes
  stickCtx.strokeStyle = '#3A3A3A';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.moveTo(cx - stickMax - 6, cy); stickCtx.lineTo(cx + stickMax + 6, cy);
  stickCtx.moveTo(cx, cy - stickMax - 6); stickCtx.lineTo(cx, cy + stickMax + 6);
  stickCtx.stroke();
  // travel ring
  stickCtx.strokeStyle = '#3D3D3D';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.arc(cx, cy, stickMax, 0, Math.PI * 2);
  stickCtx.stroke();
  // range rings: 33/66% faint, 85% dotted orbit
  [0.33, 0.66].forEach(function (f) {
    stickCtx.strokeStyle = '#242424';
    stickCtx.lineWidth = 1;
    stickCtx.beginPath();
    stickCtx.arc(cx, cy, stickMax * f, 0, Math.PI * 2);
    stickCtx.stroke();
  });
  stickCtx.save();
  stickCtx.setLineDash([2, 6]);
  stickCtx.strokeStyle = '#333333';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.arc(cx, cy, stickMax * 0.85, 0, Math.PI * 2);
  stickCtx.stroke();
  stickCtx.restore();
  // vernier graduations along both axes (every 10%, skip center)
  stickCtx.strokeStyle = '#333333';
  stickCtx.lineWidth = 1;
  var vi, v;
  for (vi = 1; vi <= 9; vi++) {
    v = stickMax * vi / 10;
    [[cx - v, cy, cx - v, cy - 4], [cx - v, cy, cx - v, cy + 4],
     [cx + v, cy, cx + v, cy - 4], [cx + v, cy, cx + v, cy + 4],
     [cx, cy - v, cx - 4, cy - v], [cx, cy - v, cx + 4, cy - v],
     [cx, cy + v, cx - 4, cy + v], [cx, cy + v, cx + 4, cy + v]].forEach(function (s) {
      stickCtx.beginPath(); stickCtx.moveTo(s[0], s[1]); stickCtx.lineTo(s[2], s[3]); stickCtx.stroke();
    });
  }
  // cardinal chevrons outside the ring, dim red, pointing out
  stickCtx.fillStyle = 'rgba(204,0,0,0.6)';
  [[0, -1], [0, 1], [-1, 0], [1, 0]].forEach(function (d) {
    var bx = cx + d[0] * (stickMax + 12), by = cy + d[1] * (stickMax + 12);
    stickCtx.save();
    stickCtx.translate(bx, by);
    stickCtx.rotate(Math.atan2(d[1], d[0]) + Math.PI / 2);
    stickCtx.beginPath();
    stickCtx.moveTo(0, -6); stickCtx.lineTo(-4, 0); stickCtx.lineTo(-4, -3);
    stickCtx.lineTo(0, -7); stickCtx.lineTo(4, -3); stickCtx.lineTo(4, 0);
    stickCtx.closePath(); stickCtx.fill();
    stickCtx.restore();
  });
  // 12 ticks: cardinals red, minors gray
  var i, a, x1, y1, x2, y2;
  for (i = 0; i < 12; i++) {
    a = i * Math.PI / 6;
    var cardinal = (i % 3 === 0);
    stickCtx.strokeStyle = cardinal ? '#CC0000' : '#3A3A3A';
    stickCtx.lineWidth = cardinal ? 2 : 1;
    var inner = cardinal ? stickMax - 11 : stickMax - 7;
    x1 = cx + Math.cos(a) * inner; y1 = cy + Math.sin(a) * inner;
    x2 = cx + Math.cos(a) * stickMax; y2 = cy + Math.sin(a) * stickMax;
    stickCtx.beginPath(); stickCtx.moveTo(x1, y1); stickCtx.lineTo(x2, y2); stickCtx.stroke();
  }
  // yaw deadzone rails (about fore-aft): lean past them to yaw
  var dzDraw = (typeof yawDeadzoneDeg === 'number' && isFinite(yawDeadzoneDeg)) ? yawDeadzoneDeg : 10;
  if (typeof vehicle !== 'undefined' && vehicle === 'balancing' && dzDraw > 0) {
    stickCtx.save();
    stickCtx.strokeStyle = 'rgba(138,138,138,0.35)';
    stickCtx.lineWidth = 1;
    [-1, 1].forEach(function (s) {
      var ra = s * dzDraw * Math.PI / 180;
      stickCtx.beginPath();
      stickCtx.moveTo(cx - Math.sin(ra) * stickMax, cy + Math.cos(ra) * stickMax);
      stickCtx.lineTo(cx + Math.sin(ra) * stickMax, cy - Math.cos(ra) * stickMax);
      stickCtx.stroke();
    });
    stickCtx.restore();
  }
  // center home marker
  stickCtx.strokeStyle = '#3A3A3A';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.moveTo(cx - 5, cy); stickCtx.lineTo(cx + 5, cy);
  stickCtx.moveTo(cx, cy - 5); stickCtx.lineTo(cx, cy + 5);
  stickCtx.stroke();
  // deflection sector: filled wedge from center toward knob, magnitude shaded
  var dx = stickX - cx, dy = stickY - cy;
  var defl = Math.sqrt(dx * dx + dy * dy);
  if (defl > 3) {
    var ang0 = Math.atan2(dy, dx);
    stickCtx.save();
    stickCtx.fillStyle = linked ? 'rgba(255,10,10,0.10)' : 'rgba(138,138,138,0.10)';
    stickCtx.beginPath();
    stickCtx.moveTo(cx, cy);
    stickCtx.arc(cx, cy, defl, ang0 - 0.35, ang0 + 0.35);
    stickCtx.closePath();
    stickCtx.fill();
    stickCtx.restore();
    stickCtx.save();
    if (linked) { stickCtx.shadowColor = 'rgba(255,10,10,0.8)'; stickCtx.shadowBlur = 8; }
    stickCtx.strokeStyle = linked ? 'rgba(255,10,10,0.6)' : 'rgba(138,138,138,0.5)';
    stickCtx.lineWidth = 2;
    stickCtx.beginPath(); stickCtx.moveTo(cx, cy); stickCtx.lineTo(stickX, stickY); stickCtx.stroke();
    stickCtx.restore();
  }
  // knob: modern thumbstick nub — solid graphite dome, single hairline rim,
  // soft red glow when live and deflected
  var knobR = Math.max(18, Math.min(34, w * 0.07));
  stickCtx.save();
  if (linked && defl > 3) { stickCtx.shadowColor = 'rgba(255,10,10,0.55)'; stickCtx.shadowBlur = 18; }
  stickCtx.fillStyle = '#1B1B1B';
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, knobR, 0, Math.PI * 2);
  stickCtx.fill();
  stickCtx.restore();
  stickCtx.strokeStyle = linked ? '#FF0A0A' : '#8A8A8A';
  stickCtx.lineWidth = 1.5;
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, knobR, 0, Math.PI * 2);
  stickCtx.stroke();
  // top-light crescent: small offset highlight arc for a domed feel
  stickCtx.strokeStyle = linked ? 'rgba(255,120,120,0.8)' : 'rgba(242,242,242,0.5)';
  stickCtx.lineWidth = 1.5;
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, knobR - 4, Math.PI * 1.1, Math.PI * 1.6);
  stickCtx.stroke();
  // contact dot only while deflected (no dead-pixel look at rest)
  if (defl > 3) {
    stickCtx.fillStyle = linked ? '#FF0A0A' : '#8A8A8A';
    stickCtx.beginPath();
    stickCtx.arc(stickX, stickY, 3, 0, Math.PI * 2);
    stickCtx.fill();
  }
  stickCtx.save();
  stickCtx.setLineDash([4, 5]);
  stickCtx.strokeStyle = 'rgba(138,138,138,0.5)';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath(); stickCtx.arc(cx, cy, stickMax * 0.15, 0, Math.PI * 2); stickCtx.stroke();
  stickCtx.restore();
}
function stickUpdate(clientX, clientY) {
  var rect = stickCanvas.getBoundingClientRect();
  var scaleX = stickCanvas.width / rect.width;
  var scaleY = stickCanvas.height / rect.height;
  var x = (clientX - rect.left) * scaleX - stickCanvas.width / 2;
  var y = (clientY - rect.top) * scaleY - stickCanvas.height / 2;
  var dist = Math.sqrt(x * x + y * y);
  if (dist > stickMax) {
    var a = Math.atan2(y, x);
    x = Math.cos(a) * stickMax;
    y = Math.sin(a) * stickMax;
  }
  stickX = stickCanvas.width / 2 + x;
  stickY = stickCanvas.height / 2 + y;
  var normX = x / stickMax;
  var normY = y / stickMax;
  if (vehicle === 'balancing') {
    // Y -> pitch / speed setpoint, X -> yaw. Throttle held at arm value.
    // Negated: canvas Y is down-positive, lean target is forward-positive.
    state.pitch = -normY * maxRollPitchAngle;
    state.roll = 0;
    state.yaw = yawWithDeadzone(normX, normY, 360);
    state.throttle = state.armed ? 0.2 : 0;
  } else if (vehicle === 'rccar') {
    // Y -> throttle 0-100%, X -> yaw / steer
    state.throttle = Math.max(0, Math.min(1, (1 - normY) / 2));
    state.yaw = normX * 45;
    state.pitch = 0;
    state.roll = 0;
  } else {
    // drone: stick -> pitch/roll, throttle via slider (same canvas convention as balancing)
    state.pitch = -normY * maxRollPitchAngle;
    state.roll = normX * maxRollPitchAngle;
    state.yaw = 0;
    state.throttle = state.armed ? droneThrottlePct / 100 : 0;
  }
  updateDisplay();
  drawStick();
}
function stickReset() {
  stickDragging = false;
  if (stickCanvas.width) { stickX = stickCanvas.width / 2; stickY = stickCanvas.height / 2; }
  if (vehicle === 'balancing') {
    state.pitch = 0; state.yaw = 0; state.roll = 0;
    state.throttle = state.armed ? 0.2 : 0;
  } else if (vehicle === 'rccar') {
    state.throttle = 0; state.yaw = 0; state.pitch = 0; state.roll = 0;
  } else {
    state.pitch = 0; state.roll = 0; state.yaw = 0;
    state.throttle = state.armed ? droneThrottlePct / 100 : 0;
  }
  updateDisplay();
  drawStick();
}
function updateDisplay() {
  document.getElementById('pitch-val').textContent = state.pitch.toFixed(1);
  document.getElementById('roll-val').textContent = state.roll.toFixed(1);
  document.getElementById('yaw-val').textContent = state.yaw.toFixed(0);
  document.getElementById('throttle-val').textContent = (state.throttle * 100).toFixed(0);
}
// Text-select bug fix: preventDefault + passive:false + touchcancel + contextmenu
stickCanvas.addEventListener('contextmenu', function (e) { e.preventDefault(); });
stickCanvas.addEventListener('mousedown', function (e) { e.preventDefault(); stickDragging = true; stickUpdate(e.clientX, e.clientY); });
stickCanvas.addEventListener('mousemove', function (e) { if (stickDragging) { e.preventDefault(); stickUpdate(e.clientX, e.clientY); } });
// Release outside the border must never leave the stick deflected:
// mouseup/touchend land on window when the pointer is released off-canvas.
window.addEventListener('mouseup', function () { if (stickDragging) stickReset(); });
window.addEventListener('blur', function () { if (stickDragging) stickReset(); });
window.addEventListener('touchend', function () { if (stickDragging) stickReset(); });
stickCanvas.addEventListener('mouseup', function (e) { if (e) e.preventDefault(); stickReset(); });
stickCanvas.addEventListener('mouseleave', function () { if (stickDragging) stickReset(); });
stickCanvas.addEventListener('touchstart', function (e) { e.preventDefault(); stickDragging = true; stickUpdate(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
stickCanvas.addEventListener('touchmove', function (e) { if (stickDragging) { e.preventDefault(); stickUpdate(e.touches[0].clientX, e.touches[0].clientY); } }, { passive: false });
stickCanvas.addEventListener('touchend', function (e) { if (e) e.preventDefault(); stickReset(); });
stickCanvas.addEventListener('touchcancel', function () { stickReset(); });
window.addEventListener('resize', function () { sizeStick(); });
