// stick.js — single joystick canvas + mappings per vehicle + select-bug fix listeners.
// Depends on state.js (state, vehicle, isConnected, maxRollPitchAngle, droneThrottlePct).
var stickCanvas = document.getElementById('joystick1');
var stickCtx = stickCanvas.getContext('2d');
var stickMax = 100;
var stickX = 0, stickY = 0; // pixel pos of knob
var stickDragging = false;

function sizeStick() {
  var rect = stickCanvas.getBoundingClientRect();
  var side = Math.max(160, Math.min(rect.width || 300, 340));
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
  stickCtx.fillStyle = '#000000';
  stickCtx.fillRect(0, 0, w, h);
  // crosshair axes
  stickCtx.strokeStyle = '#2A2A2A';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.moveTo(cx - stickMax - 6, cy); stickCtx.lineTo(cx + stickMax + 6, cy);
  stickCtx.moveTo(cx, cy - stickMax - 6); stickCtx.lineTo(cx, cy + stickMax + 6);
  stickCtx.stroke();
  // travel ring + tick marks
  stickCtx.strokeStyle = '#3A3A3A';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.arc(cx, cy, stickMax, 0, Math.PI * 2);
  stickCtx.stroke();
  var i, a, x1, y1, x2, y2;
  for (i = 0; i < 12; i++) {
    a = i * Math.PI / 6;
    x1 = cx + Math.cos(a) * (stickMax - 7); y1 = cy + Math.sin(a) * (stickMax - 7);
    x2 = cx + Math.cos(a) * stickMax; y2 = cy + Math.sin(a) * stickMax;
    stickCtx.beginPath(); stickCtx.moveTo(x1, y1); stickCtx.lineTo(x2, y2); stickCtx.stroke();
  }
  // center home dot
  stickCtx.fillStyle = '#3A3A3A';
  stickCtx.beginPath();
  stickCtx.arc(cx, cy, 4, 0, Math.PI * 2);
  stickCtx.fill();
  // knob: gray idle, red when linked
  stickCtx.fillStyle = isConnected ? '#CC0000' : '#8A8A8A';
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, 22, 0, Math.PI * 2);
  stickCtx.fill();
  stickCtx.strokeStyle = isConnected ? '#FF0A0A' : '#F2F2F2';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, 22, 0, Math.PI * 2);
  stickCtx.stroke();
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
    state.yaw = normX * 360;
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
stickCanvas.addEventListener('mouseup', function (e) { if (e) e.preventDefault(); stickReset(); });
stickCanvas.addEventListener('mouseleave', function () { if (stickDragging) stickReset(); });
stickCanvas.addEventListener('touchstart', function (e) { e.preventDefault(); stickDragging = true; stickUpdate(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
stickCanvas.addEventListener('touchmove', function (e) { if (stickDragging) { e.preventDefault(); stickUpdate(e.touches[0].clientX, e.touches[0].clientY); } }, { passive: false });
stickCanvas.addEventListener('touchend', function (e) { if (e) e.preventDefault(); stickReset(); });
stickCanvas.addEventListener('touchcancel', function () { stickReset(); });
window.addEventListener('resize', function () { sizeStick(); });
