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
  // NEBULA VOID instrument: black field, hairline plate frame
  stickCtx.fillStyle = '#000000';
  stickCtx.fillRect(0, 0, w, h);
  stickCtx.strokeStyle = '#1E1E1E';
  stickCtx.lineWidth = 1;
  stickCtx.strokeRect(4.5, 4.5, w - 9, h - 9);
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
  // deflection vector: center -> knob, faint red
  var dx = stickX - cx, dy = stickY - cy;
  if (Math.sqrt(dx * dx + dy * dy) > 3) {
    stickCtx.strokeStyle = linked ? 'rgba(255,10,10,0.45)' : 'rgba(138,138,138,0.4)';
    stickCtx.lineWidth = 2;
    stickCtx.beginPath(); stickCtx.moveTo(cx, cy); stickCtx.lineTo(stickX, stickY); stickCtx.stroke();
  }
  // knob: dark fill, red ring when linked / gray idle, hot core dot
  var knobR = Math.max(18, Math.min(34, w * 0.07));
  stickCtx.fillStyle = '#101010';
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, knobR, 0, Math.PI * 2);
  stickCtx.fill();
  stickCtx.strokeStyle = linked ? '#FF0A0A' : '#8A8A8A';
  stickCtx.lineWidth = 2;
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, knobR, 0, Math.PI * 2);
  stickCtx.stroke();
  stickCtx.fillStyle = linked ? '#FF0A0A' : '#8A8A8A';
  stickCtx.beginPath();
  stickCtx.arc(stickX, stickY, 3, 0, Math.PI * 2);
  stickCtx.fill();
  // grip ring inside the knob + dashed deadzone circle on the field
  stickCtx.strokeStyle = 'rgba(242,242,242,0.35)';
  stickCtx.lineWidth = 1.5;
  stickCtx.beginPath(); stickCtx.arc(stickX, stickY, knobR - 6, 0, Math.PI * 2); stickCtx.stroke();
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
