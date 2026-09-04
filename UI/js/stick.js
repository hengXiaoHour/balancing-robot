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
  if (window.innerWidth < 768) {
    // Fluid fit: stick takes exactly the leftover vertical room so the
    // control tab never scrolls on short phones nor floats on tall ones.
    var card = document.querySelector('#panel-control .card');
    var wrapH = document.querySelector('.joystick-wrap').getBoundingClientRect().height;
    var nonStick = card ? card.getBoundingClientRect().height - wrapH : 300;
    var mc2 = document.querySelector('.main-content');
    var padV = 24;
    if (mc2) { var mcs = getComputedStyle(mc2); padV = parseFloat(mcs.paddingTop) + parseFloat(mcs.paddingBottom); }
    var room = (mc2 ? mc2.clientHeight : window.innerHeight) - padV - nonStick;
    side = Math.max(160, Math.min(side, 320, room));
  }
  stickCanvas.width = side;
  stickCanvas.height = side;
  stickCanvas.style.width = side + 'px';
  stickCanvas.style.height = side + 'px';
  stickMax = side / 2 - 32;
  if (!stickDragging) { stickX = side / 2; stickY = side / 2; }
  drawStick();
}
function drawStick() {
  var w = stickCanvas.width, h = stickCanvas.height;
  var cx = w / 2, cy = h / 2;
  var linked = isConnected;
  // MINIMAL instrument: one hairline travel ring, four cardinal dots,
  // whisper crosshair, deadzone rails, arc position indicator, nub.
  stickCtx.clearRect(0, 0, w, h);
  // travel ring
  stickCtx.strokeStyle = '#3D3D3D';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.arc(cx, cy, stickMax, 0, Math.PI * 2);
  stickCtx.stroke();
  // four cardinal dots (red N, gray rest)
  [['#CC0000', 0, -1], ['#4A4A4A', 0, 1], ['#4A4A4A', -1, 0], ['#4A4A4A', 1, 0]].forEach(function (c) {
    stickCtx.fillStyle = c[0];
    stickCtx.beginPath();
    stickCtx.arc(cx + c[1] * stickMax, cy + c[2] * stickMax, 2.5, 0, Math.PI * 2);
    stickCtx.fill();
  });
  // whisper crosshair (short, faint)
  stickCtx.strokeStyle = 'rgba(58,58,58,0.5)';
  stickCtx.lineWidth = 1;
  stickCtx.beginPath();
  stickCtx.moveTo(cx - stickMax * 0.3, cy); stickCtx.lineTo(cx + stickMax * 0.3, cy);
  stickCtx.moveTo(cx, cy - stickMax * 0.3); stickCtx.lineTo(cx, cy + stickMax * 0.3);
  stickCtx.stroke();
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
  // deflection: slim ring-arc at stick angle + hairline tether — no wedge fill
  var dx = stickX - cx, dy = stickY - cy;
  var defl = Math.sqrt(dx * dx + dy * dy);
  if (defl > 3) {
    var ang0 = Math.atan2(dy, dx);
    stickCtx.save();
    stickCtx.strokeStyle = linked ? '#FF0A0A' : '#8A8A8A';
    stickCtx.lineWidth = 3;
    stickCtx.lineCap = 'round';
    stickCtx.beginPath();
    stickCtx.arc(cx, cy, stickMax, ang0 - 0.22, ang0 + 0.22);
    stickCtx.stroke();
    stickCtx.restore();
    stickCtx.strokeStyle = linked ? 'rgba(255,10,10,0.35)' : 'rgba(138,138,138,0.3)';
    stickCtx.lineWidth = 1;
    stickCtx.beginPath(); stickCtx.moveTo(cx, cy); stickCtx.lineTo(stickX, stickY); stickCtx.stroke();
    // overdrive halo: outer arc grows while pinned at the edge
    if (typeof odFactor === 'number' && odFactor > 1.01) {
      stickCtx.save();
      stickCtx.strokeStyle = '#FF0A0A';
      stickCtx.globalAlpha = Math.min(1, (odFactor - 1) * 2);
      stickCtx.lineWidth = 2;
      stickCtx.lineCap = 'round';
      stickCtx.beginPath();
      stickCtx.arc(cx, cy, stickMax + 8, ang0 - 0.5, ang0 + 0.5);
      stickCtx.stroke();
      stickCtx.restore();
    }
  }
  // nub: small solid graphite dot with hairline rim, glow only when live
  var knobR = Math.max(14, Math.min(24, w * 0.05));
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
  odNormX = x / stickMax;
  odNormY = y / stickMax;
  odWatch(Math.sqrt(odNormX * odNormX + odNormY * odNormY));
  applyStickOutput();
}
// Hold-at-edge overdrive (balancing): pin past 95% and the command keeps
// climbing to 1.5x while held. Ease off and it drops back to normal.
var odNormX = 0, odNormY = 0, odStart = 0, odTimer = null, odFactor = 1;
function odTick() {
  if (!stickDragging) { odStop(); return; }
  odFactor = 1 + Math.min(0.5, (Date.now() - odStart) / 1000 * 0.25);
  applyStickOutput();
}
function odStop() {
  odFactor = 1;
  if (odTimer) { clearInterval(odTimer); odTimer = null; }
}
function odWatch(mag) {
  if (vehicle === 'balancing' && stickDragging && mag > 0.95) {
    if (!odTimer) { odStart = Date.now(); odFactor = 1; odTimer = setInterval(odTick, 100); }
  } else {
    odStop();
  }
}
function applyStickOutput() {
  var normX = odNormX, normY = odNormY;
  if (vehicle === 'balancing') {
    // Y -> pitch / speed setpoint, X -> yaw. Throttle held at arm value.
    // Negated: canvas Y is down-positive, lean target is forward-positive.
    state.pitch = -normY * maxRollPitchAngle * odFactor;
    state.roll = 0;
    state.yaw = yawWithDeadzone(normX, normY, 360 * odFactor);
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
  odStop();
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
// Held drags track even off-canvas (clamped to the edge in stickUpdate):
// leaving the border pins the stick, only release recenters it.
window.addEventListener('mousemove', function (e) { if (stickDragging && e.buttons) stickUpdate(e.clientX, e.clientY); });
window.addEventListener('touchmove', function (e) { if (stickDragging && e.touches.length) stickUpdate(e.touches[0].clientX, e.touches[0].clientY); }, { passive: true });
stickCanvas.addEventListener('touchstart', function (e) { e.preventDefault(); stickDragging = true; stickUpdate(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
stickCanvas.addEventListener('touchmove', function (e) { if (stickDragging) { e.preventDefault(); stickUpdate(e.touches[0].clientX, e.touches[0].clientY); } }, { passive: false });
stickCanvas.addEventListener('touchend', function (e) { if (e) e.preventDefault(); stickReset(); });
stickCanvas.addEventListener('touchcancel', function () { stickReset(); });
window.addEventListener('resize', function () { sizeStick(); });
