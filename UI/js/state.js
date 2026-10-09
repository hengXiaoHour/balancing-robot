// state.js — shared state object, vehicle select + persistence, settings.
// Plain classic script (no modules) so it works over file:// and static hosting.
// All top-level `var` declarations are shared across the other classic scripts.
// When the page is served BY the robot (AP hotspot), talk to whatever host
// served the page. File:// or static hosting falls back to the last default.
var servedHost = (typeof location !== 'undefined' && location.hostname) ? location.hostname : '';
var espIP = localStorage.getItem('espIP') || servedHost || '192.168.100.27';
// Single-vehicle build: the stick is always the balancing-robot mapping.
// Kept as a variable (not a literal at each use site) so the mapping code
// reads the same as it did when a second vehicle existed.
var vehicle = 'balancing';
var maxRollPitchAngle = parseFloat(localStorage.getItem('maxAngle') || '10');
if (!isFinite(maxRollPitchAngle)) maxRollPitchAngle = 10;
maxRollPitchAngle = Math.max(5, Math.min(15, maxRollPitchAngle));
var yawDeadzoneDeg = parseFloat(localStorage.getItem('yawDeadzone') || '10');
if (!isFinite(yawDeadzoneDeg)) yawDeadzoneDeg = 10;
var yawRateMax = parseFloat(localStorage.getItem('yawRate') || '100');
if (!isFinite(yawRateMax)) yawRateMax = 100;

var ws = null;
var wsGeneration = 0;
var wsAutoReconnect = false;
var wsWatchdogTimer = null;
var wsLastMessageAt = 0;
var wsWatchdogInterval = 250;
var wsWatchdogTimeout = 1500;
var isConnected = false;
// Link transport: 'ws' (Wi-Fi WebSocket :81) or 'serial' (USB Web Serial 115200).
// Same stick semantics, same telemetry render — only the wire format differs
// (JSON over WS, text CLI over serial). No firmware change required.
var transport = localStorage.getItem('transport') || 'ws';
if (transport !== 'serial') transport = 'ws';
var serialPort = null, serialReader = null, serialWriter = null;
var serialKeepReading = false, serialQueue = Promise.resolve();
var lastSpSent = null, lastSrSent = null, lastYrSent = null, lastTSent = null;
var lastArmSent = null, lastSerialStickMs = 0;
var reconnectTimer = null;
var reconnectAttempts = 0;
var reconnectDelay = 1000;
var maxReconnectAttempts = 10;
var maxReconnectDelay = 10000;

var consoleLines = [];
var maxConsoleLines = 200;

var state = { roll: 0, pitch: 0, yaw: 0, throttle: 0, armed: false };
var localState = { roll: 0, pitch: 0, yaw: 0, throttle: 0, armed: false };
var userJustToggledArm = false;
var ARM_TOGGLE_DEBOUNCE = 1500;

// ===== Vehicle badge (fixed: balancing robot only) =====
var VEHICLE_HINT = 'Stick Y = speed setpoint, X = yaw. Throttle held at arm value.';
function applyVehicleUI() {
  var badge = document.getElementById('vehicleBadge');
  if (badge) {
    badge.textContent = 'BALANCING';
    badge.title = VEHICLE_HINT;
  }
  var title = document.getElementById('stickTitle');
  if (title) title.textContent = 'CONTROL STICK (SPEED / YAW)';
}
function resetStickState() {
  state.roll = 0; state.pitch = 0; state.yaw = 0;
  state.throttle = state.armed ? 0.2 : 0;
  updateDisplay();
  drawStick();
}

// ===== Settings =====
function updateMaxAngle(v) {
  maxRollPitchAngle = Math.max(5, Math.min(15, parseFloat(v) || 10));
  localStorage.setItem('maxAngle', String(maxRollPitchAngle));
  document.getElementById('maxAngleValue').textContent = maxRollPitchAngle + '\u00B0';
  var s = document.getElementById('maxAngleSlider');
  if (s) s.value = maxRollPitchAngle;
}
function resetSettings() {
  updateMaxAngle(10);
  updateYawDeadzone(10);
  updateYawRate(100);
}
function updateYawDeadzone(v) {
  yawDeadzoneDeg = Math.max(0, Math.min(45, parseFloat(v) || 0));
  localStorage.setItem('yawDeadzone', String(yawDeadzoneDeg));
  document.getElementById('yawDeadzoneValue').textContent = yawDeadzoneDeg + '\u00B0';
  var s = document.getElementById('yawDeadzoneSlider');
  if (s) s.value = yawDeadzoneDeg;
  drawStick();
}
function updateYawRate(v) {
  yawRateMax = Math.max(20, Math.min(200, parseFloat(v) || 100));
  localStorage.setItem('yawRate', String(yawRateMax));
  document.getElementById('yawRateValue').textContent = yawRateMax + '\u00B0/S';
  var s = document.getElementById('yawRateSlider');
  if (s) s.value = yawRateMax;
}
