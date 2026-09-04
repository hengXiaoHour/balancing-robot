// state.js — shared state object, vehicle select + persistence, settings.
// Plain classic script (no modules) so it works over file:// and static hosting.
// All top-level `var` declarations are shared across the other classic scripts.
var espIP = localStorage.getItem('espIP') || '192.168.100.27';
var vehicle = localStorage.getItem('vehicleType') || 'balancing';
if (['balancing', 'rccar', 'drone'].indexOf(vehicle) === -1) vehicle = 'balancing';
var maxRollPitchAngle = parseFloat(localStorage.getItem('maxAngle') || '10');
if (!isFinite(maxRollPitchAngle)) maxRollPitchAngle = 10;

var ws = null;
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
var droneThrottlePct = 0;

// ===== Vehicle selector =====
var VEHICLE_HINTS = {
  balancing: 'Stick Y = speed setpoint, X = yaw. Throttle held at arm value.',
  rccar: 'Stick Y = throttle 0-100%, X = steer.',
  drone: 'Stick = pitch/roll. Throttle via slider below.'
};
function onVehicleChange(v) {
  vehicle = v;
  localStorage.setItem('vehicleType', v);
  applyVehicleUI();
  resetStickState();
}
function applyVehicleUI() {
  var sel = document.getElementById('vehicleSelect');
  if (sel) sel.value = vehicle;
  var hint = document.getElementById('vehicleHint');
  if (hint) hint.textContent = VEHICLE_HINTS[vehicle] || '';
  var droneWrap = document.getElementById('droneThrottleWrap');
  if (droneWrap) droneWrap.style.display = (vehicle === 'drone') ? 'block' : 'none';
  var title = document.getElementById('stickTitle');
  if (title) {
    title.textContent = vehicle === 'balancing' ? 'CONTROL STICK (SPEED / YAW)'
      : vehicle === 'rccar' ? 'CONTROL STICK (THROTTLE / STEER)'
      : 'CONTROL STICK (PITCH / ROLL)';
  }
}
function onDroneThrottle(v) {
  droneThrottlePct = Math.max(0, Math.min(100, parseFloat(v) || 0));
  document.getElementById('droneThrottleVal').textContent = droneThrottlePct.toFixed(0);
  if (vehicle === 'drone' && state.armed) {
    state.throttle = droneThrottlePct / 100;
    updateDisplay();
  }
}
function resetStickState() {
  state.roll = 0; state.pitch = 0; state.yaw = 0;
  if (vehicle === 'balancing') {
    state.throttle = state.armed ? 0.2 : 0;
  } else if (vehicle === 'rccar') {
    state.throttle = 0;
  } else {
    state.throttle = state.armed ? droneThrottlePct / 100 : 0;
  }
  updateDisplay();
  drawStick();
}

// ===== Settings =====
function updateMaxAngle(v) {
  maxRollPitchAngle = Math.max(5, Math.min(45, parseFloat(v) || 10));
  localStorage.setItem('maxAngle', String(maxRollPitchAngle));
  document.getElementById('maxAngleValue').textContent = maxRollPitchAngle + '\u00B0';
  var s = document.getElementById('maxAngleSlider');
  if (s) s.value = maxRollPitchAngle;
}
function resetSettings() {
  updateMaxAngle(10);
}
