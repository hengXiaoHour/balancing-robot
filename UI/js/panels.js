// panels.js — panel switching, PID/trim/calibration actions, graph init,
// gauges, battery, arm button, fullscreen, init.
// Depends on state.js + link.js + stick.js (all plain classic scripts).
// ===== Sidebar =====
var sidebar = document.getElementById('storeSidebar');
document.getElementById('sidebarToggle').addEventListener('click', function () {
  sidebar.classList.toggle('expanded');
});
var navItems = document.querySelectorAll('.store-sidebar-nav-item');
navItems.forEach(function (btn) {
  btn.addEventListener('click', function () {
    showPanel(btn.getAttribute('data-panel'));
    navItems.forEach(function (b) { b.classList.remove('active'); });
    btn.classList.add('active');
    if (window.innerWidth < 768) sidebar.classList.remove('expanded');
  });
});
function showPanel(name) {
  document.querySelectorAll('.panel-view').forEach(function (p) { p.classList.remove('active'); });
  var el = document.getElementById('panel-' + name);
  if (el) el.classList.add('active');
  if (name === 'control' && typeof sizeStick === 'function') requestAnimationFrame(sizeStick);
  if (name === 'status' || name === 'graph') {
    if (!window.angleChartInstance) {
      setTimeout(function () { initializeAngleChart(); }, 100);
    } else {
      var c = document.getElementById('angleChart');
      if (c && window.angleChartInstance) {
        try { window.angleChartInstance.setSize({ width: c.clientWidth, height: 300 }); } catch (e) {}
      }
    }
  }
}

function toggleFullscreen() {
  try {
    if (!document.fullscreenElement) {
      var el = document.documentElement;
      if (el.requestFullscreen) el.requestFullscreen();
      else if (el.webkitRequestFullscreen) el.webkitRequestFullscreen();
    } else {
      if (document.exitFullscreen) document.exitFullscreen();
      else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
    }
  } catch (e) { console.warn('Fullscreen failed:', e); }
}

function renderArm() {
  // Single writer for the header arm pill (readonly indicator).
  // toggleArm() and telemetry both funnel here — no more alternating text.
  var btn = document.getElementById('armBtn');
  if (btn) {
    btn.textContent = state.armed ? 'ARMED' : 'DISARMED';
    btn.classList.toggle('armed', !!state.armed);
  }
  var hold = document.getElementById('armHoldBtn');
  if (hold && !holdArming) {
    hold.textContent = state.armed ? 'TAP TO DISARM' : (!isConnected ? 'NO LINK — ARM DISABLED' : 'HOLD 1 SEC TO ARM');
    hold.classList.toggle('no-link', !isConnected && !state.armed);
  }
}
// Hold-to-arm: press and hold 1s to arm (progress fill), tap to disarm.
var holdArming = false, armHoldTimer = null, armHoldDone = false;
function armHoldStart(e) {
  if (!isConnected || state.armed || holdArming) return;
  if (e && e.preventDefault) e.preventDefault();
  holdArming = true; armHoldDone = false;
  var hold = document.getElementById('armHoldBtn');
  var t0 = Date.now();
  hold.textContent = 'ARMING…';
  armHoldTimer = setInterval(function () {
    var pct = Math.min(100, (Date.now() - t0) / 10);
    hold.style.setProperty('--hold', pct + '%');
    if (pct >= 100) {
      clearInterval(armHoldTimer); armHoldTimer = null;
      armHoldDone = true; holdArming = false;
      hold.style.setProperty('--hold', '0%');
      toggleArm();
    }
  }, 50);
}
function armHoldCancel() {
  if (!holdArming) return;
  holdArming = false;
  if (armHoldTimer) { clearInterval(armHoldTimer); armHoldTimer = null; }
  var hold = document.getElementById('armHoldBtn');
  if (hold) {
    hold.style.setProperty('--hold', '0%');
    if (!state.armed) hold.textContent = 'HOLD TO ARM';
  }
}
function armHoldClick() {
  if (!isConnected) {
    // Dead button = way back in: pop the link gate instead of nagging.
    offlineDismissed = false;
    syncOfflineOverlay();
    return;
  }
  if (armHoldDone) { armHoldDone = false; return; } // hold already armed — swallow the release click
  if (state.armed) toggleArm();          // tap to disarm, instant
  else showError('Hold 1s to ARM');
}
function toggleArm() {
  if (!isConnected) {
    showError('Not connected to ESP32');
    return;
  }
  state.armed = !state.armed;
  renderArm();
  if (state.armed) {
    if (vehicle === 'balancing') state.throttle = 0.2;
    else if (vehicle === 'drone') state.throttle = droneThrottlePct / 100;
    else state.throttle = 0;
  } else {
    state.throttle = 0;
  }
  updateDisplay();
  userJustToggledArm = true;
  setTimeout(function () { userJustToggledArm = false; }, ARM_TOGGLE_DEBOUNCE);
  sendCommand();
}

// ===== Battery =====
function updateBatteryIndicator(voltage) {
  document.getElementById('batteryVoltage').textContent = voltage.toFixed(2) + ' V';
  var minV = 9.0, maxV = 12.6;
  var pct = Math.max(0, Math.min(1, (voltage - minV) / (maxV - minV)));
  var fillColor = pct > 0.5 ? '#22cc55' : pct > 0.25 ? '#ffcc00' : '#ff4d4d';
  var batteryFill = document.getElementById('batteryFill');
  batteryFill.style.width = (pct * 0.8) + '%';
  batteryFill.setAttribute('fill', fillColor);
  var svg = document.querySelector('.battery-indicator svg');
  if (svg) { svg.style.stroke = fillColor; svg.style.fill = fillColor; }
  document.getElementById('batteryVoltage').style.color = fillColor;
}

// ===== Instruments =====
function drawAttitude(roll, pitch) {
  // Artificial horizon: sky/ground split, pitch ladder, fixed wings, roll arc.
  var canvas = document.getElementById('attitudeCanvas');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var size = canvas.width;
  var center = size / 2;
  var radius = size / 2 - 4;
  ctx.clearRect(0, 0, size, size);
  ctx.save();
  ctx.beginPath();
  ctx.arc(center, center, radius, 0, Math.PI * 2);
  ctx.clip();
  ctx.fillStyle = '#000000';
  ctx.fillRect(0, 0, size, size);
  ctx.translate(center, center);
  ctx.rotate((roll * Math.PI) / 180);
  var px = radius / 45; // px per degree: +/-45deg fills radius
  var pitchOff = pitch * px;
  // sky (dark gray) above horizon, black ground below
  ctx.fillStyle = '#363636';
  ctx.fillRect(-radius * 2, -radius * 2 - pitchOff, radius * 4, radius * 2);
  ctx.fillStyle = '#000000';
  ctx.fillRect(-radius * 2, -pitchOff, radius * 4, radius * 2 + 1);
  // horizon line: red-hot
  ctx.strokeStyle = '#FF0A0A';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-radius * 2, -pitchOff);
  ctx.lineTo(radius * 2, -pitchOff);
  ctx.stroke();
  // pitch ladder every 10deg with labels
  ctx.font = '700 9px "JetBrains Mono", monospace';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  for (var a = -40; a <= 40; a += 10) {
    if (a === 0) continue;
    var y = -pitchOff - a * px;
    if (y < -radius * 1.6 || y > radius * 1.6) continue;
    var wdt = (a % 20 === 0) ? 26 : 14;
    ctx.strokeStyle = 'rgba(242,242,242,0.75)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(-wdt, y); ctx.lineTo(wdt, y);
    ctx.stroke();
    ctx.fillStyle = 'rgba(242,242,242,0.75)';
    ctx.fillText((a > 0 ? '+' : '') + a, wdt + 3, y);
  }
  ctx.restore();
  // roll arc ticks (fixed to frame)
  ctx.save();
  ctx.translate(center, center);
  for (var d = -45; d <= 45; d += 15) {
    var ar = (d * Math.PI) / 180 - Math.PI / 2;
    var r1 = radius - 3, r0 = (d % 45 === 0) ? radius - 12 : radius - 7;
    ctx.strokeStyle = (d === 0) ? '#FF0A0A' : '#8A8A8A';
    ctx.lineWidth = (d === 0) ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(Math.cos(ar) * r0, Math.sin(ar) * r0);
    ctx.lineTo(Math.cos(ar) * r1, Math.sin(ar) * r1);
    ctx.stroke();
  }
  ctx.restore();
  // bezel
  ctx.strokeStyle = '#F2F2F2';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(center, center, radius, 0, Math.PI * 2);
  ctx.stroke();
  // fixed wings: red W silhouette
  ctx.strokeStyle = '#FF0A0A';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(center - 34, center + 2);
  ctx.lineTo(center - 12, center + 2);
  ctx.lineTo(center - 6, center + 8);
  ctx.lineTo(center + 6, center + 8);
  ctx.lineTo(center + 12, center + 2);
  ctx.lineTo(center + 34, center + 2);
  ctx.stroke();
  ctx.fillStyle = '#FF0A0A';
  ctx.beginPath();
  ctx.arc(center, center + 2, 3, 0, Math.PI * 2);
  ctx.fill();
}

function drawCompass(yaw) {
  var canvas = document.getElementById('compassCanvas');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var size = canvas.width;
  var center = size / 2;
  var radius = size / 2 - 4;
  ctx.clearRect(0, 0, size, size);
  ctx.fillStyle = '#000000';
  ctx.beginPath();
  ctx.arc(center, center, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.save();
  ctx.translate(center, center);
  ctx.rotate((-yaw * Math.PI) / 180);
  ctx.fillStyle = '#F2F2F2';
  ctx.font = 'bold 12px Arial';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  var pts = [['N', 0, -radius + 14, '#ff4d4d'], ['E', radius - 14, 0, '#F2F2F2'], ['S', 0, radius - 22, '#F2F2F2'], ['W', -radius + 14, 0, '#F2F2F2']];
  pts.forEach(function (p) {
    ctx.fillStyle = p[3];
    ctx.fillText(p[0], p[1], p[2]);
  });
  for (var d = 0; d < 360; d += 15) {
    var a = (d * Math.PI) / 180;
    var r1 = radius - 4;
    var r0 = (d % 90 === 0) ? radius - 14 : radius - 9;
    ctx.strokeStyle = '#8A8A8A';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(Math.sin(a) * r0, -Math.cos(a) * r0);
    ctx.lineTo(Math.sin(a) * r1, -Math.cos(a) * r1);
    ctx.stroke();
  }
  ctx.restore();
  ctx.save();
  ctx.translate(center, center);
  ctx.rotate((yaw * Math.PI) / 180);
  ctx.fillStyle = '#ff4d4d';
  ctx.beginPath();
  ctx.moveTo(0, -radius + 22);
  ctx.lineTo(-6, -radius + 36);
  ctx.lineTo(6, -radius + 36);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
  ctx.fillStyle = '#F2F2F2';
  ctx.font = 'bold 13px Arial';
  ctx.textAlign = 'center';
  ctx.fillText(yaw.toFixed(0) + '\u00B0', center, size - 8);
  // JARVIS sweep ring: rotating dashed halo (advances with each redraw)
  ctx.save();
  ctx.translate(center, center);
  ctx.rotate((Date.now() / 40 % 360) * Math.PI / 180);
  ctx.strokeStyle = 'rgba(204,0,0,0.55)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([10, 14]);
  ctx.beginPath();
  ctx.arc(0, 0, radius - 1, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

// ===== Calibration =====
function startGyroCalibration() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  try {
    deviceSend({ calibrate_gyro: true });
    document.getElementById('calibGyroBtn').disabled = true;
    document.getElementById('calibAccelBtn').disabled = true;
    document.getElementById('calibNextBtn').style.display = 'none';
    document.getElementById('calibAbortBtn').style.display = 'block';
    document.getElementById('calibStatus').textContent = 'Gyro calibration in progress...';
    document.getElementById('calibInstructions').textContent = 'Keep device LEVEL and STILL.';
  } catch (e) { console.error('Failed to send command:', e); setConnectionStatus(false); }
}
function startAccelCalibration() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  try {
    deviceSend({ calibrate_accel: true });
    document.getElementById('calibGyroBtn').disabled = true;
    document.getElementById('calibAccelBtn').disabled = true;
    document.getElementById('calibNextBtn').style.display = 'block';
    document.getElementById('calibAbortBtn').style.display = 'block';
    document.getElementById('calibStatus').textContent = 'Accel calibration: STEP 1/6 - Level Position';
    document.getElementById('calibInstructions').textContent = 'Place device LEVEL and STILL. Click NEXT STEP to record.';
  } catch (e) { console.error('Failed to send command:', e); setConnectionStatus(false); }
}
function nextCalibrationStep() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  try { deviceSend({ calib_next: true }); }
  catch (e) { console.error('Failed to send command:', e); setConnectionStatus(false); }
}
function abortCalibration() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  try {
    deviceSend({ calib_abort: true });
    document.getElementById('calibGyroBtn').disabled = false;
    document.getElementById('calibAccelBtn').disabled = false;
    document.getElementById('calibNextBtn').style.display = 'none';
    document.getElementById('calibAbortBtn').style.display = 'none';
    document.getElementById('calibStatus').textContent = 'Calibration aborted. Ready.';
    document.getElementById('calibInstructions').textContent = '';
  } catch (e) { console.error('Failed to send command:', e); setConnectionStatus(false); }
}

// ===== PID / trim / reset / load (protocol byte-identical keys) =====
function sendPidValue(param, value) {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  var cmd = { pid_tune: {} };
  cmd.pid_tune[param] = parseFloat(value);
  try {
    deviceSend(cmd);
    addConsoleMessage('>>> PID ' + param + ' = ' + value);
  } catch (e) { console.error('Failed to send PID value:', e); setConnectionStatus(false); }
}
function sendTrimValue(param, value) {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  var cmd = {};
  cmd[param] = parseFloat(value);
  try {
    deviceSend(cmd);
    addConsoleMessage('>>> TRIM ' + param + ' = ' + value + '\u00B0');
  } catch (e) { console.error('Failed to send trim value:', e); setConnectionStatus(false); }
}
function loadStateFromDevice() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  try {
    deviceSend({ load: true });
    addConsoleMessage('>>> Requesting state from device...');
  } catch (e) { console.error('Failed to send load command:', e); setConnectionStatus(false); }
}
function resetPID() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  if (confirm('Reset all PID values to defaults?')) {
    try {
      deviceSend({ reset_pid: true });
      addConsoleMessage('>>> PID Values RESET to defaults');
    } catch (e) { console.error('Failed to send reset command:', e); setConnectionStatus(false); }
  }
}
function resetCalibration() {
  if (!isConnected) { showError('Not connected to ESP32'); return; }
  if (confirm('Reset all calibration data to defaults?')) {
    try {
      deviceSend({ reset_calibration: true });
      addConsoleMessage('>>> Calibration Data RESET to defaults');
    } catch (e) { console.error('Failed to send reset command:', e); setConnectionStatus(false); }
  }
}

// ===== uPlot graph =====
var graphData = { labels: [], roll: [], pitch: [], yaw: [], setpoint: [], maxPoints: 100 };
function toggleDataset(datasetIndex) {
  if (!window.angleChartInstance) return;
  var seriesIndex = datasetIndex + 1;
  var series = window.angleChartInstance.series[seriesIndex];
  if (series) {
    series.show = !series.show;
    window.angleChartInstance.setData(window.angleChartInstance.data);
  }
}
function addGraphDataPoint(roll, pitch, yaw) {
  var timestamp = Date.now();
  graphData.labels.push(timestamp);
  graphData.roll.push(parseFloat(roll) || 0);
  graphData.pitch.push(parseFloat(pitch) || 0);
  graphData.yaw.push(parseFloat(yaw) || 0);
  graphData.setpoint.push(0);
  if (graphData.labels.length > graphData.maxPoints) {
    graphData.labels.shift();
    graphData.roll.shift();
    graphData.pitch.shift();
    graphData.yaw.shift();
    graphData.setpoint.shift();
  }
  if (window.angleChartInstance) {
    var data = [graphData.labels.slice(), graphData.roll.slice(), graphData.pitch.slice(), graphData.yaw.slice(), graphData.setpoint.slice()];
    try {
      var autoZoomEl = document.getElementById('autoZoomCheck');
      if (autoZoomEl && autoZoomEl.checked) {
        var values = [];
        var series = window.angleChartInstance.series || [];
        var seriesMap = [graphData.roll, graphData.pitch, graphData.yaw];
        for (var i = 1; i <= 3; i++) {
          var s = series[i];
          if (s && s.show && seriesMap[i - 1].length) values = values.concat(seriesMap[i - 1]);
        }
        if (values.length > 0) {
          var min = Math.min.apply(null, values);
          var max = Math.max.apply(null, values);
          if (min === max) { min -= 5; max += 5; }
          else { var pad = Math.max(5, (max - min) * 0.12); min -= pad; max += pad; }
          try { window.angleChartInstance.setScale('y', { min: min, max: max }); }
          catch (e) { try { window.angleChartInstance.setScale('y', [min, max]); } catch (e2) {} }
        }
      }
    } catch (e) { console.warn('Auto-zoom error:', e); }
    window.angleChartInstance.setData(data, true);
  }
}
function initializeAngleChart() {
  var container = document.getElementById('angleChart');
  if (!container || typeof uPlot === 'undefined') return;
  container.innerHTML = '';
  var data = [graphData.labels.slice(), graphData.roll.slice(), graphData.pitch.slice(), graphData.yaw.slice(), graphData.setpoint.slice()];
  var opts = {
    title: 'Angle Graphs',
    width: container.clientWidth || 600,
    height: 300,
    series: [
      { label: 'Time', value: function (self, rawValue) {
        if (rawValue == null) return '--';
        return new Date(rawValue).toLocaleTimeString('en-US', { hour12: false });
      }},
      { label: 'Roll', stroke: '#FF0A0A', fill: 'rgba(255,10,10,0.1)', width: 2, spanGaps: false, show: true },
      { label: 'Pitch', stroke: '#F2F2F2', fill: 'rgba(242,242,242,0.1)', width: 2, spanGaps: false, show: true },
      { label: 'Yaw', stroke: '#8A8A8A', fill: 'rgba(138,138,138,0.1)', width: 2, spanGaps: false, show: false },
      { label: 'Setpoint (0\u00B0)', stroke: '#ffcc00', width: 2, dash: [4, 4], spanGaps: false, show: true }
    ],
    scales: { x: { time: false }, y: { auto: true, range: [-180, 180] } },
    axes: [
      { label: 'Time', stroke: '#8A8A8A', font: '11px Arial', grid: { stroke: 'rgba(138,138,138,0.15)', width: 0.5 } },
      { label: 'Angle (\u00B0)', stroke: '#8A8A8A', font: '11px Arial', grid: { stroke: 'rgba(138,138,138,0.15)', width: 0.5 } }
    ],
    legend: { show: false },
    cursor: { show: true, x: true, y: true }
  };
  window.angleChartInstance = new uPlot(opts, data, container);
  window.angleChartInstance.data = data;
}

// Bidirectional center-zero motor bars (auto-scales to peak output).
var motorPeak = 255;
function renderMotorBars(l, r) {
  if (l === undefined) return;
  motorPeak = Math.max(100, motorPeak * 0.995, Math.abs(l), Math.abs(r));
  [['mbarL', 'mbarLNum', l], ['mbarR', 'mbarRNum', r]].forEach(function (m) {
    var fill = document.getElementById(m[0]);
    var num = document.getElementById(m[1]);
    if (!fill || !num) return;
    var v = m[2] || 0;
    var pct = Math.min(50, Math.abs(v) / motorPeak * 50);
    fill.style.width = pct + '%';
    fill.style.left = v < 0 ? (50 - pct) + '%' : '50%';
    fill.style.background = v < 0 ? '#8A8A8A' : '#FF0A0A';
    num.textContent = v.toFixed(0);
  });
}
// Offline gate: connect CTAs + view-offline dismiss (session only).
var offlineDismissed = false;
function offlineConnect(t) {
  offlineDismissed = false;
  if (t === 'ws') {
    // Wi-Fi needs the robot's IP: take it from the gate field, fall back to saved.
    var ipEl = document.getElementById('offlineIpInput');
    var ip = ipEl ? ipEl.value.trim() : '';
    if (!ip) ip = espIP;
    if (!/^\d{1,3}(\.\d{1,3}){3}$/.test(ip)) { showError('Bad IP — use 4 numbers like 192.168.100.27'); return; }
    espIP = ip;
    localStorage.setItem('espIP', espIP);
    var main = document.getElementById('espIpInput');
    if (main) main.value = espIP;
  }
  onTransportChange(t);
  if (t === 'serial') connectSerial();
  else connectToESP32();
}
function offlineDismiss() {
  offlineDismissed = true;
  document.getElementById('offlineOverlay').style.display = 'none';
  if (typeof sizeStick === 'function') requestAnimationFrame(sizeStick);
}
// Help icons: tap toggles the tip on touch screens (hover covers desktop)
document.addEventListener('click', function (e) {
  var h = (e.target && e.target.closest) ? e.target.closest('.help-icon') : null;
  document.querySelectorAll('.help-icon.open').forEach(function (el) { if (el !== h) el.classList.remove('open'); });
  if (h) h.classList.toggle('open');
});
function syncOfflineOverlay() {
  if (offlineDismissed) return;
  document.getElementById('offlineOverlay').style.display = isConnected ? 'none' : 'flex';
}
window.addEventListener('load', function () {
  document.getElementById('espIpInput').value = espIP;
  var offIp = document.getElementById('offlineIpInput');
  if (offIp) offIp.value = espIP;
  var slider = document.getElementById('maxAngleSlider');
  if (slider) slider.value = maxRollPitchAngle;
  document.getElementById('maxAngleValue').textContent = maxRollPitchAngle + '\u00B0';
  var ydz = document.getElementById('yawDeadzoneSlider');
  if (ydz) ydz.value = yawDeadzoneDeg;
  document.getElementById('yawDeadzoneValue').textContent = yawDeadzoneDeg + '\u00B0';
  var yr = document.getElementById('yawRateSlider');
  if (yr) yr.value = yawRateMax;
  document.getElementById('yawRateValue').textContent = yawRateMax + '\u00B0/S';
  applyVehicleUI();
  updateDisplay();
  renderArm();
  (function () {
    var hold = document.getElementById('armHoldBtn');
    if (hold) {
      hold.addEventListener('mousedown', armHoldStart);
      hold.addEventListener('mouseup', armHoldCancel);
      hold.addEventListener('mouseleave', armHoldCancel);
      hold.addEventListener('click', armHoldClick);
      hold.addEventListener('touchstart', armHoldStart, { passive: false });
      hold.addEventListener('touchend', function (e) { if (e) e.preventDefault(); var wasArming = holdArming; armHoldCancel(); if (!wasArming) armHoldClick(); });
    }
  })();
  syncOfflineOverlay();
  sizeStick();
  drawAttitude(0, 0);
  drawCompass(0);
  if (typeof renderMotorBars === 'function') renderMotorBars(0, 0);
  // JARVIS boot chant: proves console levels + sets the tone on every load.
  addConsoleMessage('VISWA OS v1.0 // NEBULA VOID', 'dim');
  addConsoleMessage('LINK SCAN: wi-fi :81 + usb serial 115200', 'dim');
  addConsoleMessage('SYSTEMS NOMINAL — AWAITING LINK', 'dim');
  applyTransportUI();
  if (transport === 'serial') {
    addConsoleMessage('System ready. Pick USB serial in SETUP > CONNECTION.');
  } else {
    connectToESP32();
    addConsoleMessage('System ready. Waiting for ESP32...');
  }
});
