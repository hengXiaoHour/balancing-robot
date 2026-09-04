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
  if (name === 'graph') {
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

function toggleArm() {
  if (!isConnected) {
    showError('Not connected to ESP32');
    return;
  }
  state.armed = !state.armed;
  var btn = document.getElementById('armBtn');
  btn.textContent = state.armed ? 'ARMED' : 'DISARMED';
  btn.classList.toggle('armed', state.armed);
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
  var fillColor = pct > 0.5 ? '#00e5cc' : pct > 0.25 ? '#ffcc00' : '#ff4d4d';
  var batteryFill = document.getElementById('batteryFill');
  batteryFill.style.width = (pct * 0.8) + '%';
  batteryFill.setAttribute('fill', fillColor);
  var svg = document.querySelector('.battery-indicator svg');
  if (svg) { svg.style.stroke = fillColor; svg.style.fill = fillColor; }
  document.getElementById('batteryVoltage').style.color = fillColor;
}

// ===== Instruments =====
function drawAttitude(roll, pitch) {
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
  ctx.fillStyle = '#0a5cad';
  ctx.fillRect(0, 0, size, size);
  ctx.translate(center, center);
  ctx.rotate((roll * Math.PI) / 180);
  var pitchOffset = (pitch * radius) / 90;
  ctx.fillStyle = '#3d7a1e';
  ctx.fillRect(-radius * 2, -pitchOffset, radius * 4, radius * 2);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-radius, -pitchOffset);
  ctx.lineTo(radius, -pitchOffset);
  ctx.stroke();
  ctx.strokeStyle = 'rgba(255,255,255,0.4)';
  ctx.lineWidth = 1;
  for (var a = -60; a <= 60; a += 15) {
    if (a === 0) continue;
    var y = -pitchOffset - (a * radius) / 90;
    ctx.beginPath();
    ctx.moveTo(-20, y);
    ctx.lineTo(20, y);
    ctx.stroke();
  }
  ctx.restore();
  ctx.strokeStyle = '#00e5cc';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(center, center, radius, 0, Math.PI * 2);
  ctx.stroke();
  ctx.strokeStyle = '#ff4d4d';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(center - 30, center);
  ctx.lineTo(center + 30, center);
  ctx.stroke();
  ctx.fillStyle = '#ff4d4d';
  ctx.beginPath();
  ctx.arc(center, center, 4, 0, Math.PI * 2);
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
  ctx.fillStyle = '#050810';
  ctx.beginPath();
  ctx.arc(center, center, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.save();
  ctx.translate(center, center);
  ctx.rotate((-yaw * Math.PI) / 180);
  ctx.fillStyle = '#f0f4ff';
  ctx.font = 'bold 12px Arial';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  var pts = [['N', 0, -radius + 14, '#ff4d4d'], ['E', radius - 14, 0, '#f0f4ff'], ['S', 0, radius - 14, '#f0f4ff'], ['W', -radius + 14, 0, '#f0f4ff']];
  pts.forEach(function (p) {
    ctx.fillStyle = p[3];
    ctx.fillText(p[0], p[1], p[2]);
  });
  for (var d = 0; d < 360; d += 15) {
    var a = (d * Math.PI) / 180;
    var r1 = radius - 4;
    var r0 = (d % 90 === 0) ? radius - 14 : radius - 9;
    ctx.strokeStyle = '#8892b0';
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
  ctx.fillStyle = '#00e5cc';
  ctx.font = 'bold 13px Arial';
  ctx.textAlign = 'center';
  ctx.fillText(yaw.toFixed(0) + '\u00B0', center, size - 8);
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
      { label: 'Roll', stroke: '#ff6b6b', fill: 'rgba(255,107,107,0.1)', width: 2, spanGaps: false, show: true },
      { label: 'Pitch', stroke: '#00e5cc', fill: 'rgba(0,229,204,0.1)', width: 2, spanGaps: false, show: true },
      { label: 'Yaw', stroke: '#60a5fa', fill: 'rgba(96,165,250,0.1)', width: 2, spanGaps: false, show: false },
      { label: 'Setpoint (0\u00B0)', stroke: '#ffcc00', width: 2, dash: [4, 4], spanGaps: false, show: true }
    ],
    scales: { x: { time: false }, y: { auto: true, range: [-180, 180] } },
    axes: [
      { label: 'Time', stroke: '#8892b0', font: '11px Arial', grid: { stroke: 'rgba(137,146,176,0.15)', width: 0.5 } },
      { label: 'Angle (\u00B0)', stroke: '#8892b0', font: '11px Arial', grid: { stroke: 'rgba(137,146,176,0.15)', width: 0.5 } }
    ],
    legend: { show: false },
    cursor: { show: true, x: true, y: true }
  };
  window.angleChartInstance = new uPlot(opts, data, container);
  window.angleChartInstance.data = data;
}

// ===== Init =====
window.addEventListener('load', function () {
  document.getElementById('espIpInput').value = espIP;
  var slider = document.getElementById('maxAngleSlider');
  if (slider) slider.value = maxRollPitchAngle;
  document.getElementById('maxAngleValue').textContent = maxRollPitchAngle + '\u00B0';
  applyVehicleUI();
  updateDisplay();
  sizeStick();
  drawAttitude(0, 0);
  drawCompass(0);
  applyTransportUI();
  if (transport === 'serial') {
    addConsoleMessage('System ready. Pick USB serial in SETUP > CONNECTION.');
  } else {
    connectToESP32();
    addConsoleMessage('System ready. Waiting for ESP32...');
  }
});
