// link.js — transports: Wi-Fi WebSocket (:81, JSON) or USB Web Serial (115200, text CLI).
// Same stick semantics, same telemetry render — only the wire format differs.
// Serial mirrors the WS live path: sp = pitch_deg * 2.0 (speed_x, SWAP flag off),
// sr = roll_deg * 2.0, yr = yaw * YAW_SIGN(-1) -> yaw_rate_target, t = throttle 0-100.
// Depends on state.js (shared vars) and panels.js (drawAttitude, drawCompass,
// addGraphDataPoint, updateBatteryIndicator) — all called at runtime.
function escapeHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}
function addConsoleMessage(message, cls) {
  // No timestamps: on a 180px phone console they eat a third of every line
  // and add nothing (wall-clock already lives on the graph tooltip).
  var line = message;
  var c = 'console-line' + (cls ? ' ' + cls : '');
  if (!cls && /^>>> /.test(message)) c += ' out';
  else if (!cls && /error|fail|blocked|abort/i.test(message)) c += ' err';
  consoleLines.push({ t: line, c: c });
  if (consoleLines.length > maxConsoleLines) consoleLines.shift();
  var consoleEl = document.getElementById('serialConsole');
  if (!consoleEl) return;
  consoleEl.innerHTML = consoleLines.map(function (l) { return '<div class="' + l.c + '">' + escapeHtml(l.t) + '</div>'; }).join('');
  consoleEl.scrollTop = consoleEl.scrollHeight;
}
function clearConsole() {
  consoleLines = [];
  document.getElementById('serialConsole').innerHTML = '';
}
// Wireless serial monitor input (SERIAL CONSOLE card): same language as the
// Arduino IDE monitor. USB serial writes the raw line; Wi-Fi sends {"cli"}
// and the firmware runs it through the real CLI dispatcher, streaming replies
// back as console frames (see cli_tee.h). Enter key is wired in the markup.
function sendConsoleLine() {
  var input = document.getElementById('consoleInput');
  var line = input ? input.value.trim() : '';
  if (!line) return;
  if (!isConnected) { showError('Not connected — command not sent'); return; }
  addConsoleMessage('>>> ' + line);
  if (transport === 'serial') {
    if (!serialWriter) { showError('Serial writer not ready'); return; }
    serialWrite(line);
  } else {
    if (!ws || ws.readyState !== WebSocket.OPEN) { showError('Link is not open'); return; }
    try {
      ws.send(JSON.stringify({ cli: line }));
    } catch (e) {
      showError('Send failed: ' + (e.message || e));
      return;
    }
  }
  input.value = '';
  input.focus();
}

// ===== Connection =====
function transportTag() {
  return transport === 'serial' ? 'SER' : 'WS';
}
function setConnectionState(connectionState) {
  var connected = connectionState === 'connected';
  var connecting = connectionState === 'connecting';
  isConnected = connected;
  var statusEl = document.getElementById('connectionStatus');
  if (statusEl) {
    statusEl.textContent = connected ? 'CONNECTED (' + transportTag() + ')'
      : connecting ? 'CONNECTING (' + transportTag() + ')'
      : 'DISCONNECTED';
    statusEl.classList.toggle('connected', connected);
    statusEl.classList.toggle('connecting', connecting);
    statusEl.classList.toggle('disconnected', !connected && !connecting);
  }
  // Mode badge is intent, not live state — dim it while offline so it can't
  // read as an active condition next to DISCONNECTED.
  var badge = document.getElementById('vehicleBadge');
  if (badge) badge.style.opacity = connected ? '1' : '0.45';
  // No link = stale data: dim battery + arm pill so header reads dead, not live.
  ['batteryIndicator', 'armBtn'].forEach(function (id) {
    var el = document.getElementById(id);
    if (el) el.style.opacity = connected ? '1' : '0.45';
  });
  var kl2 = document.getElementById('kpi-link');
  if (kl2) kl2.textContent = connected ? (transport === 'serial' ? 'SER' : 'WI-FI') : '--';
  if (typeof syncOfflineOverlay === 'function') syncOfflineOverlay();
  if (typeof renderArm === 'function') renderArm();
}
function setConnectionStatus(connected) {
  setConnectionState(connected ? 'connected' : 'disconnected');
}
var tickTimer = null;
var pktCount = 0;
function tickPulse() {
  // Link activity blink on the connection pill — proves live packets.
  var el = document.getElementById('connectionStatus');
  if (!el) return;
  el.classList.add('tick');
  if (tickTimer) clearTimeout(tickTimer);
  tickTimer = setTimeout(function () { el.classList.remove('tick'); }, 150);
}
function showError(msg) {
  var errorEl = document.getElementById('errorMessage');
  errorEl.textContent = msg;
  errorEl.classList.add('show');
  setTimeout(function () { errorEl.classList.remove('show'); }, 3000);
}
function onTransportChange(v) {
  if (v !== 'serial') v = 'ws';
  if (v === transport) { applyTransportUI(); return; }
  // One link at a time: drop the other transport before switching.
  if (transport === 'ws') disconnectESP32(true);
  else disconnectSerial(true);
  transport = v;
  localStorage.setItem('transport', transport);
  applyTransportUI();
  addConsoleMessage('Link: ' + (transport === 'serial' ? 'USB serial (115200)' : 'Wi-Fi WebSocket (:81)'));
}
function applyTransportUI() {
  var sel = document.getElementById('transportSelect');
  if (sel) sel.value = transport;
  var wsRow = document.getElementById('wsRow');
  var serialRow = document.getElementById('serialRow');
  if (wsRow) wsRow.style.display = transport === 'ws' ? '' : 'none';
  if (serialRow) serialRow.style.display = transport === 'serial' ? '' : 'none';
}
function clearReconnectTimer() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
}
function clearWsWatchdog() {
  if (wsWatchdogTimer) {
    clearInterval(wsWatchdogTimer);
    wsWatchdogTimer = null;
  }
}
function resetReconnectBackoff() {
  reconnectAttempts = 0;
  reconnectDelay = 1000;
}
function detachWebSocketHandlers(socket) {
  if (!socket) return;
  socket.onopen = null;
  socket.onmessage = null;
  socket.onerror = null;
  socket.onclose = null;
}
function closeWebSocket(socket) {
  if (!socket) return;
  detachWebSocketHandlers(socket);
  if (socket.readyState === WebSocket.CONNECTING || socket.readyState === WebSocket.OPEN) {
    try { socket.close(); } catch (e) {}
  }
}
function isCurrentWebSocket(socket, generation) {
  return transport === 'ws' && ws === socket && wsGeneration === generation;
}
function handleWebSocketFailure(socket, generation, reason) {
  if (!isCurrentWebSocket(socket, generation)) return;
  console.warn('WebSocket link failed:', reason);
  clearWsWatchdog();
  clearReconnectTimer();
  ws = null;
  wsGeneration++;
  closeWebSocket(socket);
  setConnectionStatus(false);
  addConsoleMessage(reason + (wsAutoReconnect ? ' — reconnecting.' : '.'));
  if (wsAutoReconnect) scheduleReconnect();
}
function startWsWatchdog(socket, generation) {
  clearWsWatchdog();
  var timer = setInterval(function () {
    if (!isCurrentWebSocket(socket, generation)) {
      clearInterval(timer);
      return;
    }
    if (socket.readyState !== WebSocket.OPEN) {
      handleWebSocketFailure(socket, generation, 'WebSocket is no longer open');
      return;
    }
    if (Date.now() - wsLastMessageAt > wsWatchdogTimeout) {
      handleWebSocketFailure(socket, generation, 'No robot telemetry for ' + (wsWatchdogTimeout / 1000).toFixed(1) + 's');
    }
  }, wsWatchdogInterval);
  wsWatchdogTimer = timer;
}
function connectToESP32() {
  var ipInput = document.getElementById('espIpInput').value.trim();
  if (ipInput) {
    espIP = ipInput;
    localStorage.setItem('espIP', espIP);
  }
  wsAutoReconnect = true;
  clearReconnectTimer();
  resetReconnectBackoff();
  attemptConnection();
}
function disconnectESP32(silent) {
  wsAutoReconnect = false;
  clearReconnectTimer();
  clearWsWatchdog();
  var socket = ws;
  ws = null;
  wsGeneration++;
  closeWebSocket(socket);
  if (transport === 'ws') setConnectionStatus(false);
  if (!silent) addConsoleMessage('Disconnected by user.');
}
function attemptConnection() {
  if (transport !== 'ws' || !wsAutoReconnect) return;

  clearReconnectTimer();
  clearWsWatchdog();
  var previousSocket = ws;
  ws = null;
  wsGeneration++;
  closeWebSocket(previousSocket);

  var generation = wsGeneration;
  var target = espIP;
  var url = 'ws://' + target + ':81';
  console.log('Connecting to ESP32 at:', target);
  addConsoleMessage('Connecting to ' + url + ' ...');
  setConnectionState('connecting');

  try {
    var socket = new WebSocket(url);
    ws = socket;
    socket.onopen = function () {
      if (!isCurrentWebSocket(socket, generation)) {
        closeWebSocket(socket);
        return;
      }
      console.log('WebSocket connected to ESP32');
      clearReconnectTimer();
      resetReconnectBackoff();
      wsLastMessageAt = Date.now();
      setConnectionStatus(true);
      startWsWatchdog(socket, generation);
      addConsoleMessage('Connected to ESP32 at ' + target);
      loadStateFromDevice();
    };
    socket.onmessage = function (event) {
      if (!isCurrentWebSocket(socket, generation)) return;
      try {
        var data = JSON.parse(event.data);
        wsLastMessageAt = Date.now();
        handleDeviceMessage(data);
      } catch (e) {
        console.error('Failed to parse message:', e);
      }
    };
    socket.onerror = function (error) {
      if (!isCurrentWebSocket(socket, generation)) return;
      console.error('WebSocket error:', error);
      handleWebSocketFailure(socket, generation, 'WebSocket error');
    };
    socket.onclose = function () {
      if (!isCurrentWebSocket(socket, generation)) return;
      handleWebSocketFailure(socket, generation, 'WebSocket closed');
    };
  } catch (e) {
    if (transport === 'ws' && wsGeneration === generation) {
      ws = null;
      setConnectionStatus(false);
      console.error('Connection error:', e);
      addConsoleMessage('Connection failed — reconnecting.');
      if (wsAutoReconnect) scheduleReconnect();
    }
  }
}
function scheduleReconnect() {
  if (transport !== 'ws' || !wsAutoReconnect || reconnectTimer) return;
  if (reconnectAttempts >= maxReconnectAttempts) {
    console.log('Max reconnect attempts reached, waiting for manual reconnect');
    addConsoleMessage('Reconnect limit reached — use CONNECT to retry.');
    return;
  }
  var delay = reconnectDelay;
  reconnectAttempts++;
  reconnectDelay = Math.min(delay * 1.5, maxReconnectDelay);
  console.log('Reconnect attempt ' + reconnectAttempts + ' in ' + delay.toFixed(0) + 'ms...');
  reconnectTimer = setTimeout(function () {
    reconnectTimer = null;
    attemptConnection();
  }, delay);
}

// ===== Shared device-message render (both transports land here) =====
function handleDeviceMessage(data) {
  if (data.console !== undefined) {
    addConsoleMessage(data.console);
    return;
  }
  // ===== STATE BROADCAST (auto-fill PID / trim inputs) =====
  if (data.pitch_p !== undefined) {
    document.getElementById('pid-pitch-p').value = data.pitch_p;
    document.getElementById('pid-pitch-i').value = data.pitch_i || 0;
    document.getElementById('pid-pitch-d').value = data.pitch_d || 0;
    document.getElementById('pid-roll-p').value = data.roll_p || 0;
    document.getElementById('pid-roll-i').value = data.roll_i || 0;
    document.getElementById('pid-roll-d').value = data.roll_d || 0;
    document.getElementById('pid-yaw-p').value = data.yaw_p || 0;
    document.getElementById('pid-yaw-i').value = data.yaw_i || 0;
    document.getElementById('pid-yaw-d').value = data.yaw_d || 0;
    document.getElementById('trim-pitch-input').value = data.trim_pitch || 0;
    document.getElementById('trim-roll-input').value = data.trim_roll || 0;
    console.log('State loaded from device');
    addConsoleMessage('State loaded from device.');
    return;
  }
  // ===== ARM / FAILSAFE SYNC (readonly pill, rendered via renderArm) =====
  if (data.armed !== undefined) {
    document.getElementById('telem-armed').textContent = data.armed ? 'YES' : 'NO';
    document.getElementById('telem-armed').style.color = data.armed ? '#FF0A0A' : '#8A8A8A';
    if (data.armed !== state.armed && !userJustToggledArm) {
      state.armed = data.armed;
      console.warn('[FAILSAFE] ESP32 state change - pill synced');
    }
    localState.armed = state.armed;
    renderArm();
  }
  // ===== TELEMETRY =====
  tickPulse();
  pktCount++;
  var pk = document.getElementById('kpi-pkts');
  if (pk) pk.textContent = pktCount > 9999 ? (pktCount / 1000).toFixed(1) + 'k' : pktCount;
  var roll = 0, pitch = 0, yaw = 0;
  if (data.roll !== undefined) {
    roll = data.roll;
    document.getElementById('sensor-roll').textContent = data.roll.toFixed(1);
  }
  if (data.pitch !== undefined) {
    pitch = data.pitch;
    document.getElementById('sensor-pitch').textContent = data.pitch.toFixed(1);
  }
  if (data.yaw !== undefined) {
    yaw = data.yaw;
    document.getElementById('sensor-yaw').textContent = data.yaw.toFixed(1);
  }
  drawAttitude(roll, pitch);
  drawCompass(yaw);
  addGraphDataPoint(roll, pitch, yaw);
  if (data.battery !== undefined) {
    document.getElementById('telem-vbat').textContent = data.battery.toFixed(2);
    updateBatteryIndicator(data.battery);
  }
  if (data.loop_rate !== undefined) {
    document.getElementById('telem-loop').textContent = data.loop_rate.toFixed(0);
    var kl = document.getElementById('kpi-loop');
    if (kl) kl.textContent = data.loop_rate.toFixed(0);
    var klb = document.getElementById('kpi-loop-bar');
    if (klb) klb.style.width = Math.min(100, data.loop_rate) + '%';
  }
  if (data.throttle !== undefined) {
    // Quadcopter-era throttle readouts (telem-throttle, kpi-thr) were removed
    // from the balancing UI; the value is still accepted on the wire and ignored.
  }
  if (data.filter !== undefined) document.getElementById('telem-filter').textContent = data.filter;
  if (data.filter_time !== undefined) document.getElementById('telem-filter-time').textContent = data.filter_time.toFixed(0);
  if (data.avg_filter_time !== undefined) document.getElementById('telem-avg-filter-time').textContent = data.avg_filter_time.toFixed(0);
  if (data.cpu_load !== undefined) document.getElementById('telem-cpu').textContent = data.cpu_load.toFixed(1);
  if (data.heap_free !== undefined) {
    var heapEl = document.getElementById('telem-heap');
    heapEl.textContent = data.heap_free >= 10000 ? (data.heap_free / 1000).toFixed(1) + 'k' : data.heap_free.toFixed(0);
  }
  if (data.pid_pitch !== undefined && data.pid_yaw !== undefined) {
    document.getElementById('telem-pid').textContent = '[' + data.pid_pitch.toFixed(0) + ',' + data.pid_yaw.toFixed(0) + ']';
  }
  // 2-motor balancing robot: [L,R]
  if (data.motor_left !== undefined && data.motor_right !== undefined) {
    document.getElementById('telem-motors').textContent = '[' + data.motor_left.toFixed(0) + ',' + data.motor_right.toFixed(0) + ']';
    if (typeof renderMotorBars === 'function') renderMotorBars(data.motor_left, data.motor_right);
  } else if (data.motor0 !== undefined && data.motor1 !== undefined) {
    document.getElementById('telem-motors').textContent = '[' + data.motor0.toFixed(0) + ',' + data.motor1.toFixed(0) + ']';
    if (typeof renderMotorBars === 'function') renderMotorBars(data.motor0, data.motor1);
  }
}

// ===== Web Serial (USB) transport =====
function serialSupported() {
  return 'serial' in navigator;
}
function serialWrite(line) {
  if (!serialWriter) return;
  var data = new TextEncoder().encode(line + '\n');
  serialQueue = serialQueue.then(function () {
    return serialWriter.write(data);
  }).catch(function (e) {
    console.error('Serial write failed:', e);
    addConsoleMessage('[SER] write failed: ' + e.message);
    disconnectSerial();
  });
}
async function connectSerial() {
  if (!serialSupported()) { showError('USB serial needs Chrome or Edge'); return; }
  if (serialPort) { showError('Serial already open'); return; }
  try {
    addConsoleMessage('[SER] Pick the ESP32 USB port...');
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });
    serialWriter = serialPort.writable.getWriter();
    serialKeepReading = true;
    lastSpSent = lastSrSent = lastYrSent = lastTSent = null;
    lastArmSent = null; lastSerialStickMs = 0;
    setConnectionStatus(true);
    addConsoleMessage('[SER] Open at 115200. Enabling telemetry (status)...');
    serialWrite('status'); // toggles firmware [LOOP] stream on
    serialWrite('load'); // autofill PID/trim inputs, same as WS connect
    readSerialLoop();
  } catch (e) {
    console.error('Serial open failed:', e);
    addConsoleMessage('[SER] open failed: ' + (e.message || e));
    await closeSerialHandles();
    setConnectionStatus(false);
  }
}
async function closeSerialHandles() {
  serialKeepReading = false;
  try { if (serialReader) { await serialReader.cancel(); } } catch (e) {}
  try { if (serialReader) { serialReader.releaseLock(); } } catch (e) {}
  try { if (serialWriter) { serialWriter.releaseLock(); } } catch (e) {}
  try { if (serialPort) { await serialPort.close(); } } catch (e) {}
  serialReader = serialWriter = serialPort = null;
}
async function disconnectSerial(silent) {
  if (!serialPort && !isConnected) return;
  try { if (serialWriter) serialWrite('status'); } catch (e) {} // stream off
  await closeSerialHandles();
  if (transport === 'serial') setConnectionStatus(false);
  if (!silent) addConsoleMessage('[SER] Closed by user.');
}
async function readSerialLoop() {
  var decoder = new TextDecoder();
  var buf = '';
  try {
    serialReader = serialPort.readable.getReader();
    while (serialKeepReading) {
      var res = await serialReader.read();
      if (res.done) break;
      buf += decoder.decode(res.value, { stream: true });
      var idx;
      while ((idx = buf.indexOf('\n')) >= 0) {
        var line = buf.slice(0, idx).trim();
        buf = buf.slice(idx + 1);
        if (line) onSerialLine(line);
      }
    }
  } catch (e) {
    if (serialKeepReading) {
      console.error('Serial read failed:', e);
      addConsoleMessage('[SER] read failed: ' + (e.message || e));
    }
  }
  if (serialKeepReading) disconnectSerial(); // device unplugged mid-stream
}
function onSerialLine(line) {
  if (line.indexOf('[LOOP]') === 0) {
    var data = parseLoopLine(line);
    if (data) handleDeviceMessage(data);
    return; // [LOOP] at 10Hz stays out of the console buffer
  }
  if (line.charAt(0) === '{') {
    // Firmware state JSON (e.g. answer to `load`) — same shape as WS broadcast.
    try {
      handleDeviceMessage(JSON.parse(line));
      return;
    } catch (e) { /* fall through to console mirror */ }
  }
  addConsoleMessage(line);
}
function numAfter(re, line) {
  var m = line.match(re);
  return m ? parseFloat(m[1]) : undefined;
}
// [LOOP] [Filter: X] | Mode: .. | Status: ARMED | Rate: 100.00Hz | Pitch: 1.2deg |
// Roll: ..deg | Yaw: ..deg | PID[P,R]: a,b | Motors[L,R]: a,b | Vbat: 12.1V |
// Az: ..m/s2 | [COMP] Filter: 10us (avg: 9.5us) | Heap: 100000B (min: ..) | Load: 5.0%
function parseLoopLine(line) {
  var d = {};
  var m = line.match(/Status:\s*(ARMED|DISARMED)/);
  if (m) d.armed = (m[1] === 'ARMED');
  var v;
  v = numAfter(/Rate:\s*([-\d.]+)Hz/, line); if (v !== undefined) d.loop_rate = v;
  v = numAfter(/Pitch:\s*([-\d.]+)deg/, line); if (v !== undefined) d.pitch = v;
  v = numAfter(/Roll:\s*([-\d.]+)deg/, line); if (v !== undefined) d.roll = v;
  v = numAfter(/Yaw:\s*([-\d.]+)deg/, line); if (v !== undefined) d.yaw = v;
  v = numAfter(/Vbat:\s*([-\d.]+)V/, line); if (v !== undefined) d.battery = v;
  m = line.match(/PID\[P,R\]:\s*([-\d.]+),([-\d.]+)/);
  if (m) { d.pid_pitch = parseFloat(m[1]); d.pid_yaw = parseFloat(m[2]); }
  m = line.match(/Motors\[L,R\]:\s*([-\d.]+),([-\d.]+)/);
  if (m) { d.motor_left = parseFloat(m[1]); d.motor_right = parseFloat(m[2]); }
  m = line.match(/\[Filter:\s*([^\]]+)\]/);
  if (m) d.filter = m[1].trim();
  v = numAfter(/\[COMP\] Filter:\s*([-\d.]+)us/, line); if (v !== undefined) d.filter_time = v;
  v = numAfter(/\(avg:\s*([-\d.]+)us\)/, line); if (v !== undefined) d.avg_filter_time = v;
  v = numAfter(/Heap:\s*([-\d.]+)B/, line); if (v !== undefined) d.heap_free = v;
  v = numAfter(/Load:\s*([-\d.]+)%/, line); if (v !== undefined) d.cpu_load = v;
  return d;
}

// ===== Transport router: every device send goes through here =====
// obj uses WS-shaped keys; serial path translates to firmware CLI lines.
function deviceSend(obj) {
  if (transport === 'serial') {
    if (!isConnected || !serialWriter) { showError('Serial not connected'); return false; }
    serialSendObject(obj);
    return true;
  }
  if (!isConnected || !ws) {
    showError('Not connected to ESP32');
    return false;
  }
  if (ws.readyState !== WebSocket.OPEN) {
    handleWebSocketFailure(ws, wsGeneration, 'WebSocket send failed: link is not open');
    showError('Connection lost — reconnecting');
    return false;
  }
  try {
    ws.send(JSON.stringify(obj));
    return true;
  } catch (e) {
    console.error('Failed to send command:', e);
    handleWebSocketFailure(ws, wsGeneration, 'WebSocket send failed');
    return false;
  }
}
function serialSendObject(obj) {
  var k;
  if (obj.pitch !== undefined || obj.yaw !== undefined || obj.throttle !== undefined || obj.arm !== undefined) {
    serialSendStick(obj);
    return;
  }
  if (obj.pid_tune) {
    for (k in obj.pid_tune) serialWrite(k + ' ' + obj.pid_tune[k]); // CLI keys match: pitch_p, roll_i, yaw_d, yr...
    addConsoleMessage('>>> PID via serial: ' + JSON.stringify(obj.pid_tune));
    return;
  }
  if (obj.trim_pitch !== undefined) { serialWrite('trim_pitch ' + obj.trim_pitch); return; }
  if (obj.trim_roll !== undefined) { serialWrite('trim_roll ' + obj.trim_roll); return; }
  if (obj.calibrate_gyro) { serialWrite('calibrate_gyro'); return; }
  if (obj.calibrate_accel) { serialWrite('calibrate_accel'); return; }
  if (obj.calib_next) { serialWrite('save'); return; } // serial advance word
  if (obj.calib_abort) { serialWrite('abort'); return; }
  if (obj.load) { serialWrite('load'); addConsoleMessage('>>> load requested (autofills on reply)'); return; }
  if (obj.reset_pid) { serialWrite('reset_pid'); return; }
  if (obj.reset_calibration) { serialWrite('reset_calibration'); return; }
}
// Stick @ ~10Hz over serial, changed-values only, arm on edges.
function serialSendStick(cmd) {
  var now = Date.now();
  if (now - lastSerialStickMs < 100) return;
  lastSerialStickMs = now;
  var sp = (cmd.pitch || 0) * 2.0;   // deg -> m/s, mirrors WS fan-out
  var sr = (cmd.roll || 0) * 2.0;
  var yr = (cmd.yaw || 0) * -1.0;    // CONTROLLER_YAW_SIGN
  var t = cmd.throttle || 0;         // already 0-100 in cmd
  if (sp !== lastSpSent) { serialWrite('sp ' + sp.toFixed(2)); lastSpSent = sp; }
  if (sr !== lastSrSent) { serialWrite('sr ' + sr.toFixed(2)); lastSrSent = sr; }
  if (yr !== lastYrSent) { serialWrite('yr ' + yr.toFixed(1)); lastYrSent = yr; }
  if (t !== lastTSent) { serialWrite('t ' + t.toFixed(0)); lastTSent = t; }
  if (cmd.arm !== lastArmSent) { serialWrite(cmd.arm ? 'arm' : 'disarm'); lastArmSent = cmd.arm; }
}

function sendCommand() {
  localState.roll = state.roll;
  localState.pitch = state.pitch;
  localState.yaw = state.yaw;
  localState.throttle = state.throttle;
  localState.armed = state.armed;
  var cmd = {
    roll: state.roll,
    pitch: state.pitch,
    yaw: state.yaw,
    throttle: state.throttle * 100,
    arm: state.armed
  };
  if (transport === 'serial') {
    if (!isConnected || !serialWriter) return; // quiet offline, same as WS buffering
    serialSendStick(cmd);
    return;
  }
  if (!isConnected || !ws) {
    console.log('Not connected, command buffered locally');
    return;
  }
  if (ws.readyState !== WebSocket.OPEN) {
    handleWebSocketFailure(ws, wsGeneration, 'Command link is no longer open');
    return;
  }
  try {
    ws.send(JSON.stringify(cmd));
  } catch (e) {
    console.error('Failed to send command:', e);
    handleWebSocketFailure(ws, wsGeneration, 'WebSocket command send failed');
  }
}
setInterval(sendCommand, 50);
