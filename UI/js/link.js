// link.js — WebSocket connect/reconnect, message parse, telemetry render, console.
// Depends on state.js (shared vars) and panels.js (drawAttitude, drawCompass,
// addGraphDataPoint, updateBatteryIndicator) — all called at runtime.
function escapeHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}
function addConsoleMessage(message) {
  var line = '[' + new Date().toLocaleTimeString('en-US', { hour12: false }) + '] ' + message;
  consoleLines.push(line);
  if (consoleLines.length > maxConsoleLines) consoleLines.shift();
  var consoleEl = document.getElementById('serialConsole');
  if (!consoleEl) return;
  consoleEl.innerHTML = consoleLines.map(function (l) { return '<div class="console-line">' + escapeHtml(l) + '</div>'; }).join('');
  consoleEl.scrollTop = consoleEl.scrollHeight;
}
function clearConsole() {
  consoleLines = [];
  document.getElementById('serialConsole').innerHTML = '';
}

// ===== Connection =====
function setConnectionStatus(connected) {
  isConnected = connected;
  var statusEl = document.getElementById('connectionStatus');
  statusEl.textContent = connected ? 'CONNECTED' : 'DISCONNECTED';
  statusEl.classList.toggle('connected', connected);
  statusEl.classList.toggle('disconnected', !connected);
}
function showError(msg) {
  var errorEl = document.getElementById('errorMessage');
  errorEl.textContent = msg;
  errorEl.classList.add('show');
  setTimeout(function () { errorEl.classList.remove('show'); }, 3000);
}
function connectToESP32() {
  var ipInput = document.getElementById('espIpInput').value.trim();
  if (ipInput) {
    espIP = ipInput;
    localStorage.setItem('espIP', espIP);
  }
  attemptConnection();
}
function disconnectESP32() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  reconnectAttempts = maxReconnectAttempts;
  if (ws) { try { ws.close(); } catch (e) {} ws = null; }
  setConnectionStatus(false);
  addConsoleMessage('Disconnected by user.');
}
function attemptConnection() {
  try {
    if (ws) { try { ws.close(); } catch (e) {} }
    console.log('Connecting to ESP32 at:', espIP);
    addConsoleMessage('Connecting to ws://' + espIP + ':81 ...');
    ws = new WebSocket('ws://' + espIP + ':81');
    ws.onopen = function () {
      console.log('WebSocket connected to ESP32');
      setConnectionStatus(true);
      reconnectAttempts = 0;
      reconnectDelay = 1000;
      addConsoleMessage('Connected to ESP32 at ' + espIP);
      loadStateFromDevice();
    };
    ws.onmessage = function (event) {
      try {
        var data = JSON.parse(event.data);
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
        // ===== ARM / FAILSAFE SYNC =====
        if (data.armed !== undefined) {
          var armedEl = document.getElementById('telem-armed');
          armedEl.textContent = data.armed ? 'YES' : 'NO';
          armedEl.style.color = data.armed ? '#00e5cc' : '#ff4d4d';
          var armBtn = document.getElementById('armBtn');
          armBtn.textContent = data.armed ? 'ARMED' : 'DISARMED';
          armBtn.classList.toggle('armed', !!data.armed);
          if (data.armed !== state.armed && !userJustToggledArm) {
            state.armed = data.armed;
            console.warn('[FAILSAFE] ESP32 state change - button synced');
          }
          localState.armed = state.armed;
        }
        // ===== TELEMETRY =====
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
        if (data.loop_rate !== undefined) document.getElementById('telem-loop').textContent = data.loop_rate.toFixed(0);
        if (data.throttle !== undefined) document.getElementById('telem-throttle').textContent = data.throttle.toFixed(0);
        if (data.filter !== undefined) document.getElementById('telem-filter').textContent = data.filter;
        if (data.filter_time !== undefined) document.getElementById('telem-filter-time').textContent = data.filter_time.toFixed(0);
        if (data.avg_filter_time !== undefined) document.getElementById('telem-avg-filter-time').textContent = data.avg_filter_time.toFixed(0);
        if (data.cpu_load !== undefined) document.getElementById('telem-cpu').textContent = data.cpu_load.toFixed(1);
        if (data.heap_free !== undefined) document.getElementById('telem-heap').textContent = data.heap_free.toFixed(0);
        if (data.pid_pitch !== undefined && data.pid_yaw !== undefined) {
          document.getElementById('telem-pid').textContent = '[' + data.pid_pitch.toFixed(0) + ',' + data.pid_yaw.toFixed(0) + ']';
        }
        // 2-motor balancing robot: [L,R]
        if (data.motor_left !== undefined && data.motor_right !== undefined) {
          document.getElementById('telem-motors').textContent = '[' + data.motor_left.toFixed(0) + ',' + data.motor_right.toFixed(0) + ']';
        } else if (data.motor0 !== undefined && data.motor1 !== undefined) {
          document.getElementById('telem-motors').textContent = '[' + data.motor0.toFixed(0) + ',' + data.motor1.toFixed(0) + ']';
        }
      } catch (e) {
        console.error('Failed to parse message:', e);
      }
    };
    ws.onerror = function (error) {
      console.error('WebSocket error:', error);
      setConnectionStatus(false);
    };
    ws.onclose = function () {
      console.log('WebSocket disconnected, attempting to reconnect...');
      setConnectionStatus(false);
      scheduleReconnect();
    };
  } catch (e) {
    setConnectionStatus(false);
    console.error('Connection error:', e);
    scheduleReconnect();
  }
}
function scheduleReconnect() {
  if (reconnectAttempts >= maxReconnectAttempts) {
    console.log('Max reconnect attempts reached, waiting for manual reconnect');
    return;
  }
  reconnectAttempts++;
  reconnectDelay = Math.min(reconnectDelay * 1.5, maxReconnectDelay);
  console.log('Reconnect attempt ' + reconnectAttempts + ' in ' + reconnectDelay.toFixed(0) + 'ms...');
  if (reconnectTimer) clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(attemptConnection, reconnectDelay);
}

function sendCommand() {
  localState.roll = state.roll;
  localState.pitch = state.pitch;
  localState.yaw = state.yaw;
  localState.throttle = state.throttle;
  localState.armed = state.armed;
  if (!isConnected || !ws || ws.readyState !== WebSocket.OPEN) {
    console.log('Not connected, command buffered locally');
    return;
  }
  var cmd = {
    roll: state.roll,
    pitch: state.pitch,
    yaw: state.yaw,
    throttle: state.throttle * 100,
    arm: state.armed
  };
  try {
    ws.send(JSON.stringify(cmd));
  } catch (e) {
    console.error('Failed to send command:', e);
  }
}
setInterval(sendCommand, 50);
