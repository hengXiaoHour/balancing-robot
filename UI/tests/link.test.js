'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const linkSource = fs.readFileSync(path.join(__dirname, '..', 'js', 'link.js'), 'utf8');

class FakeElement {
  constructor() {
    this.textContent = '';
    this.value = '';
    this.innerHTML = '';
    this.scrollTop = 0;
    this.scrollHeight = 0;
    this.style = {};
    this.focusCalls = 0;
    this.focus = () => { this.focusCalls++; };
    this.classes = new Set();
    this.classList = {
      add: (name) => this.classes.add(name),
      remove: (name) => this.classes.delete(name),
      toggle: (name, enabled) => {
        if (enabled) this.classes.add(name);
        else this.classes.delete(name);
        return enabled;
      },
    };
  }
}

function createHarness() {
  let nextTimerId = 1;
  const timeouts = new Map();
  const intervals = new Map();
  const sockets = [];
  const receivedMessages = [];
  let loadRequests = 0;
  let nextSocketThrows = false;

  function addTimer(store, callback, delay) {
    const id = nextTimerId++;
    store.set(id, { callback, delay });
    return id;
  }

  function removeTimer(store, id) {
    if (id !== undefined && id !== null) store.delete(id);
  }

  class FakeWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;

    constructor(url) {
      if (nextSocketThrows) {
        nextSocketThrows = false;
        throw new Error('mock constructor failure');
      }
      this.url = url;
      this.readyState = FakeWebSocket.CONNECTING;
      this.closeCalls = 0;
      this.onopen = null;
      this.onmessage = null;
      this.onerror = null;
      this.onclose = null;
      sockets.push(this);
    }

    open() {
      this.readyState = FakeWebSocket.OPEN;
      if (this.onopen) this.onopen({ type: 'open' });
    }

    message(data) {
      if (this.onmessage) this.onmessage({ data: JSON.stringify(data) });
    }

    error(message = 'mock socket error') {
      if (this.onerror) this.onerror(new Error(message));
    }

    serverClose() {
      this.readyState = FakeWebSocket.CLOSED;
      if (this.onclose) this.onclose({ type: 'close', code: 1006 });
    }

    close() {
      this.closeCalls++;
      this.readyState = FakeWebSocket.CLOSED;
    }

    send() {
      // Successful by default; individual tests can replace this method.
    }
  }

  const elementIds = [
    'connectionStatus', 'vehicleBadge', 'batteryIndicator', 'armBtn', 'kpi-link',
    'errorMessage', 'serialConsole', 'consoleInput', 'espIpInput', 'transportSelect', 'wsRow',
    'serialRow',
  ];
  const elements = Object.fromEntries(elementIds.map((id) => [id, new FakeElement()]));
  elements.connectionStatus.classes.add('disconnected');
  elements.espIpInput.value = '192.168.100.27';

  const storage = new Map();
  const sandbox = {
    console: { log() {}, warn() {}, error() {} },
    Date,
    JSON,
    Math,
    String,
    TextEncoder,
    Uint8Array,
    WebSocket: FakeWebSocket,
    document: {
      getElementById(id) {
        return elements[id] || null;
      },
    },
    localStorage: {
      getItem(key) {
        return storage.has(key) ? storage.get(key) : null;
      },
      setItem(key, value) {
        storage.set(key, String(value));
      },
    },
    navigator: {},
    setTimeout(callback, delay) {
      return addTimer(timeouts, callback, delay);
    },
    clearTimeout(id) {
      removeTimer(timeouts, id);
    },
    setInterval(callback, delay) {
      return addTimer(intervals, callback, delay);
    },
    clearInterval(id) {
      removeTimer(intervals, id);
    },

    // Shared state normally loaded from state.js.
    espIP: '192.168.100.27',
    ws: null,
    wsGeneration: 0,
    wsAutoReconnect: false,
    wsWatchdogTimer: null,
    wsLastMessageAt: 0,
    wsWatchdogInterval: 250,
    wsWatchdogTimeout: 1500,
    isConnected: false,
    transport: 'ws',
    serialPort: null,
    serialReader: null,
    serialWriter: null,
    serialKeepReading: false,
    serialQueue: Promise.resolve(),
    lastSpSent: null,
    lastSrSent: null,
    lastYrSent: null,
    lastTSent: null,
    lastArmSent: null,
    lastSerialStickMs: 0,
    reconnectTimer: null,
    reconnectAttempts: 0,
    reconnectDelay: 1000,
    maxReconnectAttempts: 10,
    maxReconnectDelay: 10000,
    consoleLines: [],
    maxConsoleLines: 200,
    state: { roll: 0, pitch: 0, yaw: 0, throttle: 0, armed: false },
    localState: { roll: 0, pitch: 0, yaw: 0, throttle: 0, armed: false },
    userJustToggledArm: false,
    ARM_TOGGLE_DEBOUNCE: 1500,
    syncOfflineOverlay() {},
    renderArm() {},
    loadStateFromDevice() {
      loadRequests++;
    },
  };

  vm.createContext(sandbox);
  vm.runInContext(linkSource, sandbox, { filename: 'UI/js/link.js' });
  sandbox.handleDeviceMessage = (message) => receivedMessages.push(message);

  return {
    context: sandbox,
    elements,
    sockets,
    receivedMessages,
    timeouts,
    intervals,
    get loadRequests() { return loadRequests; },
    failNextConstruction() { nextSocketThrows = true; },
    openLatest() {
      const socket = sockets[sockets.length - 1];
      socket.open();
      return socket;
    },
    runOnlyTimeout() {
      assert.equal(timeouts.size, 1, 'expected exactly one pending timeout');
      const [id, timer] = timeouts.entries().next().value;
      timeouts.delete(id);
      timer.callback();
    },
    runWatchdog() {
      const watchdog = [...intervals.entries()].find(([, timer]) => timer.delay === sandbox.wsWatchdogInterval);
      assert.ok(watchdog, 'expected a WebSocket watchdog interval');
      watchdog[1].callback();
    },
  };
}

test('stale socket events cannot overwrite the current connection state', () => {
  const h = createHarness();
  h.context.reconnectAttempts = 7;
  h.context.reconnectDelay = 9000;

  h.context.connectToESP32();
  assert.equal(h.elements.connectionStatus.textContent, 'CONNECTING (WS)');
  assert.equal(h.context.reconnectAttempts, 0);
  assert.equal(h.context.reconnectDelay, 1000);

  const socketA = h.sockets[0];
  const staleClose = socketA.onclose;
  socketA.open();
  assert.equal(h.elements.connectionStatus.textContent, 'CONNECTED (WS)');
  assert.equal(h.loadRequests, 1);

  h.context.connectToESP32();
  assert.equal(socketA.closeCalls, 1, 'the replaced socket should be closed');
  assert.equal(socketA.onclose, null, 'the replaced socket handlers should be detached');

  const socketB = h.openLatest();
  staleClose();
  assert.equal(h.elements.connectionStatus.textContent, 'CONNECTED (WS)');
  assert.equal(h.context.reconnectTimer, null);
  assert.equal(h.sockets.length, 2);
  assert.ok(socketB);
});

test('an error schedules exactly one reconnect and a later close cannot duplicate it', () => {
  const h = createHarness();
  h.context.connectToESP32();
  const socket = h.openLatest();
  const laterClose = socket.onclose;

  socket.error();
  assert.equal(h.elements.connectionStatus.textContent, 'DISCONNECTED');
  assert.equal(h.timeouts.size, 1);
  assert.equal(h.context.reconnectAttempts, 1);

  laterClose();
  assert.equal(h.timeouts.size, 1);
  assert.equal(h.context.reconnectAttempts, 1);
});

test('manual connect cancels a pending reconnect and resets backoff', () => {
  const h = createHarness();
  h.context.connectToESP32();
  h.openLatest().error();
  assert.equal(h.timeouts.size, 1);

  h.context.connectToESP32();
  assert.equal(h.timeouts.size, 0);
  assert.equal(h.context.reconnectAttempts, 0);
  assert.equal(h.context.reconnectDelay, 1000);
  h.openLatest();
  assert.equal(h.elements.connectionStatus.textContent, 'CONNECTED (WS)');
});

test('the watchdog marks a silent robot disconnected and reconnects', () => {
  const h = createHarness();
  h.context.connectToESP32();
  h.openLatest();
  h.context.wsLastMessageAt = Date.now() - 5000;

  h.runWatchdog();
  assert.equal(h.elements.connectionStatus.textContent, 'DISCONNECTED');
  assert.equal(h.timeouts.size, 1);
});

test('a valid frame refreshes liveness before the watchdog fires', () => {
  const h = createHarness();
  h.context.connectToESP32();
  const socket = h.openLatest();
  h.context.wsLastMessageAt = Date.now() - 5000;

  socket.message({ roll: 1, pitch: 2, yaw: 3 });
  h.runWatchdog();
  assert.equal(h.elements.connectionStatus.textContent, 'CONNECTED (WS)');
  assert.equal(h.timeouts.size, 0);
  assert.equal(h.receivedMessages.length, 1);
});

test('a WebSocket send failure enters the same reconnect path', () => {
  const h = createHarness();
  h.context.connectToESP32();
  const socket = h.openLatest();
  socket.send = () => { throw new Error('mock send failure'); };

  assert.equal(h.context.deviceSend({ arm: true }), false);
  assert.equal(h.elements.connectionStatus.textContent, 'DISCONNECTED');
  assert.equal(h.timeouts.size, 1);
});

test('manual disconnect disables reconnect and invalidates remaining callbacks', () => {
  const h = createHarness();
  h.context.connectToESP32();
  const socket = h.openLatest();
  const laterClose = socket.onclose;

  h.context.disconnectESP32(false);
  assert.equal(h.elements.connectionStatus.textContent, 'DISCONNECTED');
  assert.equal(h.context.ws, null);
  assert.equal(h.timeouts.size, 0);

  laterClose();
  assert.equal(h.timeouts.size, 0);
});

test('constructor failure is retried through the same backoff path', () => {
  const h = createHarness();
  h.context.connectToESP32();
  const firstSocket = h.openLatest();
  firstSocket.error();
  const failedGeneration = h.context.wsGeneration;

  h.failNextConstruction();
  h.runOnlyTimeout();
  assert.equal(h.elements.connectionStatus.textContent, 'DISCONNECTED');
  assert.ok(h.context.wsGeneration > failedGeneration);
  assert.equal(h.sockets.length, 1);
  assert.equal(h.timeouts.size, 1, 'constructor failure should schedule the next retry');
});

test('console input sends {"cli"} over Wi-Fi and echoes locally', () => {
  const h = createHarness();
  h.context.isConnected = true;
  h.context.transport = 'ws';
  const sent = [];
  h.context.ws = { readyState: 1, send(m) { sent.push(m); } };
  h.elements.consoleInput.value = 'help';
  h.context.sendConsoleLine();
  assert.equal(sent.length, 1);
  assert.deepEqual(JSON.parse(sent[0]), { cli: 'help' });
  const last = h.context.consoleLines[h.context.consoleLines.length - 1];
  assert.ok(last.t.includes('>>> help'), 'typed line must echo in the console');
  assert.equal(h.elements.consoleInput.value, '', 'input clears after send');
  assert.equal(h.elements.consoleInput.focusCalls, 1, 'focus stays in the input');
});

test('console input writes the raw line over USB serial', async () => {
  const h = createHarness();
  h.context.isConnected = true;
  h.context.transport = 'serial';
  const written = [];
  h.context.serialWriter = { write(data) { written.push(data); return Promise.resolve(); } };
  h.elements.consoleInput.value = 'sp 1.5';
  h.context.sendConsoleLine();
  await new Promise((r) => setImmediate(r));
  await new Promise((r) => setImmediate(r));
  assert.equal(written.length, 1);
  assert.equal(new TextDecoder().decode(written[0]), 'sp 1.5\n');
});

test('console input ignores empty lines and refuses while offline', () => {
  const h = createHarness();
  h.context.isConnected = true;
  h.context.transport = 'ws';
  const sent = [];
  h.context.ws = { readyState: 1, send(m) { sent.push(m); } };
  h.elements.consoleInput.value = '   ';
  h.context.sendConsoleLine();
  assert.equal(sent.length, 0, 'blank line must not send');
  assert.equal(h.context.consoleLines.length, 0, 'blank line must not echo');

  h.context.isConnected = false;
  h.elements.consoleInput.value = 'help';
  h.context.sendConsoleLine();
  assert.equal(sent.length, 0, 'offline must not send');
  assert.equal(h.elements.consoleInput.value, 'help', 'offline keeps the typed line');
  assert.ok(h.elements.errorMessage.classes.has('show'), 'offline explains via the error banner');
});

test('console lines carry no timestamps and echoes stand out', () => {
  const h = createHarness();
  h.context.addConsoleMessage('Connecting to ws://192.168.100.27:81 ...');
  h.context.addConsoleMessage('>>> help');
  h.context.addConsoleMessage('WebSocket error - reconnecting.');
  const lines = h.context.consoleLines;
  assert.equal(lines.length, 3);
  for (const l of lines) {
    assert.ok(!/\[\d{1,2}:\d{2}(:\d{2})?]/.test(l.t), `no clock stamp, got: ${l.t}`);
  }
  assert.ok(lines[0].c.includes('console-line') && !lines[0].c.includes('out'));
  assert.ok(lines[1].c.includes('out'), 'typed echo gets the .out class');
  assert.ok(lines[2].c.includes('err'), 'error text keeps the .err class');
  const css = fs.readFileSync(path.join(__dirname, '..', 'css', 'panels.css'), 'utf8');
  assert.ok(!/word-break:\s*break-all/.test(css), 'break-all chops words mid-glyph');
  assert.ok(/overflow-wrap:\s*anywhere/.test(css), 'long tokens still wrap inside the plate');
});
