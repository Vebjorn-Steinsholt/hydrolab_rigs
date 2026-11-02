'use strict';

const PLOT_SECONDS = 20;
const STALE_MS = 1000;  // no update for this long counts as disconnected
const GAP_SECONDS = 0.25;  // longer pauses between samples break the line
const LOG_LINES = 500;  // older log lines are dropped from the page

const $ = (id) => document.getElementById(id);
const series = { setpoint: [], position: [] };
let state = 'unknown';
// The rig's texts, from api/config; index.html shows the wavemaker texts until they arrive.
let texts = {
  machine: 'wavemaker', motion: 'wave', a_motion: 'a wave',
  returning: 'Returning to upright…',
};
let serverTime = 0;
let lastUpdate = 0;

function connected() {
  return Date.now() - lastUpdate < STALE_MS;
}

function render() {
  const online = connected();
  $('link').textContent = online ? 'connected' : 'disconnected';
  $('link').className = 'badge ' + (online ? 'ok' : 'bad');
  $('state').textContent = online ? state : 'unknown';
  $('state').className = 'badge ' + (
    !online ? '' : state === 'idle' ? 'ok' :
      ['starting', 'running', 'stopping'].includes(state) ? 'busy' : 'bad');
  $('start').disabled = !(online && state === 'idle');
  $('stop').disabled = !online;
  $('return').disabled = !(online && state === 'idle');
  $('keepRunning').disabled = !online;
}

function formatDuration(seconds) {
  const total = Math.max(Math.round(seconds / 60), 0);
  const hours = Math.floor(total / 60), minutes = total % 60;
  if (hours && minutes) return `${hours} h ${minutes} min`;
  return hours ? `${hours} h` : `${minutes} min`;
}

function showIdleWarning(update) {
  const stopIn = update.idle_stop_in;
  $('idleWarning').hidden = stopIn === null;
  if (stopIn === null) return;
  const at = new Date(Date.now() + stopIn * 1000)
    .toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit' });
  const minutes = Math.floor(stopIn / 60);
  const seconds = String(Math.floor(stopIn % 60)).padStart(2, '0');
  $('idleText').textContent =
    `No ${texts.motion} for ${formatDuration(update.idle_timeout - stopIn)}: ` +
    `the ${texts.machine} stops at ` +
    `${at} (in ${minutes}:${seconds}).`;
}

function onUpdate(update) {
  lastUpdate = Date.now();
  state = update.state;
  serverTime = update.time;
  $('namespace').textContent = update.namespace;
  $('message').textContent = update.message || '—';
  $('notice').hidden = !update.notice;
  $('notice').textContent = update.notice;
  if (update.idle_timeout > 0) $('idleTimeout').textContent = formatDuration(update.idle_timeout);
  showIdleWarning(update);
  for (const [t, kind, value] of update.samples) {
    series[kind].push([t, value]);
  }
  for (const points of Object.values(series)) {
    while (points.length && points[0][0] < serverTime - PLOT_SECONDS) points.shift();
  }
  appendLogs(update.logs);
  render();
  drawPlot();
}

function formatStamp(stamp) {
  const date = new Date(stamp * 1000);
  const ms = String(date.getMilliseconds()).padStart(3, '0');
  return `${date.toLocaleTimeString('en-GB')}.${ms}`;
}

function appendLogs(entries) {
  if (!entries.length) return;
  const box = $('log');
  // Keep following new lines unless the user has scrolled up to read older ones.
  const follow = box.scrollTop + box.clientHeight >= box.scrollHeight - 20;
  for (const entry of entries) {
    const line = document.createElement('div');
    line.className = `log-line ${entry.level}`;
    for (const [cls, text] of [
      ['log-time', formatStamp(entry.stamp)],
      ['log-level', entry.level.toUpperCase()],
      ['log-node', entry.node],
      ['log-text', entry.text],
    ]) {
      const span = document.createElement('span');
      span.className = cls;
      span.textContent = text;
      line.append(span);
    }
    box.append(line);
  }
  while (box.childElementCount > LOG_LINES) box.firstElementChild.remove();
  if (follow) box.scrollTop = box.scrollHeight;
}

async function post(path, body) {
  const response = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
  return response.json();
}

function connect() {
  const events = new EventSource('api/events');
  // The server sends its whole log history on every new connection, so start over.
  events.onopen = () => $('log').replaceChildren();
  events.onmessage = (event) => {
    // Every acknowledged event keeps the bridge heartbeat going.
    fetch('api/ack', { method: 'POST' }).catch(() => {});
    onUpdate(JSON.parse(event.data));
  };
  events.onerror = () => render();
}

function drawPlot() {
  const canvas = $('plot');
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== Math.round(width * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const css = getComputedStyle(document.documentElement);
  const color = (name) => css.getPropertyValue(name).trim();
  const left = 52, right = 8, top = 8, bottom = 22;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;

  // Scale around the middle of the data, not around 0: the oscillator moves around 0.1 m.
  let low = Infinity, high = -Infinity;
  for (const points of Object.values(series)) {
    for (const [, value] of points) {
      low = Math.min(low, value);
      high = Math.max(high, value);
    }
  }
  if (low > high) low = high = 0;
  const middle = (low + high) / 2;
  const half = Math.max(0.01, (high - low) / 2);
  const extent = half * 1.1;
  const x = (t) => left + (t - (serverTime - PLOT_SECONDS)) / PLOT_SECONDS * plotWidth;
  const y = (v) => top + (1 - (v - middle + extent) / (2 * extent)) * plotHeight;

  ctx.font = '11px system-ui, sans-serif';
  ctx.fillStyle = color('--muted');
  ctx.strokeStyle = color('--grid');
  ctx.lineWidth = 1;
  for (const fraction of [-1, -0.5, 0, 0.5, 1]) {
    const value = middle + fraction * half;
    ctx.beginPath();
    ctx.moveTo(left, y(value));
    ctx.lineTo(width - right, y(value));
    ctx.stroke();
    ctx.textAlign = 'right';
    // A middle a hair below 0 would print as -0.000.
    ctx.fillText((value + 0).toFixed(3).replace(/^-(0\.0+)$/, '$1'), left - 6, y(value) + 4);
  }
  for (let s = 0; s <= PLOT_SECONDS; s += 5) {
    ctx.textAlign = s === 0 ? 'left' : s === PLOT_SECONDS ? 'right' : 'center';
    ctx.fillText(s === PLOT_SECONDS ? 'now' : `-${PLOT_SECONDS - s} s`,
      left + s / PLOT_SECONDS * plotWidth, height - 6);
  }

  for (const [kind, name] of [['setpoint', '--setpoint'], ['position', '--position']]) {
    const points = series[kind];
    if (points.length < 2) continue;
    ctx.strokeStyle = color(name);
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach(([t, value], i) => {
      // Nothing is published while the paddle holds still; do not bridge those gaps.
      if (i === 0 || t - points[i - 1][0] > GAP_SECONDS) ctx.moveTo(x(t), y(value));
      else ctx.lineTo(x(t), y(value));
    });
    ctx.stroke();
  }
}

$('waveForm').addEventListener('submit', async (event) => {
  event.preventDefault();
  const height = parseFloat($('height').value);
  const period = parseFloat($('period').value);
  $('commandResult').textContent = 'Starting…';
  try {
    const result = await post('api/start', { height, period });
    $('commandResult').textContent = result.message;
  } catch (error) {
    $('commandResult').textContent = 'No answer from the server.';
  }
});

$('stop').addEventListener('click', async () => {
  try {
    const result = await post('api/stop');
    $('commandResult').textContent = result.message;
  } catch (error) {
    $('commandResult').textContent = 'No answer from the server.';
  }
});

$('return').addEventListener('click', async () => {
  $('return').disabled = true;
  $('returnResult').textContent = texts.returning;
  try {
    const result = await post('api/return');
    $('returnResult').textContent = result.message;
  } catch (error) {
    $('returnResult').textContent = 'No answer from the server.';
  }
  render();
});

$('keepRunning').addEventListener('click', async () => {
  try {
    const result = await post('api/keep_running');
    $('commandResult').textContent = result.message;
  } catch (error) {
    $('commandResult').textContent = 'No answer from the server.';
  }
});

$('logLevel').addEventListener('change', () => {
  $('log').dataset.min = $('logLevel').value;
  $('log').scrollTop = $('log').scrollHeight;
});

$('logClear').addEventListener('click', () => $('log').replaceChildren());

async function loadTexts() {
  try {
    const response = await fetch('api/config');
    texts = await response.json();
  } catch (error) {
    return;  // keep the defaults
  }
  document.title = texts.title;
  for (const [id, key] of [
    ['title', 'title'], ['waveCard', 'wave_card'], ['motionCard', 'motion_card'],
    ['plotNote', 'plot_note'], ['return', 'return_button'],
    ['heightLabel', 'height_label'],
  ]) {
    $(id).textContent = texts[key];
  }
  for (const [cls, key] of [['motion', 'motion'], ['a-motion', 'a_motion'], ['machine', 'machine']]) {
    for (const element of document.getElementsByClassName(cls)) element.textContent = texts[key];
  }
}

loadTexts();
render();
setInterval(render, 500);
window.addEventListener('resize', drawPlot);
connect();
