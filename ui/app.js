class SoundManager {
  constructor() {
    const AudioCtx = window.AudioContext || window.webkitAudioContext;
    this.ctx = AudioCtx ? new AudioCtx() : null;
    this.masterGain = this.ctx ? this.ctx.createGain() : null;
    if (this.masterGain) {
      this.masterGain.gain.value = 0.35;
      this.masterGain.connect(this.ctx.destination);
    }
  }

  async unlock() {
    if (this.ctx && this.ctx.state === 'suspended') {
      await this.ctx.resume();
    }
  }

  beep(freq = 440, duration = 0.1, type = 'sine', volume = 0.16) {
    if (!this.ctx || !this.masterGain) {
      return;
    }

    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    const now = this.ctx.currentTime;

    osc.type = type;
    osc.frequency.setValueAtTime(freq, now);
    gain.gain.setValueAtTime(volume, now);
    gain.gain.exponentialRampToValueAtTime(0.0001, now + duration);

    osc.connect(gain);
    gain.connect(this.masterGain);
    osc.start(now);
    osc.stop(now + duration);
  }

  chime(up = true) {
    if (!this.ctx || !this.masterGain) {
      return;
    }

    const base = up ? 440 : 660;
    const ratios = up ? [1.0, 1.33, 1.66] : [1.0, 0.76, 0.58];
    ratios.forEach((ratio, index) => {
      window.setTimeout(() => {
        this.beep(base * ratio, 0.22, 'triangle', 0.12);
      }, index * 110);
    });
  }

  success() {
    this.beep(880, 0.08, 'triangle', 0.14);
    window.setTimeout(() => this.beep(1320, 0.16, 'triangle', 0.12), 100);
  }

  error() {
    this.beep(260, 0.24, 'sawtooth', 0.14);
    window.setTimeout(() => this.beep(180, 0.28, 'sawtooth', 0.12), 120);
  }
}

class SpeechManager {
  constructor() {
    this.synth = window.speechSynthesis || null;
    this.voice = null;
    this.enabled = false;
    this.setVoice = this.setVoice.bind(this);
    this.setVoice();
    if (this.synth && typeof this.synth.onvoiceschanged !== 'undefined') {
      this.synth.onvoiceschanged = this.setVoice;
    }
  }

  setVoice() {
    if (!this.synth) {
      return;
    }
    const voices = this.synth.getVoices();
    this.voice = voices.find((v) => v.name.includes('Google') || v.name.includes('Natural')) || voices[0] || null;
  }

  enable() {
    this.enabled = true;
  }

  say(text) {
    if (!this.enabled || !this.synth || !text) {
      return;
    }
    this.synth.cancel();
    const utterance = new SpeechSynthesisUtterance(text);
    if (this.voice) {
      utterance.voice = this.voice;
    }
    utterance.pitch = 0.95;
    utterance.rate = 1.0;
    this.synth.speak(utterance);
  }
}

const START_SOUND_PATH = 'sounds/Game_Start.mp3';

const sounds = new SoundManager();
const speech = new SpeechManager();

const scoreValue = document.getElementById('score-value');
const blockCountValue = document.getElementById('block-count-value');
const stateValue = document.getElementById('state-value');
const motionValue = document.getElementById('motion-value');
const selectionValue = document.getElementById('selection-value');
const selectionPlayerValue = document.getElementById('selection-player-value');
const statusRibbon = document.getElementById('status-ribbon');
const previewMessage = document.getElementById('preview-message');
const bridgeStatus = document.getElementById('bridge-status');
const bridgeMessage = document.getElementById('bridge-message');
const blockList = document.getElementById('block-list');
const processList = document.getElementById('process-list');
const logList = document.getElementById('log-list');
const playerStage = document.getElementById('player-stage');
const playerPhase = document.getElementById('player-phase');
const playerHeadline = document.getElementById('player-headline');
const playerDetail = document.getElementById('player-detail');
const stepProgress = document.getElementById('step-progress');
const primaryActionWrap = document.getElementById('play-again-wrap');
const primaryActionButton = document.getElementById('play-again-btn');
const observerPanel = document.getElementById('observer-panel');
const observerToggle = document.getElementById('observer-toggle');
const observerClose = document.getElementById('observer-close');

let audioArmed = false;
let lastGameState = null;

async function armExperience() {
  if (!audioArmed) {
    audioArmed = true;
    speech.enable();
  }
  await sounds.unlock();
}

async function playStartSound() {
  try {
    await armExperience();
    const audio = new Audio(START_SOUND_PATH);
    audio.volume = 0.7;
    await audio.play();
  } catch (error) {
    console.debug('Start sound blocked:', error);
  }
}

async function postJson(url) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: '{}'
  });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.message || `HTTP ${response.status}`);
  }
  return data;
}

function setObserverOpen(open) {
  document.body.classList.toggle('observer-open', open);
  observerPanel.setAttribute('aria-hidden', String(!open));
  observerToggle.setAttribute('aria-expanded', String(open));
}

function setObserverHandlers() {
  observerToggle.addEventListener('click', () => {
    const shouldOpen = observerPanel.getAttribute('aria-hidden') === 'true';
    setObserverOpen(shouldOpen);
  });

  observerClose.addEventListener('click', () => setObserverOpen(false));

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') {
      setObserverOpen(false);
    }
  });
}

async function startSession() {
  await playStartSound();
  try {
    const result = await postJson('/api/start');
    previewMessage.textContent = result.message;
    pollStatus();
  } catch (error) {
    previewMessage.textContent = `Start failed: ${error.message}`;
  }
}

async function restartGame() {
  try {
    await armExperience();
    sounds.chime(true);
    speech.say('Starting a new round.');
    const result = await postJson('/api/restart_game');
    previewMessage.textContent = result.message;
    pollStatus();
  } catch (error) {
    previewMessage.textContent = `Restart failed: ${error.message}`;
  }
}

function setButtonHandlers() {
  document.getElementById('start-btn').addEventListener('click', () => {
    startSession();
  });

  document.getElementById('end-btn').addEventListener('click', async () => {
    try {
      const result = await postJson('/api/stop');
      previewMessage.textContent = result.message;
      pollStatus();
    } catch (error) {
      previewMessage.textContent = `Stop failed: ${error.message}`;
    }
  });

  primaryActionButton.addEventListener('click', async () => {
    const action = primaryActionButton.dataset.action;
    if (action === 'start') {
      await startSession();
      return;
    }
    if (action === 'restart') {
      await restartGame();
    }
  });
}

function hasRunningSession(processStatus) {
  return Object.values(processStatus || {}).some((info) => Boolean(info && info.running));
}

function renderBlocks(blocks) {
  blockList.innerHTML = '';
  if (!blocks || blocks.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No detected blocks.';
    blockList.appendChild(item);
    return;
  }

  blocks.forEach((block) => {
    const item = document.createElement('li');
    item.innerHTML = `<strong>${block.color}</strong> · id ${block.id} · (${block.position.x}, ${block.position.y}, ${block.position.z}) · conf ${block.confidence}`;
    blockList.appendChild(item);
  });
}

function renderProcesses(processStatus) {
  processList.innerHTML = '';
  const entries = Object.entries(processStatus || {});
  if (entries.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No managed process data.';
    processList.appendChild(item);
    return;
  }

  entries.forEach(([name, info]) => {
    const item = document.createElement('li');
    const label = info.label || name;
    item.innerHTML = `<strong>${label}</strong> · ${info.running ? 'running' : 'stopped'}${info.pid ? ` · pid ${info.pid}` : ''}`;
    processList.appendChild(item);
  });
}

function renderLog(entries) {
  logList.innerHTML = '';
  if (!entries || entries.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No bridge log entries.';
    logList.appendChild(item);
    return;
  }

  entries.forEach((entry) => {
    const item = document.createElement('li');
    item.textContent = `${entry.time} - ${entry.message}`;
    logList.appendChild(item);
  });
}

function renderPlayerMessage(data) {
  if (data.session_starting) {
    playerPhase.textContent = 'Starting';
    playerHeadline.textContent = 'Starting the system';
    playerDetail.textContent = data.bridge_message || 'Launching camera, vision, selection, motion, and game.';
    playerStage.dataset.tone = 'active';
    return;
  }

  const message = data.player_message || {
    phase: 'Connecting',
    headline: 'Connecting to the game',
    detail: 'Waiting for live ROS data from the robot system.',
    tone: 'neutral'
  };

  playerPhase.textContent = message.phase;
  playerHeadline.textContent = message.headline;
  playerDetail.textContent = message.detail;
  playerStage.dataset.tone = message.tone || 'neutral';
}

function renderSelection(selection) {
  if (selection) {
    const summary = `${selection.color} · id ${selection.block_id} · ${selection.selection_type} · conf ${selection.confidence}`;
    selectionValue.textContent = summary;
    selectionPlayerValue.textContent = `${selection.color} · id ${selection.block_id}`;
  } else {
    selectionValue.textContent = 'None';
    selectionPlayerValue.textContent = 'None';
  }
}

function renderProgress(gameState, progress) {
  const isPlayerTurn = gameState === 'WAITING_PLAYER';
  const parts = progress ? progress.split('/') : [];
  const valid = parts.length === 2 && !Number.isNaN(Number(parts[0])) && !Number.isNaN(Number(parts[1]));

  if (isPlayerTurn && valid) {
    const completed = parseInt(parts[0], 10);
    const total = parseInt(parts[1], 10);
    if (total > 0) {
      const nextMove = Math.min(total, completed + 1);
      stepProgress.textContent = completed >= total ? 'Sequence complete' : `Move ${nextMove} of ${total}`;
      stepProgress.hidden = false;
      return;
    }
  }

  stepProgress.hidden = true;
}

function renderPrimaryAction(data) {
  const running = hasRunningSession(data.process_status);
  const starting = Boolean(data.session_starting);
  const canRestart = running && (data.game_state === 'GAME_OVER' || data.game_state === 'MOTION_FAILED');

  if (starting) {
    primaryActionWrap.hidden = false;
    primaryActionButton.hidden = false;
    primaryActionButton.disabled = true;
    primaryActionButton.dataset.action = 'starting';
    primaryActionButton.textContent = 'Starting...';
    return;
  }

  if (!running) {
    primaryActionWrap.hidden = false;
    primaryActionButton.hidden = false;
    primaryActionButton.disabled = false;
    primaryActionButton.dataset.action = 'start';
    primaryActionButton.textContent = 'Start Game';
    return;
  }

  if (canRestart) {
    primaryActionWrap.hidden = false;
    primaryActionButton.hidden = false;
    primaryActionButton.disabled = false;
    primaryActionButton.dataset.action = 'restart';
    primaryActionButton.textContent = 'Play Again';
    return;
  }

  primaryActionWrap.hidden = true;
  primaryActionButton.dataset.action = '';
  primaryActionButton.disabled = false;
}

function handleStateTransition(previousState, nextState, data) {
  if (!audioArmed || previousState === nextState) {
    return;
  }

  switch (nextState) {
    case 'SHOWING_SEQUENCE':
      sounds.chime(true);
      speech.say('Watch the sequence.');
      break;
    case 'WAITING_PLAYER':
      sounds.beep(880, 0.18, 'triangle', 0.14);
      speech.say('Your turn.');
      break;
    case 'ROUND_PAUSE':
      sounds.success();
      speech.say('Correct.');
      break;
    case 'GAME_OVER':
      sounds.chime(false);
      speech.say(`Game over. Final score ${data.score ?? 0}.`);
      break;
    case 'MOTION_FAILED':
      sounds.error();
      speech.say('Motion needs attention.');
      break;
    case 'IDLE':
      if (previousState && previousState !== 'UNKNOWN') {
        speech.say('System ready.');
      }
      break;
    default:
      break;
  }
}

function renderStatus(data) {
  const currentState = data.game_state || 'UNKNOWN';

  if (lastGameState !== null && currentState !== lastGameState) {
    handleStateTransition(lastGameState, currentState, data);
  }
  lastGameState = currentState;

  scoreValue.textContent = data.score === null ? '-' : String(data.score);
  blockCountValue.textContent = String(data.detected_block_count || 0);
  stateValue.textContent = currentState;
  motionValue.textContent = data.motion_status || 'UNKNOWN';
  bridgeStatus.textContent = data.bridge_status || 'UNKNOWN';
  bridgeMessage.textContent = data.bridge_message || 'No bridge message.';

  renderPlayerMessage(data);
  renderSelection(data.player_selection);
  renderProgress(currentState, data.player_progress || '');
  renderPrimaryAction(data);

  if (data.session_starting) {
    statusRibbon.textContent = 'Session startup is in progress.';
  } else {
    statusRibbon.textContent = data.last_update
      ? `Last system update: ${data.last_update}`
      : 'Waiting for live ROS data.';
  }

  renderBlocks(data.detected_blocks || []);
  renderProcesses(data.process_status || {});
  renderLog(data.log || []);
}

async function pollStatus() {
  try {
    const response = await fetch('/api/status', { cache: 'no-store' });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const data = await response.json();
    renderStatus(data);
  } catch (error) {
    bridgeStatus.textContent = 'disconnected';
    bridgeMessage.textContent = `Could not read /api/status: ${error.message}`;
    statusRibbon.textContent = 'UI is up, but the bridge endpoint is not responding.';
  }
}

setObserverHandlers();
setButtonHandlers();
pollStatus();
setInterval(pollStatus, 1000);
