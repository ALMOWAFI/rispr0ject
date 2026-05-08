function playSound(file) {
  const audio = new Audio(file);
  audio.volume = 0.7;
  audio.play();
}
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
const playAgainWrap = document.getElementById('play-again-wrap');
const observerPanel = document.getElementById('observer-panel');
const observerToggle = document.getElementById('observer-toggle');
const observerClose = document.getElementById('observer-close');

let lastGameState = null;
let lastScore = null;

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

function setButtonHandlers() {
  document.getElementById('start-btn').addEventListener('click', async () => {

   playSound('sounds/Game_Start.mp3');


    try {
      const result = await postJson('/api/start');
      previewMessage.textContent = result.message;
      pollStatus();
    } catch (error) {
      previewMessage.textContent = `Start failed: ${error.message}`;
    }
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

  document.getElementById('play-again-btn').addEventListener('click', async () => {
    try {
      await postJson('/api/restart_game');
      pollStatus();
    } catch (error) {
      console.error('Restart failed:', error.message);
    }
  });
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
  const valid = parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1]);

  if (isPlayerTurn && valid) {
    const current = parseInt(parts[0], 10);
    const total = parseInt(parts[1], 10);
    stepProgress.textContent = `Step ${current} of ${total}`;
    stepProgress.hidden = false;
  } else {
    stepProgress.hidden = true;
  }
}

function renderPlayAgain(gameState) {
  const show = gameState === 'GAME_OVER' || gameState === 'MOTION_FAILED';
  playAgainWrap.hidden = !show;
}

function renderStatus(data) {

  const currentState = data.game_state || 'UNKNOWN';
  const currentScore = data.score;

  if (lastGameState !== null && currentState !== lastGameState) {
    if (currentState === 'GAME_OVER' || currentState === 'MOTION_FAILED') {
      playSound('sounds/Game_Over.mp3');
    }
  }

  if (lastScore !== null && currentScore !== null && currentScore > lastScore) {
    playSound('sounds/Level_Up.mp3');
  }

  lastGameState = currentState;
  lastScore = currentScore;

  scoreValue.textContent = data.score === null ? '-' : String(data.score);
  blockCountValue.textContent = String(data.detected_block_count || 0);
  stateValue.textContent = data.game_state || 'UNKNOWN';
  motionValue.textContent = data.motion_status || 'UNKNOWN';
  bridgeStatus.textContent = data.bridge_status || 'UNKNOWN';
  bridgeMessage.textContent = data.bridge_message || 'No bridge message.';

  renderPlayerMessage(data);
  renderSelection(data.player_selection);
  renderProgress(data.game_state, data.player_progress || '');
  renderPlayAgain(data.game_state);

  statusRibbon.textContent = data.last_update
    ? `Last system update: ${data.last_update}`
    : 'Waiting for live ROS data.';

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
