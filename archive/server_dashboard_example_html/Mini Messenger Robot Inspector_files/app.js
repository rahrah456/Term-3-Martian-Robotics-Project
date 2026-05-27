'use strict';

const TEAM_COLORS = [
  '#246b9f',
  '#317456',
  '#7c5cba',
  '#b05d2a',
  '#9f2f4c',
  '#007c7c',
  '#76621c',
  '#4f6f33',
  '#5f6aa3',
  '#8a4f7d'
];

const route = parseRoute();
const stateView = {
  connected: false,
  snapshot: null
};

let judgePassword = sessionStorage.getItem('judgePassword');

const overlayEl = document.getElementById('login-overlay');
const passwordInputEl = document.getElementById('password-input');
const loginErrorEl = document.getElementById('login-error');

// Expose login function to global scope for HTML onclick
window.login = login;

if (route.mode === 'base' && !judgePassword) {
  overlayEl.style.display = 'flex';
}

async function login() {
  const password = passwordInputEl.value;
  try {
    const response = await fetch('/api/auth/judge', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ password })
    });

    if (response.ok) {
      judgePassword = password;
      sessionStorage.setItem('judgePassword', password);
      overlayEl.style.display = 'none';
      await loadState();
    } else {
      loginErrorEl.textContent = 'Invalid security key.';
    }
  } catch (err) {
    loginErrorEl.textContent = 'Connection error.';
  }
}

const appTitleEl = document.getElementById('app-title');
const teamsEl = document.getElementById('teams');
const eventsEl = document.getElementById('events');
const gridEl = document.getElementById('grid');
const debugRowEl = document.getElementById('rfid-debug-row');
const mapTitleEl = document.getElementById('map-title');
const mapLegendEl = document.getElementById('map-legend');
const mapDetailsEl = document.getElementById('map-details');
const connectionPill = document.getElementById('connection-pill');
const lastUpdateEl = document.getElementById('last-update');
const clearRegistrationsButton = document.getElementById('clear-registrations');
const enableAllButton = document.getElementById('enable-all');
const emergencyStopButton = document.getElementById('emergency-stop');
const emergencyResetButton = document.getElementById('emergency-reset');
const overrideAButton = document.getElementById('override-airlock-a');
const overrideBButton = document.getElementById('override-airlock-b');
const resetPlantedButton = document.getElementById('reset-planted');
const resetExplorationButton = document.getElementById('reset-exploration');
const rerandomizeMapButton = document.getElementById('rerandomize-map');
const debugEls = {
  updated: document.getElementById('debug-updated'),
  tagId: document.getElementById('debug-tag-id'),
  robot: document.getElementById('debug-robot'),
  result: document.getElementById('debug-result'),
  cell: document.getElementById('debug-cell')
};

document.body.classList.add(route.mode === 'team' ? 'mode-team' : 'mode-base');
appTitleEl.textContent = route.mode === 'team' ? `Team ${route.teamId} Dashboard` : 'Base Dashboard';

if (clearRegistrationsButton) clearRegistrationsButton.addEventListener('click', clearRegistrations);
if (enableAllButton) enableAllButton.addEventListener('click', enableAllRobots);
if (emergencyStopButton) emergencyStopButton.addEventListener('click', () => setEmergency(true));
if (emergencyResetButton) emergencyResetButton.addEventListener('click', () => setEmergency(false));
if (overrideAButton) overrideAButton.addEventListener('click', () => overrideAirlock('A'));
if (overrideBButton) overrideBButton.addEventListener('click', () => overrideAirlock('B'));
if (resetPlantedButton) resetPlantedButton.addEventListener('click', resetMap);
if (resetExplorationButton) resetExplorationButton.addEventListener('click', resetFog);
if (rerandomizeMapButton) rerandomizeMapButton.addEventListener('click', rerandomizeMap);

connectWebSocket();
loadState();

setInterval(() => {
  if (!stateView.connected) {
    loadState();
  }
}, 5000);

async function overrideAirlock(airlock) {
  const response = await fetch(`/api/airlocks/override/${airlock}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

function handleAuthError() {
  judgePassword = null;
  sessionStorage.removeItem('judgePassword');
  if (route.mode === 'base') {
    overlayEl.style.display = 'flex';
  }
}

function parseRoute() {
  const parts = window.location.pathname.split('/').filter(Boolean);
  if (parts[0] === 'team' && parts[1]) {
    return {
      mode: 'team',
      teamId: decodeURIComponent(parts[1])
    };
  }
  return { mode: 'base', teamId: null };
}

function connectWebSocket() {
  const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new WebSocket(`${scheme}//${window.location.host}`);

  socket.addEventListener('open', () => {
    stateView.connected = true;
    render();
  });

  socket.addEventListener('message', (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'state') {
      stateView.snapshot = message.state;
      render();
    }
  });

  socket.addEventListener('close', () => {
    stateView.connected = false;
    render();
    window.setTimeout(connectWebSocket, 1500);
  });
}

async function loadState() {
  const response = await fetch('/api/state');
  stateView.snapshot = await response.json();
  render();
}

async function setRobotEnabled(teamId, boardId, enabled) {
  const response = await fetch(`/api/robots/${encodeURIComponent(teamId)}/${encodeURIComponent(boardId)}/disable`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled }) // No password needed for robot control
  });
  await loadState();
}

async function setTeamEnabled(teamId, enabled) {
  const response = await fetch(`/api/teams/${encodeURIComponent(teamId)}/disable`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled }) // No password needed for team control
  });
  await loadState();
}

async function enableAllRobots() {
  const response = await fetch('/api/robots/enable-all', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function clearRegistrations() {
  if (!confirm('Are you sure you want to clear all registered robots? They will need to re-register to be controlled.')) {
    return;
  }
  const response = await fetch('/api/registrations/clear', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function resetMap() {
  if (!confirm('Clear all planted seeds?')) return;
  const response = await fetch('/api/map/reset-planted', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function resetFog() {
  if (!confirm('Reset all exploration? Fog of war will be restored for all robots.')) return;
  const response = await fetch('/api/map/reset-exploration', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function rerandomizeMap() {
  if (!confirm('Rerandomize all fertile zones and reset discovery?')) return;
  const response = await fetch('/api/map/rerandomize', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function togglePlanted(tagId) {
  if (route.mode !== 'base') return;
  const response = await fetch('/api/map/toggle-planted', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ tagId, password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

async function setEmergency(enabled) {
  const response = await fetch('/api/emergency', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled, password: judgePassword })
  });
  if (response.status === 403) {
    handleAuthError();
    return;
  }
  await loadState();
}

function render() {
  const snapshot = stateView.snapshot;
  if (!snapshot) {
    connectionPill.textContent = 'Connecting';
    connectionPill.className = 'status-pill';
    return;
  }

  const visibleTeams = getVisibleTeams(snapshot);
  const visibleRobots = visibleTeams.flatMap((team) => team.robots);
  const registeredRobots = snapshot.teams.flatMap((team) => team.robots).filter((robot) => robot.registered);
  const teamEmergency = route.mode === 'team' && visibleTeams[0] ? visibleTeams[0].emergency : false;
  const emergencyActive = snapshot.globalEmergency || teamEmergency;
  const mqttConnected = snapshot.inspector.mqtt.connected;

  connectionPill.textContent = stateView.connected && mqttConnected ? 'Live' : mqttConnected ? 'MQTT only' : 'MQTT offline';
  connectionPill.className = `status-pill ${stateView.connected && mqttConnected ? 'online' : 'offline'}`;

  document.getElementById('summary-teams-label').textContent = route.mode === 'team' ? 'Team' : 'Teams';
  document.getElementById('summary-teams').textContent = route.mode === 'team' ? route.teamId : snapshot.teams.length;
  document.getElementById('summary-robots').textContent = visibleRobots.length;
  document.getElementById('summary-tags').textContent = snapshot.rfidTags.length;
  document.getElementById('summary-emergency').textContent = emergencyActive ? 'On' : 'Off';

  if (enableAllButton) enableAllButton.disabled = snapshot.globalEmergency || registeredRobots.length === 0;
  if (emergencyStopButton) emergencyStopButton.disabled = snapshot.globalEmergency;
  if (emergencyResetButton) emergencyResetButton.disabled = !snapshot.globalEmergency;
  lastUpdateEl.textContent = `Updated ${formatTime(snapshot.inspector.now)}`;

  renderMap(snapshot, visibleTeams);
  renderDebug(snapshot.debug);
  renderTeams(snapshot, visibleTeams);
  renderEvents(filterEvents(snapshot.events));
}

function getVisibleTeams(snapshot) {
  if (route.mode === 'team') {
    return snapshot.teams.filter((team) => team.teamId === route.teamId);
  }
  return snapshot.teams;
}

function renderMap(snapshot, teams) {
  const robots = teams.flatMap((team) => team.robots);
  mapTitleEl.textContent = route.mode === 'team' ? `Team ${route.teamId} RFID Map` : 'Shared RFID Map';
  renderLegend(teams);
  renderMapDetails(teams);
  renderGrid(gridEl, debugRowEl, snapshot.rfidTags, snapshot.debugRfidTags || [], robots);
}

function renderLegend(teams) {
  mapLegendEl.replaceChildren();
  for (const team of teams) {
    const item = document.createElement('span');
    item.className = 'legend-item';

    const swatch = document.createElement('span');
    swatch.className = 'legend-swatch';
    swatch.style.backgroundColor = teamColor(team.teamId);

    item.append(swatch, textSpan(`Team ${team.teamId}`));
    mapLegendEl.appendChild(item);
  }
}

function renderMapDetails(teams) {
  mapDetailsEl.replaceChildren();

  if (teams.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'empty-state compact';
    empty.textContent = route.mode === 'team' ? `Team ${route.teamId} has not registered yet.` : 'Waiting for robot registration or RFID traffic.';
    mapDetailsEl.appendChild(empty);
    return;
  }

  for (const team of teams) {
    const row = document.createElement('div');
    row.className = 'team-summary-row';

    const color = document.createElement('span');
    color.className = 'legend-swatch';
    color.style.backgroundColor = teamColor(team.teamId);

    const title = document.createElement('strong');
    title.textContent = `Team ${team.teamId}`;

    const meta = document.createElement('span');
    meta.textContent = `${team.robots.length} robots`;

    row.append(color, title, meta);

    if (route.mode === 'base') {
      const link = document.createElement('a');
      link.href = `/team/${encodeURIComponent(team.teamId)}`;
      link.textContent = 'Team page';
      row.appendChild(link);
    }

    mapDetailsEl.appendChild(row);
  }
}

function renderDebug(debug) {
  let latest = debug && debug.latestRfidScan;
  if (route.mode === 'team' && latest && latest.teamId !== route.teamId) {
    latest = null;
  }

  if (!latest) {
    debugEls.updated.textContent = route.mode === 'team' ? 'No team scans yet' : 'No scans yet';
    debugEls.tagId.textContent = 'None';
    debugEls.robot.textContent = 'None';
    debugEls.result.textContent = 'Waiting';
    debugEls.cell.textContent = 'Unknown';
    return;
  }

  debugEls.updated.textContent = `Scanned ${formatTime(latest.time)}`;
  debugEls.tagId.textContent = latest.tagId || 'Invalid';
  debugEls.robot.textContent = `Team ${latest.teamId} / Board ${latest.boardId}`;

  if (!latest.valid) {
    debugEls.result.textContent = 'Invalid tag';
  } else if (!latest.known) {
    debugEls.result.textContent = 'Unknown tag';
  } else {
    debugEls.result.textContent = latest.fertile ? 'Fertile' : 'Not fertile';
  }

  if (latest.known && latest.debug) {
    debugEls.cell.textContent = latest.debugLabel || `Debug ${latest.x}`;
  } else if (latest.x !== null && latest.x !== undefined && latest.y !== null && latest.y !== undefined) {
    debugEls.cell.textContent = `${latest.x},${latest.y}`;
  } else {
    debugEls.cell.textContent = 'Unmapped';
  }
}

function renderTeams(snapshot, teams) {
  teamsEl.replaceChildren();

  if (teams.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'empty-state';
    empty.textContent = route.mode === 'team' ? `No robots seen for Team ${route.teamId} yet.` : 'Waiting for robot registration or RFID traffic.';
    teamsEl.appendChild(empty);
    return;
  }

  for (const team of teams) {
    teamsEl.appendChild(teamPanel(team, snapshot.globalEmergency));
  }
}

function teamPanel(team, globalEmergency) {
  const panel = document.createElement('article');
  panel.className = `team-panel ${team.emergency ? 'emergency' : ''}`;

  const heading = document.createElement('div');
  heading.className = 'team-heading';

  const titleWrap = document.createElement('div');
  const eyebrow = document.createElement('p');
  eyebrow.className = 'eyebrow team-eyebrow';
  eyebrow.textContent = team.emergency ? 'Emergency active' : 'Team online';
  const title = document.createElement('h2');
  title.textContent = `Team ${team.teamId}`;
  titleWrap.append(eyebrow, title);

  const headingActions = document.createElement('div');
  headingActions.className = 'team-heading-actions';
  headingActions.appendChild(chip(`A ${team.airlocks.airlockABusy ? 'busy' : 'ready'} - enter queue ${team.airlocks.queueEnter}`, team.airlocks.airlockABusy));
  headingActions.appendChild(chip(`B ${team.airlocks.airlockBBusy ? 'busy' : 'ready'} - exit queue ${team.airlocks.queueExit}`, team.airlocks.airlockBBusy));

  if (route.mode === 'base') {
    headingActions.appendChild(teamLink(team.teamId));
    headingActions.appendChild(actionButton('Enable Team', 'secondary-button', globalEmergency || team.emergency || registeredRobots(team).length === 0, () => setTeamEnabled(team.teamId, true)));
    headingActions.appendChild(actionButton('Disable Team', 'danger-button', team.robots.length === 0, () => setTeamEnabled(team.teamId, false)));
  }

  heading.append(titleWrap, headingActions);

  const body = document.createElement('div');
  body.className = 'team-body';
  const robotList = document.createElement('div');
  robotList.className = 'robot-list';
  renderRobotList(robotList, team.robots, globalEmergency || team.emergency);
  body.appendChild(robotList);

  panel.append(heading, body);
  return panel;
}

function renderGrid(grid, debugRow, tags, debugTags, robots) {
  grid.replaceChildren();
  debugRow.replaceChildren();

  const tagsByCoord = new Map();
  for (const tag of tags) {
    tagsByCoord.set(`${tag.x}:${tag.y}`, tag);
  }

  const robotsByCoord = groupRobotsByCoord(robots, false);

  for (let y = 9; y >= 1; y -= 1) {
    for (let x = 1; x <= 9; x += 1) {
      const cell = document.createElement('div');
      const tag = tagsByCoord.get(`${x}:${y}`);
      const cellRobots = robotsByCoord.get(`${x}:${y}`) || [];
      
      const isUnexplored = tag && !tag.explored;
      cell.className = `cell ${tag ? tag.fertile ? 'fertile' : 'sterile' : ''} ${tag && tag.planted ? 'planted' : ''} ${isUnexplored ? 'unexplored' : ''}`;
      cell.setAttribute('role', 'gridcell');
      cell.title = tag ? `${tag.tagId} - ${tag.fertile ? 'fertile' : 'not fertile'}${tag.planted ? ' - planted' : ''}${isUnexplored ? ' - unexplored' : ''}` : `${x},${y}`;

      if (route.mode === 'base' && tag) {
        cell.style.cursor = 'pointer';
        cell.addEventListener('click', () => togglePlanted(tag.tagId));
      }

      if (tag && tag.planted) {
        const plant = document.createElement('span');
        plant.className = 'plant-icon';
        plant.textContent = '🌱';
        cell.appendChild(plant);
      }

      if (isUnexplored) {
        const fog = document.createElement('span');
        fog.className = 'fog-icon';
        fog.textContent = '❓';
        cell.appendChild(fog);
      }

      const coord = document.createElement('span');
      coord.className = 'cell-coord';
      coord.textContent = tag && tag.row && tag.col ? `R${tag.row} C${tag.col}` : `${x},${y}`;
      cell.appendChild(coord);

      // AIRLOCK MARKERS
      if (y === 9 && (x === 3 || x === 7)) {
        const airlockLabel = document.createElement('div');
        airlockLabel.className = 'airlock-indicator';
        airlockLabel.textContent = x === 3 ? 'AIRLOCK A' : 'AIRLOCK B';
        cell.appendChild(airlockLabel);
        cell.classList.add('airlock-cell');
      }

      appendRobotMarkers(cell, cellRobots);
      grid.appendChild(cell);
    }
  }

  renderDebugRow(debugRow, debugTags, groupRobotsByCoord(robots, true));
}

function renderDebugRow(debugRow, tags, robotsByCoord) {
  if (tags.length === 0) {
    debugRow.hidden = true;
    return;
  }

  debugRow.hidden = false;
  const label = document.createElement('div');
  label.className = 'rfid-debug-label';
  label.textContent = 'Debug Row';
  debugRow.appendChild(label);

  for (const tag of tags) {
    const cell = document.createElement('div');
    const cellRobots = robotsByCoord.get(`${tag.x}:${tag.y}`) || [];
    cell.className = `debug-cell ${tag.fertile ? 'fertile' : 'sterile'} ${!tag.explored ? 'unexplored' : ''}`;
    cell.title = `${tag.label || 'Debug'} - ${tag.tagId}`;

    const name = document.createElement('span');
    name.className = 'debug-cell-label';
    name.textContent = tag.label || `Debug ${tag.x}`;
    const uid = document.createElement('span');
    uid.className = 'debug-cell-id';
    uid.textContent = tag.tagId;
    cell.append(name, uid);
    appendRobotMarkers(cell, cellRobots);
    debugRow.appendChild(cell);
  }
}

function groupRobotsByCoord(robots, debugOnly) {
  const robotsByCoord = new Map();
  for (const robot of robots) {
    if (Boolean(robot.debug) !== debugOnly || robot.x === null || robot.x === undefined || robot.y === null || robot.y === undefined) {
      continue;
    }
    const key = `${robot.x}:${robot.y}`;
    if (!robotsByCoord.has(key)) {
      robotsByCoord.set(key, []);
    }
    robotsByCoord.get(key).push(robot);
  }
  return robotsByCoord;
}

function appendRobotMarkers(parent, robots) {
  for (const robot of robots.slice(0, 4)) {
    const marker = document.createElement('span');
    const isStranded = Boolean(robot.stranded);
    marker.className = `robot-marker ${robot.disabled || !robot.controlEnabled ? 'disabled' : ''} ${isStranded ? 'stranded' : ''}`;
    marker.style.setProperty('--team-color', teamColor(robot.teamId));
    marker.textContent = route.mode === 'team' ? `B${robot.boardId}` : `${robot.teamId}.${robot.boardId}`;
    marker.title = `Team ${robot.teamId} board ${robot.boardId} ${isStranded ? '(STRANDED)' : ''}`;
    parent.appendChild(marker);
  }

  if (robots.length > 4) {
    const more = document.createElement('span');
    more.className = 'robot-marker more';
    more.textContent = `+${robots.length - 4}`;
    more.title = `${robots.length - 4} more robots in this cell`;
    parent.appendChild(more);
  }
}

function renderRobotList(listEl, robots, emergencyActive) {
  listEl.replaceChildren();

  if (robots.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'empty-state';
    empty.textContent = 'No robots seen for this team yet.';
    listEl.appendChild(empty);
    return;
  }

  for (const robot of robots) {
    const controlEnabled = Boolean(robot.controlEnabled);
    const row = document.createElement('div');
    row.className = 'robot-row';

    const details = document.createElement('div');
    const title = document.createElement('div');
    title.className = 'robot-title';

    const swatch = document.createElement('span');
    swatch.className = 'legend-swatch';
    swatch.style.backgroundColor = teamColor(robot.teamId);

    title.append(swatch, textSpan(`Board ${robot.boardId}`), statusBadge(robot));

    const meta = document.createElement('div');
    meta.className = 'robot-meta';
    
    let timeRemainingText = '';
    if (robot.controlEnabled && robot.timerStart) {
      const elapsed = Date.now() - robot.timerStart;
      const remaining = Math.max(0, robot.sessionDuration - elapsed);
      const minutes = Math.floor(remaining / 60000);
      const seconds = Math.floor((remaining % 60000) / 1000);
      timeRemainingText = `Time ${minutes}:${seconds.toString().padStart(2, '0')}`;
    }

    meta.append(
      textSpan(`Position ${formatRobotPosition(robot)}`),
      textSpan(`Heading ${robot.heading || 'unknown'}`),
      textSpan(`Tag ${robot.lastTagId || robot.unknownTagId || 'none'}`),
      textSpan(`Control ${controlEnabled ? 'enabled' : 'off'}`),
      textSpan(timeRemainingText ? timeRemainingText : `Seen ${relativeTime(robot.lastSeen)}`)
    );

    if (robot.warning) {
      meta.appendChild(badge(robot.warning, 'warn'));
    }
    if (robot.unknownTagId) {
      meta.appendChild(badge('Unknown RFID', 'warn'));
    }

    details.append(title, meta);

    const actions = document.createElement('div');
    actions.className = 'robot-actions';
    const button = actionButton(
      controlEnabled ? 'Disable' : 'Enable',
      controlEnabled ? 'danger-button' : 'secondary-button',
      emergencyActive || (!controlEnabled && (!robot.registered || !robot.online || robot.stale)),
      () => setRobotEnabled(robot.teamId, robot.boardId, !controlEnabled)
    );
    actions.appendChild(button);

    row.append(details, actions);
    listEl.appendChild(row);
  }
}

function renderEvents(events) {
  eventsEl.replaceChildren();
  for (const event of events.slice(0, 20)) {
    const item = document.createElement('li');
    item.append(textSpan(formatTime(event.time)), textSpan(event.message));
    eventsEl.appendChild(item);
  }
}

function filterEvents(events) {
  if (route.mode === 'base') {
    return events;
  }

  return events.filter((event) => {
    const message = event.message || '';
    return message.includes(`Robot ${route.teamId}/`) || message.includes(`from ${route.teamId}/`) || message.includes(`Team ${route.teamId}`) || message.includes('Emergency');
  });
}

function teamLink(teamId) {
  const link = document.createElement('a');
  link.className = 'team-page-link';
  link.href = `/team/${encodeURIComponent(teamId)}`;
  link.textContent = 'Team page';
  return link;
}

function actionButton(label, className, disabled, onClick) {
  const button = document.createElement('button');
  button.className = className;
  button.type = 'button';
  button.disabled = disabled;
  button.textContent = label;
  button.addEventListener('click', onClick);
  return button;
}

function chip(label, busy) {
  const el = document.createElement('span');
  el.className = `airlock-chip ${busy ? 'busy' : ''}`;
  el.textContent = label;
  return el;
}

function statusBadge(robot) {
  if (robot.stranded) {
    return badge('Stranded', 'danger');
  }
  if (!robot.registered) {
    return badge('Unregistered', 'warn');
  }
  if (!robot.online || robot.stale) {
    return badge('Stale', 'warn');
  }
  if (robot.controlEnabled && !robot.disabled) {
    return badge('Enabled', 'good');
  }
  if (robot.disabled) {
    return badge('Disabled', 'danger');
  }
  if (!robot.controlEnabled) {
    return badge('Standby', 'warn');
  }
  return badge('Ready', 'good');
}

function badge(label, tone) {
  const el = document.createElement('span');
  el.className = `badge ${tone || ''}`;
  el.textContent = label;
  return el;
}

function textSpan(text) {
  const el = document.createElement('span');
  el.textContent = text;
  return el;
}

function registeredRobots(team) {
  return team.robots.filter((robot) => robot.registered);
}

function teamColor(teamId) {
  const text = String(teamId);
  let hash = 0;
  for (let index = 0; index < text.length; index += 1) {
    hash = (hash * 31 + text.charCodeAt(index)) >>> 0;
  }
  return TEAM_COLORS[hash % TEAM_COLORS.length];
}

function formatTime(ms) {
  return new Date(ms).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

function relativeTime(ms) {
  if (!ms) {
    return 'never';
  }
  const seconds = Math.max(0, Math.round((Date.now() - ms) / 1000));
  if (seconds < 2) {
    return 'now';
  }
  if (seconds < 60) {
    return `${seconds}s ago`;
  }
  return `${Math.round(seconds / 60)}m ago`;
}

function formatRobotPosition(robot) {
  if (robot.debug) {
    return robot.debugLabel || `Debug ${robot.x}`;
  }
  if (robot.x !== null && robot.x !== undefined && robot.y !== null && robot.y !== undefined) {
    return `${robot.x},${robot.y}`;
  }
  return 'unknown';
}
