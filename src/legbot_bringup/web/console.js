const byId = (id) => document.getElementById(id);
const canvas = byId('map');
const ctx = canvas.getContext('2d');
const atlas = await fetch('/api/map').then((response) => response.json());
const [cols, rows] = atlas.shape;
const resolution = atlas.resolution;
const floors = atlas.floors.map((floor, index) => {
  byId('floor').add(new Option(floor.name, index));
  const cells = Uint8Array.from(atob(floor.pixels), (character) => character.charCodeAt(0));
  const bytes = Uint8Array.from(atob(floor.elevation_cm || ''), character => character.charCodeAt(0));
  const elevation = bytes.length === 2 * cols * rows ? new DataView(bytes.buffer) : null;
  return { ...floor, cells, elevation };
});
function makeLimitsImage(cells, showSurface) {
  const image = document.createElement('canvas');
  image.width = cols;
  image.height = rows;
  const context = image.getContext('2d');
  const pixels = context.createImageData(cols, rows);
  for (let column = 0; column < cols; column++) {
    for (let row = 0; row < rows; row++) {
      const value = cells[column * rows + row];
      const offset = 4 * ((rows - row - 1) * cols + column);
      if (value === 1) pixels.data.set([193, 115, 60, 105], offset);
      if (showSurface && value === 2) pixels.data.set([249, 252, 248, 255], offset);
    }
  }
  context.putImageData(pixels, 0, 0);
  return image;
}
async function loadStructures() {
  const bundle = JSON.parse(byId('structureData').textContent);
  if (bundle.resolution !== resolution || bundle.origin.some((value, i) => value !== atlas.origin[i]) ||
      floors.length !== bundle.heights.length) return null;
  for (const [index, floor] of floors.entries()) {
    const digest = new Uint8Array(await crypto.subtle.digest('SHA-256', floor.cells));
    const checksum = Array.from(digest, byte => byte.toString(16).padStart(2, '0')).join('');
    if (floor.height !== bundle.heights[index] || checksum !== bundle.checksums[index]) return null;
  }
  const sheet = new Image();
  sheet.src = 'data:image/png;base64,' + bundle.png;
  await sheet.decode();
  return sheet.width === cols && sheet.height === rows * floors.length ? { sheet, stairs: bundle.stairs || [] } : null;
}
const structures = await loadStructures().catch(() => null);
const stairLanes = floors.map((_, floor) => {
  const stairs = (structures?.stairs || []).filter(stair => stair.floor === floor || stair.to === floor);
  return stairs.map(stair => {
    const group = stairs.filter(other => Math.abs(other.x - stair.x) < (stair.width + other.width) / 2 &&
      Math.max(Math.min(other.start, other.end), Math.min(stair.start, stair.end)) <
      Math.min(Math.max(other.start, other.end), Math.max(stair.start, stair.end)));
    const index = group.indexOf(stair), count = group.length, width = stair.width / count;
    return {stair, index, count, width, left: stair.x - stair.width / 2 + index * width};
  });
});
const limitsImages = floors.map((floor, floorIndex) => {
  const cells = floor.cells.slice();
  for (let column = 0; column < cols; column++) {
    for (let row = 0; row < rows; row++) {
      const x = atlas.origin[0] + column * resolution, y = atlas.origin[1] + row * resolution;
      const surfaces = targetSurfaces(x, y, floorIndex);
      if (surfaces[0].stair) cells[column * rows + row] = surfaces.some(surface => surface.value === 2) ? 2 : 1;
    }
  }
  return makeLimitsImage(cells, !structures);
});
const limits = byId('limits');
limits.checked = !structures;
limits.disabled = !structures;
byId('structureLegend').hidden = !structures;
byId('limitLegend').hidden = !limits.checked;
limits.onchange = () => {
  byId('limitLegend').hidden = !limits.checked;
  draw();
};
let state = {},
  online = false,
  busy = false,
  mode = null,
  tool = 'browse';
let pendingTask = null, cancelRequested = null, previousFix;
let database, records = [], libraryKind = 'place', editingRecord = null, replaceIndex = null;
const stairPicker = byId('stairPicker');
let draft = [],
  following = false,
  drag = null,
  scale = 1,
  center = [0, 0],
  listKey = '';
const terminal = ['idle', 'succeeded', 'failed', 'canceled'];
const active = () => state.task && !terminal.includes(state.task.phase);
const currentTask = () => active() ? state.task.id : pendingTask;
const localizedPose = () => online && previousFix === undefined && state.localization?.state === 'LOCALIZED' ? state.pose : null;
const selected = () => Number(byId('floor').value);
const nearest = (z) =>
  floors.reduce(
    (best, f, i) => (Math.abs(f.height - z) < Math.abs(floors[best].height - z) ? i : best),
    0,
  );
const floorName = (z) => floors[nearest(z)].name;
const world = (p) => [
  center[0] + (p[0] - canvas.clientWidth / 2) / scale,
  center[1] - (p[1] - canvas.clientHeight / 2) / scale,
];
const screen = (x, y) => [
  (x - center[0]) * scale + canvas.clientWidth / 2,
  (center[1] - y) * scale + canvas.clientHeight / 2,
];
function fit() {
  center = [
    atlas.origin[0] + ((cols - 1) * resolution) / 2,
    atlas.origin[1] + ((rows - 1) * resolution) / 2,
  ];
  scale = Math.min(
    canvas.clientWidth / ((cols + 10) * resolution),
    canvas.clientHeight / ((rows + 10) * resolution),
  );
  draw();
}
function follow(value) {
  following = value;
  byId('follow').setAttribute('aria-pressed', value);
  if (value && localizedPose()) {
    center = state.pose.slice(0, 2);
    byId('floor').value = nearest(state.pose[2] - 0.32);
  }
  draw();
}
function arrow(pose, ground, color) {
  if (!pose || nearest(pose[2] - ground) !== selected()) return;
  ctx.save();
  ctx.translate(...screen(...pose));
  ctx.rotate(-pose[3]);
  ctx.fillStyle = color;
  ctx.strokeStyle = 'white';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-8, -7);
  ctx.lineTo(-5, 0);
  ctx.lineTo(-8, 7);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}
function drawStairs() {
  for (const lane of stairLanes[selected()]) drawStair(lane);
}
function drawStair({stair, index, count, left: laneLeft, width: laneWidth}) {
  const outgoing = stair.floor === selected();
  const [left, bottom] = screen(laneLeft, stair.start);
  const [, top] = screen(stair.x, stair.end);
  const width = laneWidth * scale;
  const cx = left + width / 2;
  const end = outgoing ? top : bottom, start = outgoing ? bottom : top;
  const mid = (start + end) / 2 + (index - (count - 1) / 2) * Math.min(54, Math.abs(end - start) / 3);
  ctx.save();
  ctx.strokeStyle = '#557571';
  ctx.fillStyle = 'rgba(169, 198, 190, .32)';
  ctx.fillRect(left, bottom, width, top - bottom);
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let step = 0; step <= stair.count; step++) {
    const y = bottom + (top - bottom) * step / stair.count;
    ctx.moveTo(left, y);
    ctx.lineTo(left + width, y);
  }
  ctx.stroke();
  ctx.strokeStyle = '#235a53';
  ctx.lineWidth = 2.5;
  const tip = mid + .20 * (end - start);
  const barb = Math.sign(end - start) * Math.min(9, width / 4);
  ctx.beginPath();
  ctx.moveTo(cx, mid - .20 * (end - start));
  ctx.lineTo(cx, tip);
  ctx.moveTo(cx - Math.abs(barb) * .65, tip - barb);
  ctx.lineTo(cx, tip);
  ctx.lineTo(cx + Math.abs(barb) * .65, tip - barb);
  ctx.stroke();
  const text = (outgoing ? '上行 · ' : '下行 · ') + floors[outgoing ? stair.to : stair.floor].name;
  ctx.font = '600 12px sans-serif';
  const size = ctx.measureText(text).width;
  const labelX = left + width + size + 16 < canvas.clientWidth ? left + width + 5 : left - size - 14;
  ctx.fillStyle = 'rgba(255,255,255,.94)';
  ctx.fillRect(labelX, mid - 11, size + 10, 22);
  ctx.fillStyle = '#234c47';
  ctx.textAlign = 'left';
  ctx.fillText(text, labelX + 5, mid + 4);
  ctx.restore();
}
function draw() {
  const width = canvas.clientWidth,
    height = canvas.clientHeight;
  if (!width || !height) return;
  const ratio = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.imageSmoothingEnabled = false;
  const top = screen(atlas.origin[0] - resolution / 2, atlas.origin[1] + (rows - 0.5) * resolution);
  const destination = [...top, cols * resolution * scale, rows * resolution * scale];
  if (structures) ctx.drawImage(structures.sheet, 0, selected() * rows, cols, rows, ...destination);
  if (limits.checked) ctx.drawImage(limitsImages[selected()], ...destination);
  drawStairs();
  ctx.beginPath();
  ctx.lineWidth = 3;
  ctx.strokeStyle = '#cd7024';
  let connected = false;
  let lastVisible = null;
  const transfers = [];
  for (const point of state.route || []) {
    const routeFloor = nearest(point[2] - 0.5);
    if (routeFloor !== selected()) {
      if (connected) transfers.push({position: lastVisible, floor: routeFloor});
      connected = false;
      continue;
    }
    if (connected) ctx.lineTo(...screen(...point));
    else ctx.moveTo(...screen(...point));
    connected = true;
    lastVisible = point;
  }
  ctx.stroke();
  ctx.save();
  ctx.font = '600 12px sans-serif';
  ctx.textAlign = 'left';
  for (const transfer of transfers) {
    const [x, y] = screen(...transfer.position);
    const text = `路线 → ${floors[transfer.floor].name}`;
    const offset = x + ctx.measureText(text).width + 12 < width ? 7 : -ctx.measureText(text).width - 7;
    ctx.lineWidth = 3;
    ctx.strokeStyle = 'white';
    ctx.strokeText(text, x + offset, y - 8);
    ctx.fillStyle = '#974a19';
    ctx.fillText(text, x + offset, y - 8);
  }
  ctx.restore();
  const showTask = active();
  const points = showTask
    ? state.task.points.map((point, i) => storedPoint(point, state.task.headings?.[i]))
    : draft;
  points.forEach((p, i) => {
    if (p.stair ? ![p.stair.floor, p.stair.to].includes(selected()) : selected() !== p.floor) return;
    const [x, y] = screen(p.x, p.y);
    ctx.fillStyle = showTask && i < state.task.index ? '#597b6e' : '#b75e22';
    if (Number.isFinite(p.yaw)) {
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-p.yaw);
      ctx.beginPath();
      ctx.moveTo(7, -8);
      ctx.lineTo(26, 0);
      ctx.lineTo(7, 8);
      ctx.fill();
      ctx.restore();
    }
    ctx.beginPath();
    ctx.arc(x, y, 11, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(i + 1, x, y + 4);
  });
  arrow(localizedPose(), 0.32, '#006e62');
  if (drag && drag.kind !== 'pan') {
    ctx.beginPath();
    ctx.moveTo(...drag.start);
    ctx.lineTo(...drag.end);
    ctx.strokeStyle = '#006e62';
    ctx.lineWidth = 3;
    ctx.stroke();
  }
}
const labels = {
  WAITING_FOR_INITIAL_POSE: '等待给定初始位姿',
  COLLECTING: '采集静止点云',
  MATCHING: '正在匹配地图',
  LOCALIZED: '定位成功',
  FAILED: '定位失败',
  LOST: '定位丢失',
};
const reasons = {
  AMBIGUOUS_FLOOR: '多个楼层结构相似，请在楼梯口或转角重试',
  LOW_OVERLAP: '点云重合不足，请调整位置和朝向',
  MAP_OUT_OF_COVERAGE: '初值超出地图范围',
  FREE_SPACE_CONFLICT: '雷达射线与地图冲突',
  AMBIGUOUS: '存在多个可信位置',
  DEGENERATE_GEOMETRY: '当前观测缺少几何约束',
  SEARCH_BOUNDARY: '估计修正超出初值允许范围',
  VALIDATION_FAILED: '独立点云复核未通过',
  INSUFFICIENT_POINTS: '有效点云不足',
  HIGH_RESIDUAL: '点云与地图对齐后的误差过大',
  LOW_STRUCTURAL_SUPPORT: '可相互印证的建筑结构不足',
};
const phases = {
  idle: '等待任务',
  planning: '规划中',
  walking: '正在前往',
  at_waypoint: '已站稳，准备下一段',
  stopping: '正在确认停车',
  canceled: '任务已取消',
  failed: '任务失败',
  succeeded: '全部到点',
};
function button(text, action, label = text) {
  const element = document.createElement('button');
  element.textContent = text;
  element.title = label;
  element.setAttribute('aria-label', label);
  element.onclick = action;
  return element;
}
function row(parent, text) {
  const item = document.createElement('li'),
    label = document.createElement('span');
  label.textContent = text;
  item.append(label);
  parent.append(item);
  return item;
}
function stopRow(list, index, name, description) {
  const item = document.createElement('li');
  const number = document.createElement('b');
  number.className = 'stop-index';
  number.textContent = String(index + 1).padStart(2, '0');
  const copy = document.createElement('div'), title = document.createElement('strong'), meta = document.createElement('small');
  copy.className = 'row-copy';
  title.textContent = name;
  meta.className = 'meta';
  meta.textContent = description;
  copy.append(title, meta);
  item.append(number, copy);
  list.append(item);
  return item;
}
function pointTitle(point) {
  return point.stair ? stairName(point.stair) : floors[point.floor].name;
}
function update() {
  const awaitingTask = pendingTask && state.task?.id !== pendingTask && !active();
  const task = awaitingTask ? {phase: 'planning', id: pendingTask, points: [], index: 0} :
    state.task || { phase: 'idle', points: [], index: 0 };
  const taskId = currentTask();
  const locked = !!taskId || busy;
  const loc = previousFix === undefined ? (state.localization || {}) : {state: 'COLLECTING'};
  byId('link').textContent = online ? '已连接' : '连接中断';
  byId('link').dataset.online = online;
  byId('loc').dataset.ok = !!localizedPose();
  byId('loc').textContent =
    (labels[loc.state] || '等待机器人') +
    (['FAILED', 'LOST'].includes(loc.state) ? ' · ' + (reasons[loc.reason] || loc.reason) : '');
  const waitingHint = loc.state === 'WAITING_FOR_INITIAL_POSE'
    ? '默认仿真出生点在入口层 (−5.5, 6.0) m，朝地图右方。'
    : '等待有效地图位姿';
  byId('pose').textContent = localizedPose()
    ? `${floorName(state.pose[2] - 0.32)} · (${state.pose[0].toFixed(2)}, ${state.pose[1].toFixed(2)}) m · ${degrees(state.pose[3])}`
    : waitingHint;
  if (localizedPose() && loc.pose && state.seed) {
    const delta = loc.pose[3] - state.seed[3];
    byId('pose').textContent += `\n初值修正 ${Math.hypot(loc.pose[0] - state.seed[0], loc.pose[1] - state.seed[1]).toFixed(2)} m / ${degrees(Math.atan2(Math.sin(delta), Math.cos(delta)))}`;
  }
  for (const id of ['seed', 'single', 'multi']) byId(id).disabled = locked || !online;
  byId('seed').disabled = locked || !online || state.standing === false || ['COLLECTING', 'MATCHING'].includes(loc.state);
  if (stairPicker.open && (locked || !online || (tool === 'seed' && byId('seed').disabled))) stairPicker.close();
  byId('seed').setAttribute('aria-pressed', tool === 'seed');
  byId('seed').textContent = tool === 'seed' ? '取消给定位姿' : '拖动给定初始位姿';
  byId('newTask').hidden = !!mode || !!taskId;
  byId('taskManager').hidden = !!taskId;
  byId('editor').hidden = !mode || !!taskId;
  byId('discard').disabled = busy;
  byId('saveDraft').disabled = !database || locked || !draft.length || draft.some(p => !Number.isFinite(p.yaw));
  byId('saveDraft').textContent = editingRecord ? '保存修改' : (mode === 'multi' ? '保存路线' : '保存点位');
  byId('saveTask').hidden = !!taskId || !task.points.length || !task.points.every((_, i) => Number.isFinite(task.headings?.[i]));
  byId('saveTask').disabled = !database;
  byId('tip').textContent =
    (editingRecord ? `编辑「${editingRecord.name}」 · ` : '') + (replaceIndex !== null ? `请重新拖动设置第 ${replaceIndex + 1} 站` : mode === 'single' ? '单点任务 · 拖动设置或替换终点' : `多点任务 · 已选 ${draft.length} 站`);
  byId('mapnote').textContent =
    tool === 'seed' ? '按住机身位置，沿狗头方向拖动后松开' : mode && !taskId ? '按住目标位置，向到点时狗头应朝的方向拖动' : '滚轮缩放 · 拖动平移';
  byId('start').disabled = locked || !localizedPose() || !state.ready || !draft.length || draft.some(p => !Number.isFinite(p.yaw)) || tool === 'seed';
  byId('cancel').hidden = !taskId;
  byId('cancel').disabled = !!taskId && (cancelRequested === taskId || (task.phase === 'stopping' && task.reason === '用户取消任务'));
  byId('cancel').textContent = byId('cancel').disabled ? '正在确认停车…' : '取消任务并停车';
  byId('taskStatus').hidden = task.phase === 'idle' || (!!mode && !taskId);
  byId('statusTitle').textContent = taskId ? '当前任务' : '上次任务';
  byId('resume').hidden =
    !['failed', 'canceled'].includes(task.phase) || task.index >= task.points.length;
  byId('progress').textContent = task.phase === 'succeeded' ? `已完成全部 ${task.points.length} 站` :
    `${phases[task.phase] || task.phase}${task.points.length ? ` · 已到达 ${task.index} / ${task.points.length} 站` : ''}`;
  const failedSegment = task.phase === 'failed' && task.index < task.points.length
    ? (task.index ? `点 ${task.index} → 点 ${task.index + 1}` : '起点 → 点 1') : '';
  let remaining = '';
  if (task.phase === 'walking' && localizedPose() && task.points[task.index]) {
    const [x, y, z] = task.points[task.index];
    const [px, py, pz, yaw] = state.pose;
    const distance = Math.hypot(x - px, y - py);
    const targetYaw = task.headings?.[task.index];
    if (distance <= 0.5 && nearest(pz - 0.32) === nearest(z) && Number.isFinite(targetYaw)) {
      const delta = targetYaw - yaw;
      remaining = `终点附近 · 位置差 ${(distance * 100).toFixed(0)} cm · 朝向差 ${degrees(Math.abs(Math.atan2(Math.sin(delta), Math.cos(delta))))}`;
    }
  }
  byId('reason').textContent = [failedSegment, task.reason, remaining].filter(Boolean).join(' · ');
  const key = JSON.stringify([draft, task, locked]);
  if (key !== listKey) {
    listKey = key;
    const list = byId('points');
    list.replaceChildren();
    draft.forEach((p, i) => {
      const item = stopRow(list, i, pointTitle(p), `(${p.x.toFixed(1)}, ${p.y.toFixed(1)}) m · ${degrees(p.yaw)}`);
      const focusPoint = () => {
        follow(false);
        byId('floor').value = p.viewFloor ?? p.floor;
        center = [p.x, p.y];
        draw();
      };
      item.querySelector('.row-copy').onclick = focusPoint;
      if (locked) return;
      function swap(j) {
        [draft[i], draft[j]] = [draft[j], draft[i]];
        replaceIndex = null;
        update();
      }
      if (i > 0) item.append(button('前移', () => swap(i - 1), '向前移动'));
      if (i + 1 < draft.length) item.append(button('后移', () => swap(i + 1), '向后移动'));
      if (mode === 'multi') item.append(button('重选', () => { replaceIndex = i; tool = 'target'; focusPoint(); update(); }, '重新设置该站位姿'));
      item.append(
        button(
          '删除',
          () => {
            draft.splice(i, 1);
            replaceIndex = null;
            update();
          },
          '移除该点',
        ),
      );
    });
    const live = byId('executing');
    live.replaceChildren();
    task.points.forEach(([x, y, z], i) => {
      const label = i < task.index ? '已到达' : i === task.index && !['at_waypoint','canceled'].includes(task.phase) ? phases[task.phase] || '待执行' : terminal.includes(task.phase) ? '未执行' : '待执行';
      const point = storedPoint([x, y, z], task.headings?.[i]);
      const item = stopRow(live, i, pointTitle(point), `${point.stair ? `踏步 ${z.toFixed(2)} m · ` : ''}朝向 ${degrees(point.yaw)}`);
      const tag = document.createElement('small');
      tag.className = 'state-tag';
      tag.dataset.ok = i < task.index;
      tag.textContent = label;
      item.append(tag);
    });
  }
  if (following && localizedPose()) follow(true);
  else draw();
}
async function command(path, payload) {
  busy = true;
  byId('notice').textContent = '';
  update();
  try {
    const response = await fetch('/api/' + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    if (!response.ok) throw Error(result.error || '请求失败');
    return true;
  } catch (error) {
    byId('notice').textContent = error.message;
    return false;
  } finally {
    busy = false;
    update();
  }
}
const events = new EventSource('/api/events');
events.onmessage = (e) => {
  state = JSON.parse(e.data);
  if (previousFix !== undefined && (state.localization?.state !== 'LOCALIZED' || state.localization?.request_id !== previousFix)) previousFix = undefined;
  if (state.task?.id === pendingTask) pendingTask = null;
  online = true;
  update();
};
events.onerror = () => {
  online = false;
  update();
};
byId('floor').onchange = () => follow(false);
function degrees(yaw) {
  return Number.isFinite(yaw) ? `${Math.round(yaw * 180 / Math.PI)}°` : '朝向不限';
}
byId('follow').onclick = () => follow(!following);
byId('fit').onclick = () => {
  follow(false);
  fit();
};
byId('seed').onclick = () => {
  tool = tool === 'seed' ? (mode ? 'target' : 'browse') : 'seed';
  update();
};
for (const next of ['single', 'multi'])
  byId(next).onclick = () => {
    mode = next;
    tool = 'target';
    byId('notice').textContent = '';
    update();
  };
function closeEditor() {
  editingRecord = null;
  replaceIndex = null;
  mode = null;
  tool = 'browse';
  draft = [];
  drag = null;
  byId('notice').textContent = '';
  update();
}
byId('discard').onclick = closeEditor;
window.addEventListener('keydown', (event) => {
  if (event.key !== 'Escape' || document.querySelector('dialog[open]') || busy || currentTask()) return;
  if (drag) { drag = null; draw(); }
  else if (tool === 'seed') byId('seed').click();
  else if (mode) closeEditor();
});
byId('start').onclick = async () => {
  const id = crypto.randomUUID();
  if (await command('tasks', { id, points: draft.map(({x, y, floor, yaw}) => ({x, y, floor, yaw})) })) {
    pendingTask = state.task?.id === id.replaceAll('-', '') ? null : id.replaceAll('-', '');
    closeEditor();
  }
};
byId('cancel').onclick = async () => {
  const id = currentTask();
  if (!id || byId('cancel').disabled) return;
  cancelRequested = id;
  if (!await command('cancel', { id })) cancelRequested = null;
  update();
};
byId('resume').onclick = () => {
  draft = state.task.points
    .slice(state.task.index)
    .map((point, i) => storedPoint(point, state.task.headings?.[state.task.index + i]));
  mode = draft.length > 1 ? 'multi' : 'single';
  tool = 'target';
  if (draft.some(p => !Number.isFinite(p.yaw))) byId('notice').textContent = '旧任务含未设朝向的点，请删除这些点并重新拖动添加。';
  update();
};
function mouse(e) {
  const box = canvas.getBoundingClientRect();
  return [e.clientX - box.left, e.clientY - box.top];
}
function cellIndex(x, y) {
  const column = Math.round((x - atlas.origin[0]) / resolution);
  const row = Math.round((y - atlas.origin[1]) / resolution);
  return column >= 0 && column < cols && row >= 0 && row < rows ? column * rows + row : -1;
}
function stairName(stair) {
  return `${floors[stair.floor].name} ↔ ${floors[stair.to].name}`;
}
function targetSurfaces(x, y, viewFloor) {
  const cell = cellIndex(x, y);
  const matches = stairLanes[viewFloor].filter(({stair}) => Math.abs(x - stair.x) <= stair.width / 2 &&
    y >= Math.min(stair.start, stair.end) && y <= Math.max(stair.start, stair.end));
  if (!matches.length) return [{floor: viewFloor, value: floors[viewFloor].cells[cell] || 0}];
  return matches.map(({stair}) => stairSurface(stair, cell, y));
}
function stairSurface(stair, cell, y) {
  const rise = floors[stair.to].height - floors[stair.floor].height;
  const fraction = (y - stair.start) / (stair.end - stair.start);
  const expected = floors[stair.floor].height + rise * fraction;
  let best = {floor: -1, value: 0, stair}, error = 1.5 * Math.abs(rise) / stair.count;
  for (const floor of [stair.floor, stair.to]) {
    const {cells, elevation} = floors[floor];
    if (!cells[cell] || !elevation) continue;
    const difference = Math.abs(elevation.getInt16(cell * 2, true) / 100 - expected);
    if (difference <= error) {
      best = {floor, value: cells[cell], stair};
      error = difference;
    }
  }
  return best;
}
function surfaceHeight(x, y, floor) {
  const index = cellIndex(x, y);
  if (index < 0 || !floors[floor].cells[index] || !floors[floor].elevation) return '';
  return ` · 楼面 ${(floors[floor].elevation.getInt16(index * 2, true) / 100).toFixed(2)} m`;
}
function storedPoint([x, y, z], yaw) {
  const floor = nearest(z);
  const views = [floor, floor - 1, floor + 1];
  for (const viewFloor of views) {
    if (!floors[viewFloor]) continue;
    const surface = targetSurfaces(x, y, viewFloor).find(candidate => candidate.stair && candidate.floor === floor);
    if (surface) return {x, y, floor, viewFloor, yaw, stair: surface.stair};
  }
  return {x, y, floor, viewFloor: floor, yaw};
}
canvas.onpointerdown = (e) => {
  if (drag || e.button !== 0) return;
  const pos = mouse(e);
  if (tool !== 'browse' && following) follow(false);
  drag = {
    id: e.pointerId,
    kind: tool === 'seed' && !byId('seed').disabled ? 'seed' : tool === 'target' && !currentTask() && !busy ? 'heading' : 'pan',
    start: pos,
    position: world(pos),
    viewFloor: selected(),
    end: pos,
    center: [...center],
  };
  canvas.setPointerCapture(e.pointerId);
};
canvas.onpointermove = (e) => {
  const pos = mouse(e),
    [x, y] = world(pos);
  const surfaces = targetSurfaces(x, y, selected());
  const surface = surfaces[0];
  const cell = cellIndex(x, y), cells = floors[selected()].cells;
  const gap = !surface.stair && !surface.value && cell >= rows && cell < cells.length - rows &&
    cell % rows > 0 && cell % rows < rows - 1 && [-rows, rows, 1, -1].every(offset => cells[cell + offset]);
  const hint = surfaces.length > 1 ? ` · ${surfaces.length} 段楼梯重叠 · 拖动后选择楼段` :
    `${surface.stair ? ' · 楼梯' : ''}${surface.floor >= 0 ? surfaceHeight(x, y, surface.floor) : ''} · ${gap ? '楼面采样缺口' : ['无规划表面', '规划受限', '可选目标'][surface.value]}`;
  byId('xy').textContent = `${x.toFixed(2)}, ${y.toFixed(2)} m${hint}`;
  if (!drag || e.pointerId !== drag.id) return;
  drag.end = pos;
  if (drag.kind === 'pan' && Math.hypot(pos[0] - drag.start[0], pos[1] - drag.start[1]) > 4) {
    follow(false);
    center = [
      drag.center[0] - (pos[0] - drag.start[0]) / scale,
      drag.center[1] + (pos[1] - drag.start[1]) / scale,
    ];
  }
  draw();
};
canvas.onpointercancel = () => {
  drag = null;
  draw();
};
canvas.onpointerup = async (e) => {
  if (!drag || e.pointerId !== drag.id) return;
  const { start, kind, position, viewFloor } = drag;
  drag = null;
  const end = mouse(e),
    dx = end[0] - start[0],
    dy = start[1] - end[1];
  if (currentTask() || busy || !online || kind === 'pan') return;
  if (Math.hypot(dx, dy) < 6) {
    byId('notice').textContent = '请拖动箭头给出朝向';
    draw();
    return;
  }
  const [x, y] = position, yaw = Math.atan2(dy, dx);
  const surfaces = targetSurfaces(x, y, viewFloor);
  if (surfaces.length === 1) return applySurface(surfaces[0], {x, y, yaw, viewFloor}, kind);
  byId('stairPosition').textContent = `落点 (${x.toFixed(2)}, ${y.toFixed(2)}) m · 朝向 ${degrees(yaw)}`;
  const options = byId('stairOptions');
  options.replaceChildren();
  for (const surface of surfaces) {
    const {stair, floor, value} = surface;
    const direction = stair.floor === viewFloor ? '上行' : '下行';
    const available = kind === 'seed' ? value > 0 : value === 2;
    const status = !value ? '未匹配到踏步' : available ? '可选' : '规划受限';
    const height = floor < 0 ? '' : surfaceHeight(x, y, floor);
    const choice = button(`${direction} · ${stairName(stair)}${height} · ${status}`, () => {
      if (!stairPicker.open) return;
      stairPicker.close();
      return applySurface(surface, {x, y, yaw, viewFloor}, kind);
    });
    choice.disabled = !available;
    options.append(choice);
  }
  draw();
  stairPicker.showModal();
};
async function applySurface({floor, value: cell, stair}, {x, y, yaw, viewFloor}, kind) {
  if (!cell || (kind !== 'seed' && cell !== 2)) {
    byId('notice').textContent = cell === 1 ? '该处规划受限，可打开“显示规划限制”查看。' : stair ? '该处未匹配到踏步，请在楼梯内重新选点。' : '该处无有效楼面，请重新选点。';
    draw();
    return;
  }
  byId('notice').textContent = '';
  const point = { x, y, floor, yaw };
  if (stair) Object.assign(point, {viewFloor, stair});
  if (kind === 'seed') {
    previousFix = state.localization?.request_id ?? null;
    if (await command('seed', { x, y, floor, yaw })) tool = mode ? 'target' : 'browse';
    else previousFix = undefined;
  } else if (replaceIndex !== null) { draft[replaceIndex] = point; replaceIndex = null; }
  else if (mode === 'single') draft = [point];
  else if (draft.length < 100) draft.push(point);
  else byId('notice').textContent = '每个任务最多支持 100 个点';
  update();
}
canvas.addEventListener(
  'wheel',
  (e) => {
    e.preventDefault();
    const pos = mouse(e),
      before = world(pos);
    follow(false);
    scale = Math.min(350, Math.max(6, scale * Math.exp(-e.deltaY * 0.0015)));
    const after = world(pos);
    center = [center[0] + before[0] - after[0], center[1] + before[1] - after[1]];
    draw();
  },
  { passive: false },
);
async function openLibraryDatabase() {
  if (!window.indexedDB) throw Error('浏览器存储不可用');
  const request = indexedDB.open(`迹航/${atlas.name}/${atlas.origin}/${atlas.shape}/${resolution}`, 1);
  request.onupgradeneeded = () => request.result.createObjectStore('records', {keyPath: 'id'});
  return new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}
async function refreshRecords() {
  const request = database.transaction('records').objectStore('records').getAll();
  records = await new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
  floors.forEach((floor, index) => {
    floor.name = records.find(item => item.id === `floor-${index}`)?.name ?? atlas.floors[index].name;
    byId('floor').options[index].text = floor.name;
  });
  listKey = '';
  renderLibrary();
  update();
}
async function writeRecord(previous, next) {
  const transaction = database.transaction('records', 'readwrite');
  const store = transaction.objectStore('records');
  const request = store.get(previous?.id ?? next.id);
  let conflict = false;
  const committed = new Promise((resolve, reject) => {
    transaction.oncomplete = resolve;
    transaction.onabort = () => reject(Error(conflict ? '此记录已在另一页面修改，请重新打开。' : '写入失败，请检查浏览器可用空间。'));
  });
  request.onsuccess = () => {
    if (request.result?.revision !== previous?.revision) { conflict = true; transaction.abort(); return; }
    if (next) store.put({...next, revision: (previous?.revision ?? 0) + 1});
    else store.delete(previous.id);
  };
  await committed;
  await refreshRecords();
}
function recordForm(title, name, action, detail = '', deleting = false) {
  const dialog = byId('recordDialog'), field = byId('recordName'), confirm = byId('recordConfirm');
  byId('recordTitle').textContent = title;
  byId('recordDetail').textContent = detail;
  byId('nameLabel').hidden = deleting;
  field.disabled = deleting;
  field.value = name;
  field.setCustomValidity('');
  confirm.disabled = false;
  confirm.textContent = deleting ? '删除' : '保存';
  confirm.className = deleting ? 'danger' : 'primary';
  byId('recordForm').onsubmit = async event => {
    event.preventDefault();
    if (confirm.disabled) return;
    const value = field.value.trim();
    if (!deleting && !value) { field.setCustomValidity('请输入名称'); field.reportValidity(); return; }
    confirm.disabled = true;
    try { await action(value); dialog.close(); }
    catch (error) { byId('recordDetail').textContent = error.message; }
    finally { confirm.disabled = false; }
  };
  dialog.showModal();
  if (!deleting) { field.focus(); field.select(); }
}
byId('recordName').oninput = () => byId('recordName').setCustomValidity('');
byId('recordCancel').onclick = () => byId('recordDialog').close();
byId('floorRename').onclick = async () => {
  const index = selected(), id = `floor-${index}`;
  try { await refreshRecords(); } catch { byId('notice').textContent = '读取楼层名称失败'; return; }
  const previous = records.find(item => item.id === id);
  recordForm('编辑楼层名称', floors[index].name, name => writeRecord(previous, {id, kind: 'floor', name}));
};
function savePoints(points, kind, previous) {
  const title = kind === 'place' ? '点位' : '路线';
  const snapshot = structuredClone(points);
  recordForm(`保存${title}`, previous?.name ?? '', async name => {
    const id = previous?.id ?? crypto.randomUUID();
    await writeRecord(previous, {id, name, kind, points: snapshot});
    if (points === draft) editingRecord = records.find(item => item.id === id);
    byId('notice').textContent = `已保存「${name}」`;
    update();
  }, `${points.length} 个停靠点 · 保留位置与朝向`);
}
byId('saveDraft').onclick = () => savePoints(draft, mode === 'multi' ? 'route' : 'place', editingRecord);
byId('saveTask').onclick = () => {
  const task = state.task;
  const points = task.points.map((point, index) => storedPoint(point, task.headings[index]));
  savePoints(points, points.length > 1 ? 'route' : 'place', null);
};
function loadRecord(record, edit) {
  if (currentTask() || busy || (edit && mode)) return;
  const incoming = structuredClone(record.points);
  if (mode === 'multi' && draft.length + incoming.length > 100) { byId('libraryEmpty').textContent = '每个任务最多支持 100 个点'; return; }
  if (mode === 'single' && incoming.length > 1) return;
  draft = mode === 'multi' ? [...draft, ...incoming] : incoming;
  if (!mode) mode = record.kind === 'place' ? 'single' : 'multi';
  if (edit) editingRecord = record;
  replaceIndex = null;
  tool = 'target';
  byId('notice').textContent = '';
  follow(false);
  byId('floor').value = incoming[0].viewFloor ?? incoming[0].floor;
  fit();
  byId('library').close();
  update();
}
function renderLibrary() {
  const places = libraryKind === 'place', type = places ? '点位' : '路线';
  const list = byId('savedList');
  list.replaceChildren();
  byId('placesTab').setAttribute('aria-pressed', places);
  byId('routesTab').setAttribute('aria-pressed', !places);
  const visible = records.filter(item => item.kind === libraryKind).sort((a,b) => a.name.localeCompare(b.name,'zh-CN'));
  byId('libraryEmpty').textContent = visible.length ? '' : `尚未保存${type}`;
  byId('libraryNew').textContent = `新建${type}`;
  byId('libraryNew').disabled = !!mode || !!currentTask() || busy || !online;
  for (const record of visible) {
    const item = row(list, record.name), copy = item.firstChild, meta = document.createElement('small');
    copy.className = 'row-copy'; meta.className = 'meta';
    const first = record.points[0];
    meta.textContent = places ? `${pointTitle(first)} · ${degrees(first.yaw)}` : `${record.points.length} 站 · ${pointTitle(first)}起步`;
    copy.append(meta);
    const use = button(mode === 'multi' ? '添加' : mode === 'single' && draft.length ? '替换' : '使用', () => loadRecord(record, false));
    use.disabled = !!currentTask() || busy || !online || (mode === 'single' && record.points.length > 1);
    const edit = button('编辑', () => loadRecord(record, true));
    edit.disabled = !!mode || !!currentTask() || busy || !online;
    item.append(use, edit, button('删除', () => recordForm(`删除「${record.name}」？`, '', () => writeRecord(record, null), '', true)));
  }
}
byId('placesTab').onclick = () => { libraryKind = 'place'; renderLibrary(); };
byId('routesTab').onclick = () => { libraryKind = 'route'; renderLibrary(); };
byId('libraryOpen').onclick = async () => {
  try { await refreshRecords(); byId('library').showModal(); }
  catch { byId('notice').textContent = '无法读取当前浏览器的资料库'; }
};
byId('libraryNew').onclick = () => {
  if (currentTask() || mode || busy || !online) return;
  byId('library').close();
  byId(libraryKind === 'place' ? 'single' : 'multi').click();
};
byId('libraryClose').onclick = () => byId('library').close();
try {
  database = await openLibraryDatabase();
  await refreshRecords();
  byId('libraryOpen').disabled = byId('floorRename').disabled = false;
  window.addEventListener('focus', () => refreshRecords().catch(() => { byId('notice').textContent = '资料库刷新失败'; }));
} catch {
  database = null;
  byId('libraryOpen').disabled = byId('floorRename').disabled = true;
  byId('notice').textContent = '当前浏览器存储不可用，无法保存点位或楼层名称。';
}
new ResizeObserver(draw).observe(canvas);
fit();
update();
