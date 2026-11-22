const el = (id) => document.getElementById(id);
const map = el("map"),
  ctx = map.getContext("2d");
const labels = {
  paint_free: "人工可通行",
  free: "自动清障",
  obstacle: "障碍物",
  deny: "禁行区",
  fill: "补建地面",
  restore: "恢复自动结果",
};
let atlas,
  base,
  state,
  edits = [],
  floor = 0,
  tool = "pan",
  poly = [],
  box = null,
  view = [0, 0],
  scale = 1,
  gesture,
  cache,
  mask,
  shown,
  heights;
let before = false,
  preview = null,
  route = [],
  first = null,
  busy = false,
  probe = null,
  cloud,
  orbit = [0.4, 0.65, 1],
  orbitDown;
let lastPair = null,
  savedRoutes = [];
const hiddenStairs = new Set();
const stairInk = (ascending) => (ascending ? "#185a98" : "#763d79");
const num = (id) => el(id).valueAsNumber;
const metric = (pt) => [
  (pt[0] - view[0]) * scale + map.clientWidth / 2,
  map.clientHeight / 2 - (pt[1] - view[1]) * scale,
];
const world = (pt) => [
  (pt[0] - map.clientWidth / 2) / scale + view[0],
  (map.clientHeight / 2 - pt[1]) / scale + view[1],
];
const point = (e) => {
  const r = map.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
};
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const bytes = (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
function note(text, error = false) {
  el("status").textContent = text;
  el("status").dataset.error = error;
}
async function request(path, body) {
  const res = await fetch(
    path,
    body === undefined
      ? {}
      : {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(body),
        },
  );
  const data = await res.json();
  if (!res.ok) throw Error(data.message || "请求失败");
  return data;
}
async function run(task) {
  if (busy) return;
  busy = true;
  for (const input of el("selection").querySelectorAll("input,select"))
    input.disabled = true;
  controls();
  try {
    await task();
  } catch (e) {
    note(e.message, true);
  } finally {
    busy = false;
    for (const input of el("selection").querySelectorAll("input,select"))
      input.disabled = false;
    controls();
  }
}
function controls() {
  const enabled = !!state && !busy;
  el("undo").disabled = !enabled || state.cursor === 0;
  el("redo").disabled = !enabled || state.cursor + 1 >= state.history.length;
  el("preview").disabled = !enabled || before || poly.length < 3;
  el("apply").disabled = !enabled || before || poly.length < 3;
  el("inspect").disabled = !enabled || (!poly.length && !probe);
  el("floor").disabled = busy;
  for (const btn of el("tools").children) btn.disabled = busy;
  el("auto").disabled = busy;
  el("export").style.pointerEvents = busy ? "none" : "";
  el("boxFields").hidden = tool !== "box";
  el("boxFields").style.display = tool === "box" ? "grid" : "none";
  el("fillLabel").hidden = el("action").value !== "fill";
}
function clearPreview() {
  preview = null;
  before = false;
  el("auto").setAttribute("aria-pressed", false);
  if (atlas) raster();
  controls();
}
function raster() {
  shown = before ? base : preview?.atlas || atlas;
  if (!shown) return;
  const surface = shown.floors[floor];
  mask = bytes(surface.pixels);
  const raw = bytes(surface.elevation_cm);
  heights = new Int16Array(raw.buffer);
  const [nx, ny] = shown.shape;
  cache = document.createElement("canvas");
  cache.width = nx;
  cache.height = ny;
  const c = cache.getContext("2d"),
    img = c.createImageData(nx, ny),
    rgba = new Uint32Array(img.data.buffer);
  const colors = [0xffeceee9, 0xff8ba5cc, 0xffb1c98f];
  const manual = new Uint8Array(nx * ny);
  for (const [start, stop, value] of shown.manual?.[floor] || [])
    manual.fill(value, start, stop);
  const previous =
    preview && !before ? bytes(atlas.floors[floor].pixels) : null;
  for (let x = 0; x < nx; x++)
    for (let y = 0; y < ny; y++) {
      const idx = x * ny + y;
      rgba[(ny - 1 - y) * nx + x] =
        previous &&
        mask[idx] !== previous[idx] &&
        (mask[idx] === 2 || previous[idx] === 2)
          ? mask[idx] === 2
            ? 0xff70ab26
            : 0xff5152d6
          : manual[idx] === 1
            ? 0xff454e65
            : manual[idx] === 2
              ? 0xff9a7091
              : colors[mask[idx]];
    }
  c.putImageData(img, 0, 0);
  draw();
}
function fit() {
  if (!atlas) return;
  const [nx, ny] = atlas.shape,
    res = atlas.resolution;
  let minX = nx,
    maxX = -1,
    minY = ny,
    maxY = -1;
  for (let x = 0; x < nx; x++)
    for (let y = 0; y < ny; y++)
      if (mask[x * ny + y]) {
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
      }
  if (maxX < 0) {
    minX = minY = 0;
    maxX = nx - 1;
    maxY = ny - 1;
  }
  view = [
    atlas.origin[0] + ((minX + maxX) * res) / 2,
    atlas.origin[1] + ((minY + maxY) * res) / 2,
  ];
  scale =
    0.91 *
    Math.min(
      map.clientWidth / ((maxX - minX + 1) * res),
      map.clientHeight / ((maxY - minY + 1) * res),
    );
  draw();
}
function stroke(points, color, fill = false) {
  if (!points.length) return;
  ctx.beginPath();
  points.forEach((pt, i) => {
    const [x, y] = metric(pt);
    if (!i) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  if (fill) {
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
}
function dot(pt, color, r = 4) {
  const [x, y] = metric(pt);
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  ctx.fillStyle = color;
  ctx.fill();
}
function rotateHandle() {
  const local = [0, box.d / 2 + 22 / scale],
    ang = (box.a * Math.PI) / 180;
  return [box.x - local[1] * Math.sin(ang), box.y + local[1] * Math.cos(ang)];
}
function stairControls() {
  const list = el("stairs");
  list.replaceChildren();
  for (const [index, flight] of atlas.stairs.entries()) {
    if (flight.floor !== floor && flight.to !== floor) continue;
    const ascending = floor === flight.floor;
    const target = atlas.floors[ascending ? flight.to : flight.floor];
    const button = document.createElement("button");
    button.textContent = `楼梯 ${index + 1} · ${ascending ? "上行" : "下行"}至${target.name}`;
    button.title = "显示或隐藏这段楼梯";
    button.style.setProperty("--ink", stairInk(ascending));
    button.setAttribute("aria-pressed", !hiddenStairs.has(index));
    button.onclick = () => {
      if (hiddenStairs.has(index)) hiddenStairs.delete(index);
      else hiddenStairs.add(index);
      button.setAttribute("aria-pressed", !hiddenStairs.has(index));
      draw();
    };
    list.append(button);
  }
  if (!list.childElementCount && atlas.floors.length > 1)
    list.textContent = "本层未识别到连接其他楼层的楼梯";
}
function drawStairs() {
  for (const [index, flight] of shown.stairs.entries()) {
    if (
      hiddenStairs.has(index) ||
      (flight.floor !== floor && flight.to !== floor)
    )
      continue;
    const ascending = flight.floor === floor;
    const color = stairInk(ascending);
    const cosine = Math.cos(flight.angle),
      sine = Math.sin(flight.angle);
    const pointAt = (side, fraction) => {
      const along = flight.start + (flight.end - flight.start) * fraction;
      const across = flight.x + side * flight.width;
      return [across * cosine - along * sine, across * sine + along * cosine];
    };
    ctx.save();
    if (!ascending) ctx.setLineDash([5, 3]);
    stroke(
      [pointAt(-0.5, 0), pointAt(0.5, 0), pointAt(0.5, 1), pointAt(-0.5, 1)],
      color,
      color + "12",
    );
    ctx.setLineDash([]);
    ctx.globalAlpha = 0.55;
    for (let step = 1; step < flight.count; step++) {
      const fraction = step / flight.count;
      stroke([pointAt(-0.5, fraction), pointAt(0.5, fraction)], color);
    }
    ctx.globalAlpha = 1;
    const side = ascending ? -0.18 : 0.18;
    const tail = pointAt(side, ascending ? 0.18 : 0.82);
    const head = pointAt(side, ascending ? 0.46 : 0.54);
    stroke([tail, head], color);
    const [tipX, tipY] = metric(head),
      [tailX, tailY] = metric(tail);
    ctx.translate(tipX, tipY);
    ctx.rotate(Math.atan2(tipY - tailY, tipX - tailX));
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(-9, -5);
    ctx.lineTo(-9, 5);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.restore();
  }
}
function draw() {
  if (!cache) return;
  const ratio = devicePixelRatio || 1,
    w = map.clientWidth,
    h = map.clientHeight;
  map.width = w * ratio;
  map.height = h * ratio;
  ctx.scale(ratio, ratio);
  ctx.imageSmoothingEnabled = false;
  const res = atlas.resolution,
    [nx, ny] = atlas.shape;
  const [left, top] = metric([
    atlas.origin[0] - res / 2,
    atlas.origin[1] + (ny - 0.5) * res,
  ]);
  ctx.drawImage(cache, left, top, nx * res * scale, ny * res * scale);
  drawStairs();
  stroke(poly, "#185fbc", poly.length > 2 ? "#2779d51a" : false);
  if (tool === "box" && box) {
    poly.forEach((pt) => dot(pt, "#1663a4"));
    dot([box.x, box.y], "#145b91");
    const h = rotateHandle();
    stroke(
      [[(poly[2][0] + poly[3][0]) / 2, (poly[2][1] + poly[3][1]) / 2], h],
      "#176899",
    );
    dot(h, "#176899", 6);
  } else if (poly.length && poly.length < 24)
    poly.forEach((pt) => dot(pt, "#1663a4", 3));
  if (first && first.floor === floor) dot([first.x, first.y], "#0746af", 6);
  for (const segment of route) stroke(segment, "#174d9f");
}
function floorDefaults() {
  const level = atlas.floors[floor].height;
  el("lo").value = (level - 0.15).toFixed(3);
  el("hi").value = (level + 0.65).toFixed(3);
  el("height").value = level.toFixed(3);
}
function chooseTool(next) {
  tool = next;
  poly = [];
  box = null;
  route = [];
  first = null;
  probe = null;
  for (const btn of el("tools").children)
    btn.setAttribute("aria-pressed", btn.dataset.tool === tool);
  el("hint").textContent = {
    pan: "拖动平移，滚轮缩放。单击查看格子状态。",
    box: "拖出矩形。拖中心移动，拖角点缩放，拖圆形把手旋转。",
    lasso: "按住拖动圈选，或依次点击顶点。拖动已有顶点可微调。",
    route: "先点击起点，再点击终点。选终点前可以切换楼层。",
  }[tool];
  clearPreview();
  draw();
}
function setBox(value, inputs = true) {
  box = value;
  const ang = (value.a * Math.PI) / 180,
    co = Math.cos(ang),
    si = Math.sin(ang);
  poly = [
    [-1, -1],
    [1, -1],
    [1, 1],
    [-1, 1],
  ].map(([x, y]) => [
    value.x + (co * x * value.w) / 2 - (si * y * value.d) / 2,
    value.y + (si * x * value.w) / 2 + (co * y * value.d) / 2,
  ]);
  if (inputs) {
    for (const [id, key] of Object.entries({
      cx: "x",
      cy: "y",
      width: "w",
      depth: "d",
      angle: "a",
    }))
      el(id).value = Number(value[key].toFixed(3));
  }
  draw();
  controls();
}
function resetRegion() {
  poly = [];
  box = null;
  probe = null;
  preview = null;
  for (const id of ["cx", "cy", "width", "depth", "angle"]) el(id).value = "";
}
async function reload(initial = false) {
  const data = await request("atlas");
  atlas = data.atlas;
  state = data.state;
  edits = data.edits;
  preview = null;
  before = false;
  if (initial) {
    base = (await request("atlas?base=1")).atlas;
    el("floor").replaceChildren(
      ...atlas.floors.map((item, i) => new Option(item.name, i)),
    );
    floorDefaults();
  }
  if (!base) base = atlas;
  stairControls();
  raster();
  el("history").textContent =
    `当前第 ${state.cursor} 步，共 ${state.history.length - 1} 步` +
    (edits.length ? " · " + labels[edits.at(-1).action] : " · 自动生成结果");
  controls();
}
function selectedCell(pt) {
  const [ix, iy] = pt.map((v, i) =>
    Math.round((v - atlas.origin[i]) / atlas.resolution),
  );
  if (ix < 0 || iy < 0 || ix >= atlas.shape[0] || iy >= atlas.shape[1])
    return null;
  const idx = ix * atlas.shape[1] + iy;
  return { value: mask[idx], z: heights[idx] / 100 };
}
function position(pt) {
  const cell = selectedCell(pt);
  return (
    `X ${pt[0].toFixed(2)} · Y ${pt[1].toFixed(2)} m` +
    (cell ? ` · ${["未知", "受限", "可通行"][cell.value]}` : "")
  );
}
function pickRoute(pt) {
  const cell = selectedCell(pt);
  if (!cell || cell.value !== 2) return note("请选择可通行格。", true);
  const stop = { floor, x: pt[0], y: pt[1] };
  if (!first) {
    first = stop;
    lastPair = null;
    route = [];
    note("起点已选。请点击终点。");
    draw();
    return;
  }
  run(async () => {
    const result = await request("plan", { points: [first, stop] });
    lastPair = [first, stop];
    const nominal = atlas.floors[floor].height;
    let segment = [];
    route = [];
    for (const p of result.path) {
      if (Math.abs(p[2] - 0.5 - nominal) <= 0.8) segment.push(p);
      else if (segment.length) {
        route.push(segment);
        segment = [];
      }
    }
    if (segment.length) route.push(segment);
    first = null;
    draw();
    note(`PCT 已规划 · 路径 ${result.length.toFixed(2)} m`);
  });
}
map.onpointerdown = (e) => {
  if (!atlas || busy) return;
  const pixel = point(e),
    pt = world(pixel);
  map.setPointerCapture(e.pointerId);
  if (e.button === 1 || e.button === 2 || tool === "pan") {
    gesture = { mode: "pan", pixel, view: [...view] };
    return;
  }
  if (before) return;
  if (tool === "route") {
    pickRoute(pt);
    return;
  }
  clearPreview();
  if (tool === "lasso") {
    const index = poly.findIndex((p) => distance(metric(p), pixel) < 7);
    gesture = { mode: index >= 0 ? "vertex" : "lasso", index, pixel };
    if (index < 0) poly.push(pt);
    draw();
    controls();
    return;
  }
  if (box) {
    if (distance(metric(rotateHandle()), pixel) < 12) {
      gesture = { mode: "rotate" };
      return;
    }
    const index = poly.findIndex((p) => distance(metric(p), pixel) < 9);
    if (index >= 0) {
      gesture = { mode: "resize", opposite: poly[(index + 2) % 4], a: box.a };
      return;
    }
    if (distance(metric([box.x, box.y]), pixel) < 14) {
      gesture = { mode: "move", pt, old: { ...box } };
      return;
    }
  }
  gesture = { mode: "new", pt };
  setBox({ x: pt[0], y: pt[1], w: 0.1, d: 0.1, a: 0 });
};
map.onpointermove = (e) => {
  if (!atlas) return;
  const pixel = point(e),
    pt = world(pixel);
  el("position").textContent = position(pt);
  if (!gesture) return;
  const g = gesture;
  if (g.mode === "pan") {
    view = [
      g.view[0] - (pixel[0] - g.pixel[0]) / scale,
      g.view[1] + (pixel[1] - g.pixel[1]) / scale,
    ];
    draw();
    return;
  }
  if (g.mode === "lasso") {
    if (poly.length < 4000 && distance(metric(poly.at(-1)), pixel) > 5)
      poly.push(pt);
    draw();
    controls();
  } else if (g.mode === "vertex") {
    poly[g.index] = pt;
    draw();
  } else if (g.mode === "move") {
    setBox({
      ...g.old,
      x: g.old.x + pt[0] - g.pt[0],
      y: g.old.y + pt[1] - g.pt[1],
    });
  } else if (g.mode === "rotate") {
    setBox({
      ...box,
      a: (Math.atan2(pt[1] - box.y, pt[0] - box.x) * 180) / Math.PI - 90,
    });
  } else {
    const anchor = g.opposite || g.pt,
      ang = ((g.a || 0) * Math.PI) / 180,
      dx = pt[0] - anchor[0],
      dy = pt[1] - anchor[1];
    setBox({
      x: (pt[0] + anchor[0]) / 2,
      y: (pt[1] + anchor[1]) / 2,
      w: Math.max(0.1, Math.abs(dx * Math.cos(ang) + dy * Math.sin(ang))),
      d: Math.max(0.1, Math.abs(-dx * Math.sin(ang) + dy * Math.cos(ang))),
      a: g.a || 0,
    });
  }
};
map.onpointerup = (e) => {
  if (gesture?.mode === "pan" && distance(gesture.pixel, point(e)) < 4) {
    probe = world(point(e));
    const at = probe;
    note(position(probe));
    if (!before && !preview)
      request(`probe?floor=${floor}&x=${at[0]}&y=${at[1]}`)
        .then((data) => {
          if (probe === at) note(data.info);
        })
        .catch((err) => note(err.message, true));
  }
  gesture = null;
  controls();
};
map.onlostpointercapture = () => (gesture = null);
map.oncontextmenu = (e) => e.preventDefault();
map.onwheel = (e) => {
  e.preventDefault();
  if (!atlas) return;
  const px = point(e),
    anchor = world(px);
  scale = Math.max(0.02, Math.min(2000, scale * Math.exp(-0.0015 * e.deltaY)));
  const after = world(px);
  view = [view[0] + anchor[0] - after[0], view[1] + anchor[1] - after[1]];
  draw();
};
for (const id of ["cx", "cy", "width", "depth", "angle"])
  el(id).oninput = () => {
    clearPreview();
    const nums = ["cx", "cy", "width", "depth", "angle"].map(num);
    if (nums.every(Number.isFinite) && nums[2] > 0 && nums[3] > 0)
      setBox(
        { x: nums[0], y: nums[1], w: nums[2], d: nums[3], a: nums[4] },
        false,
      );
    else {
      poly = [];
      box = null;
      draw();
      controls();
    }
  };
for (const id of ["action", "lo", "hi", "height"])
  el(id).oninput = clearPreview;
async function listRoutes() {
  savedRoutes = await request("routes");
  el("routes").replaceChildren(
    ...savedRoutes.map((item) => new Option(item.name, item.name)),
  );
}
for (const btn of el("tools").children)
  btn.onclick = () => {
    chooseTool(btn.dataset.tool);
    el("regression").hidden = tool !== "route";
    if (tool === "route") listRoutes().catch((e) => note(e.message, true));
  };
el("saveRoute").onclick = () =>
  run(async () => {
    if (!lastPair) throw Error("请先在地图上完成一次试规划");
    await request("routes", {
      epoch: state.epoch,
      name: el("routeName").value,
      points: lastPair,
    });
    await listRoutes();
    note("路线已保存。");
  });
el("removeRoute").onclick = () =>
  run(async () => {
    await request("routes", {
      epoch: state.epoch,
      name: el("routes").value,
      remove: true,
    });
    await listRoutes();
  });
el("testRoutes").onclick = () =>
  run(async () => {
    const lines = [];
    for (const saved of savedRoutes) {
      try {
        const test = await request("plan", { points: saved.points });
        lines.push(`${saved.name} · ${test.length.toFixed(1)} m`);
      } catch (err) {
        lines.push(`${saved.name} · ${err.message}`);
      }
    }
    el("routeResult").textContent = lines.join(" / ") || "尚未保存路线";
  });
el("floor").onchange = () => {
  floor = +el("floor").value;
  stairControls();
  resetRegion();
  route = [];
  floorDefaults();
  clearPreview();
  fit();
};
el("fit").onclick = fit;
el("auto").onclick = () => {
  before = !before;
  el("auto").setAttribute("aria-pressed", before);
  raster();
  controls();
  note(before ? "正在对照自动生成结果。" : "当前规划地图。");
};
function pickFlight(choices) {
  return new Promise((resolve) => {
    const dialog = el("flightDialog");
    el("flightChoices").replaceChildren();
    let target = null;
    for (const choice of [{ index: -1, name: "本层楼面" }, ...choices]) {
      const btn = document.createElement("button");
      btn.textContent =
        choice.index < 0
          ? choice.name
          : `楼梯 ${choice.index + 1} · ${choice.name}`;
      btn.onclick = () => {
        target = choice;
        dialog.close();
      };
      el("flightChoices").append(btn);
    }
    dialog.onclose = () => resolve(target);
    el("skipFlight").onclick = () => dialog.close();
    dialog.showModal();
  });
}
async function submitRegion(action) {
  const body = {
    epoch: state.epoch,
    floor,
    action: el("action").value,
    polygon: poly,
    z: [num("lo"), num("hi")],
    height: num("height"),
  };
  let result = await request(action, body);
  if (result.choices) {
    const choice = await pickFlight(result.choices);
    if (!choice) {
      note("已取消，选区已保留。");
      return null;
    }
    body.flight = choice.index;
    if (
      choice.index >= 0 &&
      Math.abs(body.z[0] - (atlas.floors[floor].height - 0.15)) < 0.001 &&
      Math.abs(body.z[1] - (atlas.floors[floor].height + 0.65)) < 0.001
    ) {
      body.z = [choice.lo, choice.hi];
    }
    result = await request(action, body);
    if (choice.index >= 0)
      result.scopeLabel = `楼梯 ${choice.index + 1}（${choice.name}）`;
  }
  if (result.unchanged) {
    clearPreview();
    note(result.message);
    return null;
  }
  return result;
}
function changeSummary(previous, current) {
  let gained = 0,
    lost = 0,
    supported = 0;
  for (let level = 0; level < current.floors.length; level++) {
    const old = bytes(previous.floors[level].pixels),
      next = bytes(current.floors[level].pixels);
    for (let idx = 0; idx < old.length; idx++) {
      if (old[idx] !== 2 && next[idx] === 2) gained++;
      if (old[idx] === 2 && next[idx] !== 2) lost++;
      if (old[idx] === 0 && next[idx] > 0) supported++;
    }
  }
  if (gained || lost) return `可通行格增加 ${gained} 个，减少 ${lost} 个。`;
  if (supported) return `补出了 ${supported} 个楼面格，周围的通行限制仍在。`;
  return "地图数据已调整，可通行格数量相同。";
}
el("preview").onclick = () =>
  run(async () => {
    note("正在计算预览…");
    const result = await submitRegion("preview");
    if (!result) return;
    preview = result;
    raster();
    note(
      `${preview.scopeLabel || "修改预览"} · ${changeSummary(atlas, preview.atlas)} 绿色高亮为新增可通行，红色高亮为失去可通行。`,
    );
  });
el("apply").onclick = () =>
  run(async () => {
    note("正在应用修改…");
    if (preview)
      await request("edit", { epoch: state.epoch, preview: preview.key });
    else if (!(await submitRegion("edit"))) return;
    const previous = atlas;
    resetRegion();
    first = null;
    lastPair = null;
    route = [];
    await reload();
    note(`已应用修改 · ${changeSummary(previous, atlas)}`);
  });
function step(action) {
  run(async () => {
    await request(action, { epoch: state.epoch });
    resetRegion();
    route = [];
    first = null;
    await reload();
    note(action === "undo" ? "已撤销上一步。" : "已重做。");
  });
}
el("undo").onclick = () => step("undo");
el("redo").onclick = () => step("redo");
document.onkeydown = (e) => {
  if (
    (e.ctrlKey || e.metaKey) &&
    e.key.toLowerCase() === "z" &&
    !["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement.tagName)
  ) {
    e.preventDefault();
    const action = e.shiftKey ? "redo" : "undo";
    if (!el(action).disabled) step(action);
  }
};
function drawCloud() {
  if (!cloud) return;
  const canvas = el("cloud"),
    c = canvas.getContext("2d"),
    w = canvas.clientWidth,
    h = canvas.clientHeight;
  canvas.width = w * devicePixelRatio;
  canvas.height = h * devicePixelRatio;
  c.scale(devicePixelRatio, devicePixelRatio);
  const [yaw, pitch, zoom] = orbit,
    sc = (Math.min(w, h) * zoom) / (cloud.radius * 2.6),
    floorZ = atlas.floors[floor].height;
  function project(x, y, z) {
    x -= cloud.x;
    y -= cloud.y;
    z -= floorZ;
    const along = x * Math.sin(yaw) + y * Math.cos(yaw);
    return [
      w / 2 + (x * Math.cos(yaw) - y * Math.sin(yaw)) * sc,
      h * 0.57 - (z * Math.cos(pitch) + along * Math.sin(pitch)) * sc,
    ];
  }
  if (!cloud.highlight) {
    const selection = new Path2D();
    poly.forEach(([x, y], idx) =>
      idx ? selection.lineTo(x, y) : selection.moveTo(x, y),
    );
    selection.closePath();
    const hit = document.createElement("canvas").getContext("2d");
    const lo = num("lo"),
      hi = num("hi");
    cloud.highlight = new Uint8Array(cloud.points.length / 3);
    for (let i = 0; i < cloud.points.length; i += 3) {
      const pts = cloud.points;
      cloud.highlight[i / 3] =
        pts[i + 2] >= lo &&
        pts[i + 2] <= hi &&
        (!poly.length || hit.isPointInPath(selection, pts[i], pts[i + 1]));
    }
  }
  for (
    let i = 0;
    i < cloud.points.length;
    i += orbitDown ? 3 * Math.max(1, Math.ceil(cloud.points.length / 60000)) : 3
  ) {
    const x = cloud.points[i],
      y = cloud.points[i + 1],
      z = cloud.points[i + 2];
    if (Math.abs(z - floorZ) > Math.max(3, cloud.radius)) continue;
    c.fillStyle = cloud.highlight[i / 3] ? "#23825c" : "#879898";
    const pt = project(x, y, z);
    c.fillRect(...pt, 1.6, 1.6);
  }
  if (poly.length >= 3) {
    c.strokeStyle = "#3472ae";
    for (const z of [num("lo"), num("hi")]) {
      c.beginPath();
      poly.forEach(([x, y], i) => {
        const p = project(x, y, z);
        if (i) c.lineTo(...p);
        else c.moveTo(...p);
      });
      c.closePath();
      c.stroke();
    }
    for (const [x, y] of poly.filter(
      (_, i) => i % Math.max(1, Math.ceil(poly.length / 16)) === 0,
    )) {
      c.beginPath();
      c.moveTo(...project(x, y, num("lo")));
      c.lineTo(...project(x, y, num("hi")));
      c.stroke();
    }
  }
}
el("inspect").onclick = () =>
  run(async () => {
    const center = poly.length
      ? poly.reduce(
          (acc, p) => [
            acc[0] + p[0] / poly.length,
            acc[1] + p[1] / poly.length,
          ],
          [0, 0],
        )
      : probe;
    const radius = Math.min(
      32,
      Math.max(2, ...poly.map((p) => distance(p, center) + 0.3)),
    );
    const response = await fetch(
      `cloud?x=${center[0]}&y=${center[1]}&radius=${radius}`,
    );
    if (!response.ok) throw Error("读取局部点云失败");
    cloud = {
      x: center[0],
      y: center[1],
      radius,
      points: new Float32Array(await response.arrayBuffer()),
    };
    el("dialog").showModal();
    drawCloud();
  });
el("close").onclick = () => el("dialog").close();
el("cloud").onpointerdown = (e) => {
  orbitDown = [e.clientX, e.clientY];
  e.target.setPointerCapture(e.pointerId);
};
el("cloud").onpointermove = (e) => {
  if (orbitDown) {
    orbit[0] += (e.clientX - orbitDown[0]) * 0.012;
    orbit[1] = Math.max(
      -1.55,
      Math.min(1.55, orbit[1] + (e.clientY - orbitDown[1]) * 0.012),
    );
    orbitDown = [e.clientX, e.clientY];
    drawCloud();
  }
};
el("cloud").onlostpointercapture = () => (orbitDown = null);
el("cloud").onwheel = (e) => {
  e.preventDefault();
  orbit[2] = Math.max(
    0.1,
    Math.min(20, orbit[2] * Math.exp(-0.002 * e.deltaY)),
  );
  drawCloud();
};
try {
  await reload(true);
  chooseTool("pan");
  fit();
  new ResizeObserver(draw).observe(map);
  note("规划地图已载入。");
} catch (e) {
  note(e.message, true);
}
