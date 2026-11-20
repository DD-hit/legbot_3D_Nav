"""Point tiles, bounded floor edits and native PCT exports for the map studio."""

import base64
import hashlib
import io
import json
import math
import os
import pickle
import sys
from types import SimpleNamespace

import numpy as np
import yaml
from PIL import Image
from matplotlib.path import Path as Polygon
from scipy.ndimage import binary_dilation, distance_transform_edt, find_objects, label, maximum_filter, minimum_filter
from scipy.spatial import cKDTree

from build_floor_display import FloorMap, read_pcd, root
from map_match_core import reference_index_path
from floor_review import candidates
from studio_support import apply_support, support_patch


def read_json(path):
    return json.loads(path.read_text())


def save_json(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False))
    os.replace(temp, path)


def write_cloud(path, points):
    count = len(points)
    header = f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n"
    with path.open("wb") as stream:
        stream.write(header.encode())
        points.astype("<f4", copy=False).tofile(stream)


def prepare(project, name):
    source = project / "original.pcd"
    try:
        raw = read_pcd(source)
    except ValueError as error:
        if "encoding must" not in str(error):
            raise
        import open3d as o3d

        raw = np.asarray(o3d.io.read_point_cloud(str(source)).points)
        write_cloud(project / "localization.pcd", raw)
    valid = np.isfinite(raw).all(axis=1)
    pts = raw[valid].astype("<f4", copy=False)
    if len(pts) < 3:
        raise ValueError("有效 XYZ 点不足，请检查 PCD 文件和单位")
    if np.abs(pts).max() > 1e8:
        raise ValueError("点坐标超出局部地图表示范围，请核对单位和坐标系")
    tiles = np.floor(pts[:, :2] / 8).astype("i4")
    order = np.lexsort((tiles[:, 1], tiles[:, 0]))
    tiles = tiles[order]
    pts = pts[order]
    starts = np.r_[0, np.flatnonzero((tiles[1:] != tiles[:-1]).any(axis=1)) + 1]
    ends = np.r_[starts[1:], len(pts)]
    (project / "tiles").mkdir()
    for start, end in zip(starts, ends):
        row, col = tiles[start]
        np.save(project / "tiles" / f"{row}_{col}.npy", pts[start:end])
    voxels, inverse, counts = np.unique(
        np.floor(pts / 1.0).astype("i4"), axis=0, return_inverse=True, return_counts=True
    )
    neighbors = cKDTree(voxels).query_ball_point(voxels, 3.0, return_length=True)
    isolated = ((neighbors == 1) & (counts <= 2))[inverse]
    clean = pts[~isolated]
    if len(clean) < 3:
        clean = pts
        isolated[:] = False
    np.save(project / "isolated.npy", pts[isolated])
    write_cloud(project / "finite.pcd", pts)
    write_cloud(project / "filtered.pcd", clean)
    np.save(project / "z.npy", np.sort(pts[:, 2].astype(float)))
    rng = np.random.default_rng(164)
    preview = pts if len(pts) <= 250000 else pts[rng.choice(len(pts), 250000, replace=False)]
    preview.tofile(project / "overview.bin")
    zmin, zmax = clean[:, 2].min(), clean[:, 2].max()
    if float(zmax - zmin) > 10000:
        raise ValueError("高度跨度超过 10 km，请检查单位、坐标原点或离群点")
    bounds = [
        math.floor(float(pts[:, 2].min()) * 1000) / 1000,
        math.ceil(float(pts[:, 2].max()) * 1000) / 1000,
    ]
    if bounds[0] == bounds[1]:
        bounds = [bounds[0] - 0.001, bounds[1] + 0.001]
    result = dict(
        source=name,
        count=len(pts),
        invalid=int((~valid).sum()),
        isolated=int(isolated.sum()),
        tile_count=len(starts),
        bounds=bounds,
        candidates=candidates(clean),
        extent=np.ptp(pts, axis=0).tolist(),
        clean_extent=np.ptp(clean, axis=0).tolist(),
    )
    save_json(project / "meta.json", result)


def tile_points(project, low, high):
    chunks = []
    for row in range(math.floor(low[0] / 8), math.floor(high[0] / 8) + 1):
        for col in range(math.floor(low[1] / 8), math.floor(high[1] / 8) + 1):
            path = project / "tiles" / f"{row}_{col}.npy"
            if path.exists():
                pts = np.load(path, mmap_mode="r")
                chunks.append(pts[((pts[:, :2] >= low) & (pts[:, :2] <= high)).all(axis=1)])
    return np.concatenate(chunks) if chunks else np.empty((0, 3), "f4")


def checked_edit(body, floors, data):
    floor = body.get("floor")
    action = body.get("action")
    polygon = np.asarray(body.get("polygon"), float)
    if (
        type(floor) is not int
        or not 0 <= floor < len(floors)
        or action not in ("free", "obstacle", "deny", "restore", "fill")
    ):
        raise ValueError("请选择有效楼层和操作")
    if (
        polygon.ndim != 2
        or polygon.shape[1] != 2
        or not 3 <= len(polygon) <= 4096
        or not np.isfinite(polygon).all()
    ):
        raise ValueError("请画出有效的选区")
    shape = np.array(data["data"].shape[-2:])
    res = data["resolution"]
    origin = data["center"] - shape // 2 * res
    if ((polygon < origin - 2 * res) | (polygon > origin + shape * res + 2 * res)).any():
        raise ValueError("选区超出规划地图边界")
    low, high = body["z"]
    level = float(body.get("height", floors[floor][0]))
    if (
        not np.isfinite([low, high, level]).all()
        or low >= high
    ):
        raise ValueError("请检查选区上下限和补建标高")
    centers = np.asarray([item[0] for item in floors])
    boundaries = (centers[1:] + centers[:-1]) / 2
    if action == "fill" and (
        abs(level - centers[floor]) > 0.75 or np.searchsorted(boundaries, level) != floor
    ):
        raise ValueError("补建的地面标高必须属于当前楼层")
    edit = dict(floor=floor, action=action, polygon=polygon.tolist(), z=[low, high], height=level)
    if action == "free":
        edit["repair_small_gaps"] = True
        edit["use_edited_floor"] = True
    if action == "fill" and len(floors) == 1:
        edit["missing_only"] = True
    return edit


def in_polygon(vertices, xy):
    poly = np.asarray(vertices)
    cross = np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - poly[:, 1] * np.roll(poly[:, 0], -1))
    return Polygon(poly if cross > 0 else poly[::-1]).contains_points(xy, radius=1e-8)


def flights_at(edit, flights, floors):
    poly = Polygon(np.vstack([edit["polygon"], edit["polygon"][0]]), closed=True)
    result = []
    for index, flight in enumerate(flights):
        if edit["floor"] not in (flight["floor"], flight["to"]):
            continue
        angle = flight["angle"]
        rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
        left = flight["x"] - flight["width"] / 2
        right = left + flight["width"]
        start, end = flight["start"], flight["end"]
        corners = (
            np.array([[left, start], [right, start], [right, end], [left, end], [left, start]]) @ rotation
        )
        if poly.intersects_path(Polygon(corners, closed=True), filled=True):
            lower, upper = (floors[flight[key]] for key in ("floor", "to"))
            result.append(
                dict(index=index, name=f"{lower[1]} ↔ {upper[1]}", lo=lower[0] - 0.1, hi=upper[0] + 0.65)
            )
    return result


def flight_heights(base, flight, floors, origin, resolution):
    rows, cols = base.shape[-2:]
    xx = origin[0] + np.arange(rows)[:, None] * resolution
    yy = origin[1] + np.arange(cols) * resolution
    angle = flight["angle"]
    across = xx * np.cos(angle) + yy * np.sin(angle)
    along = -xx * np.sin(angle) + yy * np.cos(angle)
    fraction = (along - flight["start"]) / (flight["end"] - flight["start"])
    inside = (abs(across - flight["x"]) <= flight["width"] / 2) & (fraction >= 0) & (fraction <= 1)
    low = floors[flight["floor"]][0]
    rise = floors[flight["to"]][0] - low
    expected = low + rise * fraction
    selected = np.full((rows, cols), np.nan)
    best = np.full((rows, cols), max(0.1, 1.5 * abs(rise) / flight["count"]))
    for heights in base[3]:
        diff = abs(heights - expected)
        better = inside & (diff <= best)
        best[better] = diff[better]
        selected[better] = heights[better]
    return selected


def repair_small_gaps(heights, inside, layers, lowest, resolution, low, high, repaired_layers):
    missing = ~np.isfinite(heights)
    if not (missing & inside).any():
        return
    neighbors = np.ones((3, 3), bool)
    components, _ = label(missing, neighbors)
    boxes = find_objects(components)
    for identifier in np.unique(components[inside & missing]):
        box = boxes[identifier - 1]
        if any(side.start == 0 or side.stop == size or (side.stop - side.start) * resolution > 0.401
               for side, size in zip(box, heights.shape)):
            continue
        area = tuple(slice(side.start - 1, side.stop + 1) for side in box)
        gap = components[area] == identifier
        if not inside[area][gap].all():
            continue
        rim = binary_dilation(gap, neighbors) & ~gap
        support = heights[area][rim]
        if np.ptp(support) > 0.075:
            continue
        elevation = float(np.median(support))
        if not low <= elevation <= high or (lowest[area][gap] < elevation - 0.08).any():
            continue
        for index, layer in enumerate(layers):
            local = layer[area]
            if np.isfinite(local[gap]).any() or not np.all(abs(local[rim] - support) < 0.07):
                continue
            repaired_layers[index][area][gap] = True
            heights[area][gap] = elevation


def update_region(project, data, base, bounds, floor, edits, manifest, selected, footprint, raw_tile):
    res = data["resolution"]
    shape = np.array(base.shape[-2:])
    origin = data["center"] - shape // 2 * res
    start, stop = bounds
    region = (slice(start[0], stop[0]), slice(start[1], stop[1]))
    grid = np.indices(stop - start).transpose(1, 2, 0)
    xy = origin + (grid + start) * res
    shape2 = xy.shape[:2]
    level = manifest["floors"][floor][0]
    original = selected[region].astype(float)
    original[~np.isfinite(original)] = np.nan
    heights = original.copy()
    reference = np.where(np.isfinite(original), original, level)
    pts = tile_points(project, xy[0, 0] - res / 2, xy[-1, -1] + res / 2)
    indices = np.rint((pts[:, :2] - data["center"]) / res).astype(int) + shape // 2 - start
    valid = ((indices >= 0) & (indices < shape2)).all(axis=1)
    pts = pts[valid]
    indices = indices[valid]
    cell = tuple(indices.T)
    layers = base[3][(slice(None), *region)]
    repaired_layers = np.zeros(layers.shape, bool)
    lowest = np.full(shape2, np.inf)
    np.minimum.at(lowest, cell, pts[:, 2])
    removed = np.zeros(len(pts), bool)
    manual = np.zeros(shape2, bool)
    touched = np.zeros(shape2, bool)
    artificial_wall = np.zeros(shape2, bool)
    for edit in edits:
        poly = edit["polygon"]
        low, high = edit["z"]
        inside = in_polygon(poly, xy.reshape(-1, 2)).reshape(shape2)
        hit = inside[cell] & (pts[:, 2] >= low) & (pts[:, 2] <= high)
        action = edit["action"]
        edit_reference = reference
        if action == "free" and edit.get("use_edited_floor"):
            edit_reference = np.where(np.isfinite(heights), heights, level)
        apply = inside & (edit_reference <= high) & (edit_reference + 0.65 >= low)
        if "flight" in edit:
            apply &= np.isfinite(original)
            hit &= apply[cell]
        if action == "restore":
            removed[hit] = False
            manual[apply] = False
            artificial_wall[apply] = False
            heights[apply] = original[apply]
            touched[apply] = False
        elif action in ("free", "fill"):
            if edit.get('missing_only'):
                missing = apply & ~np.isfinite(heights)
                heights[missing] = edit['height']
                touched |= missing
                continue
            relative = pts[:, 2] - edit_reference[cell]
            removed[hit & (relative > 0.07) & (relative <= 0.65)] = True
            manual[apply] = False
            artificial_wall[apply] = False
            touched |= apply
            if action == "fill":
                heights[apply] = edit["height"]
            elif edit.get("repair_small_gaps") and "flight" not in edit:
                repair_small_gaps(heights, apply, layers, lowest, res, low, high, repaired_layers)
        else:
            manual[apply] = True
            artificial_wall[apply] = action == "obstacle"
            touched |= apply
    near = maximum_filter(touched, size=2 * math.ceil((footprint + 0.45) / res) + 1)
    floorref = np.where(np.isfinite(heights), heights, level)
    relative = pts[:, 2] - floorref[cell]
    ceiling = np.full(shape2, np.inf)
    overhead = (relative > 0.07) & ~removed
    np.minimum.at(ceiling, tuple(indices[overhead].T), pts[overhead, 2])
    ceiling[~np.isfinite(ceiling)] = np.nan
    physical = np.zeros(shape2, bool)
    physical[tuple(indices[overhead & (relative <= 0.65)].T)] = True
    rough = (
        maximum_filter(np.nan_to_num(heights, nan=-100), 3)
        - minimum_filter(np.nan_to_num(heights, nan=100), 3)
    ) > 0.075
    blocked = manual | physical | rough | ~np.isfinite(heights)
    relaxation = np.zeros(shape2, bool)
    relaxation[tuple(indices[removed].T)] = True
    relaxation |= np.isfinite(heights) & (~np.isfinite(original) | (abs(heights - original) > 0.001))
    relaxation = maximum_filter(relaxation, size=2 * math.ceil((footprint + 0.45) / res) + 1)
    clearance = distance_transform_edt(np.pad(~blocked, 1))[1:-1, 1:-1] * res - res / np.sqrt(2)
    if len(layers) == 1 and all(edit.get('missing_only') or edit['action'] == 'restore' for edit in edits):
        original_physical = np.zeros(shape2, bool)
        relative_before = pts[:, 2] - reference[cell]
        original_physical[tuple(indices[(relative_before > 0.07) & (relative_before <= 0.65)].T)] = True
        original_rough = (
            maximum_filter(np.nan_to_num(original, nan=-100), 3)
            - minimum_filter(np.nan_to_num(original, nan=100), 3)
        ) > 0.075
        original_blocked = original_physical | original_rough | ~np.isfinite(original)
        original_clearance = distance_transform_edt(np.pad(~original_blocked, 1))[
            1:-1, 1:-1
        ] * res - res / np.sqrt(2)
        relaxation &= (clearance != original_clearance) | (np.isfinite(heights) & ~np.isfinite(original))
    cost = np.where(clearance >= footprint, 1 + 15 * np.maximum(0, 1 - (clearance - footprint) / 0.4), 100)
    manual_clearance = (
        distance_transform_edt(~manual) * res - res / np.sqrt(2) if manual.any() else np.full(shape2, np.inf)
    )
    manual_cost = np.where(
        manual_clearance >= footprint, 1 + 15 * np.maximum(0, 1 - (manual_clearance - footprint) / 0.4), 100
    )
    baseline = base[(slice(None), slice(None), *region)]
    target = data["data"][(slice(None), slice(None), *region)]
    layers = baseline[3].astype(float)
    same = np.isfinite(layers) & np.isfinite(original) & (np.abs(layers - reference) < 0.07)
    if len(layers) == 1:
        same[:] = True
    else:
        repaired = repaired_layers & np.isfinite(heights)
        same |= repaired
        fill = np.isfinite(heights) & ~np.isfinite(original) & ~repaired.any(axis=0)
        match = np.nanmedian(layers, axis=(1, 2))
        nearest = np.argmin(np.where(np.isfinite(match), abs(match - level), np.inf))
        same[nearest] |= fill
    for layer in range(len(layers)):
        change = same[layer] & near
        if not change.any():
            continue
        dest = target[:, layer]
        adjusted = np.where(relaxation, cost, np.maximum(manual_cost, baseline[0, layer]))
        dest[0, change] = adjusted[change]
        dest[3, change] = heights[change]
        dest[4, change & relaxation] = ceiling[change & relaxation]
        gx = np.zeros(shape2)
        gy = np.zeros(shape2)
        values = dest[0].astype(float)
        gx[1:-1] = values[2:] - values[:-2]
        gy[:, 1:-1] = values[:, 2:] - values[:, :-2]
        gradient = maximum_filter(change, size=3)
        dest[1, gradient] = gx[gradient]
        dest[2, gradient] = gy[gradient]
    structure = raw_tile.copy()
    structure[touched & np.isfinite(heights) & (structure == 0)] = 1
    lost = np.zeros(shape2, bool)
    lost[tuple(indices[removed].T)] = True
    if lost.any():
        votes = np.zeros(shape2, "u1")
        for offset in (0.30, 0.52, 0.74, 0.96):
            hit = (~removed) & (pts[:, 2] >= level + offset) & (pts[:, 2] < level + offset + 0.22)
            band = np.zeros(shape2, bool)
            band[tuple(indices[hit].T)] = True
            votes += band
        structure[lost & (votes < 2)] = np.where(np.isfinite(heights[lost & (votes < 2)]), 1, 0)
    structure[artificial_wall] = 2
    return structure, np.where(artificial_wall, 1, np.where(manual, 2, 0)).astype("u1")


def make_revision(project, generation, dest, edits):
    source = generation / "base"
    manifest = yaml.safe_load((source / "map.yaml").read_text())
    report = read_json(source / "report.json")
    with (source / "planning.pickle").open("rb") as stream:
        data = pickle.load(stream)
    patches = {}
    owners = {}
    for edit in edits:
        scope = ("flight", edit["flight"]) if "flight" in edit else ("floor", edit["floor"])
        cells = []
        if edit.get("native_support"):
            if edit.get("support_version") == report["version"]:
                cells = edit["support_cells"]
            else:
                flight = report["stairs"][edit["flight"]] if "flight" in edit else None
                cells = support_patch(
                    data,
                    edit,
                    flight,
                    manifest["floors"],
                    in_polygon,
                    lambda low, high: tile_points(project, low, high),
                )
        for layer, row, col, height in cells:
            key = (layer, row, col)
            patches[key] = height
            owners[key] = scope
        if edit["action"] == "restore":
            keys = [
                key
                for key in patches
                if owners[key] == scope and edit["z"][0] <= patches[key] <= edit["z"][1]
            ]
            if keys:
                offset = np.array(data["data"].shape[-2:]) // 2
                xy = (np.array(keys)[:, 1:] - offset) * data["resolution"] + data["center"]
                for key, hit in zip(keys, in_polygon(edit["polygon"], xy)):
                    if hit:
                        del patches[key]
    apply_support(data, patches, root)
    base = data["data"]
    data["data"] = base.copy()
    shape = np.array(base.shape[-2:])
    res = data["resolution"]
    origin = data["center"] - shape // 2 * res
    robot = yaml.safe_load((root / "config/robots/go2.yaml").read_text())
    footprint = robot["footprint_radius"] + abs(robot["footprint_offset"])
    geometry_path = source / "planning.pickle"
    if patches:
        geometry_path = dest / "planning.pickle"
        with geometry_path.open("wb") as stream:
            pickle.dump(data, stream, protocol=4)
    atlas = FloorMap(str(geometry_path), manifest["floors"])
    tile_size = 128
    pad = math.ceil((2 * footprint + 1.1) / res)
    display = read_json(source / "display.json")
    image = Image.open(io.BytesIO(base64.b64decode(display["png"])))
    sheet = np.array(image)
    for floor, (surface, _) in enumerate(atlas.surfaces):
        plane = sheet[floor * shape[1]:(floor + 1) * shape[1]][::-1].T
        for _, row, col in patches:
            if plane[row, col] == 0 and np.isfinite(surface[row, col]):
                plane[row, col] = 1
    original_sheet = sheet.copy()
    annotations = np.zeros((len(manifest["floors"]), *shape), "u1")
    ordinary = [edit for edit in edits if not edit.get("native_support")]
    for floor, flight_index in sorted({(edit["floor"], edit.get("flight", -1)) for edit in ordinary}):
        layer_edits = [
            edit for edit in ordinary if edit["floor"] == floor and edit.get("flight", -1) == flight_index
        ]
        tiles = set()
        selected = (
            atlas.surfaces[floor][0]
            if flight_index < 0
            else flight_heights(base, report["stairs"][flight_index], manifest["floors"], origin, res)
        )
        for edit in layer_edits:
            coords = (np.asarray(edit["polygon"]) - origin) / res
            low = np.maximum(0, np.floor(coords.min(0)).astype(int) - pad) // tile_size
            high = np.minimum(shape - 1, np.ceil(coords.max(0)).astype(int) + pad) // tile_size
            tiles.update(
                (row, col) for row in range(low[0], high[0] + 1) for col in range(low[1], high[1] + 1)
            )
        for tile in sorted(tiles):
            start = np.array(tile) * tile_size
            stop = np.minimum(start + tile_size, shape)
            outer = np.maximum(0, start - pad)
            end = np.minimum(shape, stop + pad)
            region = (slice(None), slice(None), slice(outer[0], end[0]), slice(outer[1], end[1]))
            before = data["data"][region].copy()
            sheet_slice = slice(floor * shape[1], (floor + 1) * shape[1])
            old_plane = original_sheet[sheet_slice][::-1].T
            plane = sheet[sheet_slice][::-1].T
            updated, marked = update_region(
                project,
                data,
                base,
                (outer, end),
                floor,
                layer_edits,
                manifest,
                selected,
                footprint,
                old_plane[region[2:]],
            )
            core = (
                slice(None),
                slice(None),
                slice(start[0] - outer[0], stop[0] - outer[0]),
                slice(start[1] - outer[1], stop[1] - outer[1]),
            )
            before[core] = data["data"][region][core]
            data["data"][region] = before
            plane[start[0] : stop[0], start[1] : stop[1]] = updated[core[2:]]
            annotation = annotations[floor, start[0] : stop[0], start[1] : stop[1]]
            np.maximum(annotation, marked[core[2:]], out=annotation)
    with (dest / "planning.pickle").open("wb") as stream:
        pickle.dump(data, stream, protocol=4)
    manifest["version"] = (
        "edit-"
        + hashlib.sha256((manifest["version"] + json.dumps(edits, sort_keys=True)).encode()).hexdigest()[:14]
    )
    atlas = FloorMap(str(dest / "planning.pickle"), manifest["floors"], manifest["id"], manifest["version"])
    painted = Image.fromarray(sheet).convert("P")
    painted.putpalette(image.getpalette())
    stream = io.BytesIO()
    painted.save(stream, format="PNG", bits=2)
    display["png"] = base64.b64encode(stream.getvalue()).decode()
    display["manual_runs"] = []
    for plane in annotations:
        flat = plane.ravel()
        breaks = np.r_[0, np.flatnonzero(flat[1:] != flat[:-1]) + 1, len(flat)]
        display["manual_runs"].append(
            [
                [int(start), int(stop), int(flat[start])]
                for start, stop in zip(breaks, breaks[1:])
                if flat[start]
            ]
        )
    display["checksums"] = [
        hashlib.sha256(base64.b64decode(f["pixels"])).hexdigest() for f in json.loads(atlas.payload)["floors"]
    ]
    save_json(dest / "display.json", display)
    os.link(source / "source.pcd", dest / "source.pcd")
    index = reference_index_path(source / 'source.pcd')
    if index.is_file():
        os.link(index, dest / index.name)
    save_json(dest / "edits.json", edits)
    report.update(
        version=manifest["version"],
        selectable_cells=[int(mask.sum()) for _, mask in atlas.surfaces],
        edit_count=len(edits),
    )
    save_json(dest / "report.json", report)
    (dest / "map.yaml").write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False))


def same_map(previous, candidate):
    old_display, new_display = (read_json(path / "display.json") for path in (previous, candidate))
    if any(old_display.get("manual_runs", [])) or any(new_display["manual_runs"]):
        if old_display.get("manual_runs") != new_display["manual_runs"]:
            return False
    if old_display["png"] != new_display["png"]:
        images = [Image.open(io.BytesIO(base64.b64decode(display["png"]))) for display in (old_display, new_display)]
        if not np.array_equal(np.asarray(images[0]), np.asarray(images[1])):
            return False
    with (previous / "planning.pickle").open("rb") as stream:
        old = pickle.load(stream)["data"]
    with (candidate / "planning.pickle").open("rb") as stream:
        new = pickle.load(stream)["data"]
    return all(np.array_equal(left, right, equal_nan=True) for channel in (0, 3, 4)
               for left, right in zip(old[channel], new[channel]))


def plan(bundle, floors, endpoints):
    library = root / "install/humble-pct/pct_planner/share/pct_planner/planner"
    if str(library) not in sys.path:
        sys.path[:0] = [str(library / "scripts"), str(library)]
    from planner_wrapper import TomogramPlanner

    atlas = FloorMap(str(bundle / "planning.pickle"), floors)
    positions = [np.array(atlas.point(point, True)) for point in endpoints]
    planner = TomogramPlanner(
        SimpleNamespace(planner=SimpleNamespace(use_quintic=False, max_heading_rate=10))
    )
    planner.loadTomogram(str(bundle / "planning"))
    route = planner.plan(*positions, allow_stationary=True)
    if route is None:
        raise ValueError("规划器未找到连通路线")
    return dict(
        path=route.tolist(), length=float(np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1).sum())
    )
