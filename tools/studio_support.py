"""Local repairs to sampled surfaces of a native multi-slice PCT map."""

import sys
from types import SimpleNamespace

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, find_objects, label


def support_patch(data, edit, flight, floors, in_polygon, read_points):
    layers = data["data"][3]
    shape = np.array(layers.shape[1:])
    res = data["resolution"]
    origin = data["center"] - shape // 2 * res
    vertices = (np.asarray(edit["polygon"]) - origin) / res
    reach = int(np.ceil(0.4 / res)) + 2
    start = np.maximum(0, np.floor(vertices.min(axis=0)).astype(int) - reach)
    stop = np.minimum(shape, np.ceil(vertices.max(axis=0)).astype(int) + reach + 1)
    region = tuple(slice(first, last) for first, last in zip(start, stop))
    xy = origin + (np.indices(stop - start).transpose(1, 2, 0) + start) * res
    inside = in_polygon(edit["polygon"], xy.reshape(-1, 2)).reshape(xy.shape[:2])
    if not inside.any():
        return []
    expected = np.full(inside.shape, edit["height"])
    tolerance = 0.10
    if flight:
        side = np.array([np.cos(flight["angle"]), np.sin(flight["angle"])])
        fraction = (xy @ [-side[1], side[0]] - flight["start"]) / (flight["end"] - flight["start"])
        low, high = (floors[flight[key]][0] for key in ("floor", "to"))
        expected = low + (high - low) * fraction
        tolerance = min(0.22, 1.5 * (high - low) / flight["count"])
        footprint = (abs(xy @ side - flight["x"]) <= flight["width"] / 2) & (fraction >= 0) & (fraction <= 1)
        inside &= footprint
    local = layers[(slice(None), *region)].astype(float)
    exists = np.any(abs(local - expected) <= tolerance, axis=0)
    if flight:
        exists &= footprint
    missing = inside & ~exists
    if not missing.any():
        return []
    pts = read_points(xy[0, 0] - res / 2, xy[-1, -1] + res / 2)
    index = np.rint((pts[:, :2] - data["center"]) / res).astype(int) + shape // 2 - start
    use = ((index >= 0) & (index < stop - start)).all(axis=1)
    index, z = index[use], pts[use, 2]
    raw = {}
    relevant = missing[tuple(index.T)]
    for cell, elevation in zip(index[relevant], z[relevant]):
        raw.setdefault(tuple(cell), []).append(elevation)
    neighbors = np.ones((3, 3), bool)
    components, _ = label(~exists, neighbors)
    boxes = find_objects(components)
    result = []
    for identifier in np.unique(components[inside & ~exists]):
        box = boxes[identifier - 1]
        enclosed = all(
            axis.start > 0 and axis.stop < size and (axis.stop - axis.start) * res <= 0.400001
            for axis, size in zip(box, inside.shape)
        )
        if flight and not enclosed:
            continue
        corner = np.array([max(0, axis.start - 1) for axis in box])
        area = tuple(
            slice(first, min(size, axis.stop + 1)) for first, size, axis in zip(corner, inside.shape, box)
        )
        gap = components[area] == identifier
        if flight and not inside[area][gap].all():
            continue
        rim = binary_dilation(gap, neighbors) & ~gap
        for layer_index, plane in enumerate(local):
            layer = plane[area]
            supports = rim & (abs(layer - expected[area]) <= tolerance)
            if not supports.any() or (flight and not supports[rim].all()):
                continue
            nearest = (
                None
                if flight
                else distance_transform_edt(~supports, return_distances=False, return_indices=True)
            )
            for cell in np.argwhere(gap & inside[area]):
                pos = tuple(cell)
                if flight:
                    samples = []
                    for sign in (-1, 1):
                        for offset in range(1, reach):
                            probe = tuple(cell + np.rint(side * sign * offset).astype(int))
                            if not all(0 <= coord < size for coord, size in zip(probe, gap.shape)):
                                break
                            if not gap[probe]:
                                if (
                                    footprint[area][probe]
                                    and abs(layer[probe] - expected[area][pos]) <= tolerance
                                ):
                                    samples.append(layer[probe])
                                break
                    if len(samples) != 2:
                        continue
                else:
                    anchor = tuple(nearest[:, pos[0], pos[1]])
                    if abs(layer[anchor] - edit["height"]) > 0.04 and not supports[rim].all():
                        continue
                    samples = layer[supports] if enclosed and supports[rim].all() else [edit["height"]]
                if np.ptp(samples) > 0.04:
                    continue
                height = float(np.median(samples))
                if not edit["z"][0] <= height <= edit["z"][1] or layer[pos] >= height - tolerance:
                    continue
                global_cell = cell + corner + start
                ceiling = data["data"][4, layer_index, global_cell[0], global_cell[1]]
                if np.isfinite(ceiling) and ceiling < height + 0.50:
                    continue
                measured = np.asarray(raw.get(tuple(cell + corner), []))
                if ((measured > height - 0.35) & (measured < height - 0.07)).any():
                    continue
                if ((measured > height + 0.07) & (measured < height + 0.50)).any():
                    continue
                result.append([layer_index, *map(int, global_cell), height])
    return result


def apply_support(data, patches, root):
    if not patches:
        return
    import cupy as cp

    tomography = root / "src/pct_planner/tomography"
    for path in (tomography, tomography / "scripts"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from config import SceneBuilding
    from tomogram import Tomogram

    def costs(mapping, ground, ceiling):
        mapping.clearMap()
        ground = cp.asarray(np.nan_to_num(ground, nan=-1e6), dtype=cp.float32)[None]
        ceiling = cp.asarray(np.nan_to_num(ceiling, nan=1e6), dtype=cp.float32)[None]
        dx = cp.maximum((ground[:, 1:-1] - ground[:, :-2]) ** 2, (ground[:, 1:-1] - ground[:, 2:]) ** 2)
        dy = cp.maximum(
            (ground[:, :, 1:-1] - ground[:, :, :-2]) ** 2, (ground[:, :, 1:-1] - ground[:, :, 2:]) ** 2
        )
        mapping.grad_mag_sq[:, 1:-1, 1:-1] = dx[:, :, 1:-1] + dy[:, 1:-1]
        mapping.grad_mag_max[:, 1:-1, 1:-1] = cp.maximum(dx[:, :, 1:-1], dy[:, 1:-1])
        trav = mapping.trav_cost
        mapping.trav_kernel(ceiling - ground, mapping.grad_mag_sq, mapping.grad_mag_max, trav, size=trav.size)
        distance = cp.zeros_like(trav)
        mapping.dist_transform_kernel(trav, distance, size=trav.size)
        mapping.adjust_cost_kernel(trav, distance, trav, size=trav.size)
        mapping.inflation_kernel(trav, mapping.inf_table, mapping.inflated_cost, size=trav.size)
        return mapping.inflated_cost[0].get()

    for layer_index in sorted({key[0] for key in patches}):
        plane = data["data"][:, layer_index]
        cells = np.array([key[1:] for key in patches if key[0] == layer_index])
        grid = SimpleNamespace(resolution=data["resolution"], slice_dh=data["slice_dh"])
        mapping = Tomogram(SimpleNamespace(map=grid, trav=SceneBuilding.trav))
        pad = 2 * mapping.half_inf_k_size + mapping.half_trav_k_size + 3
        old_ground = plane[3].copy()
        cost_changed = np.zeros(old_ground.shape, bool)
        blocks = set()
        for cell in cells:
            first = np.maximum(0, cell - pad) // 128
            last = np.minimum(np.array(old_ground.shape) - 1, cell + pad) // 128
            blocks.update(
                (row, col) for row in range(first[0], last[0] + 1) for col in range(first[1], last[1] + 1)
            )
        for block in sorted(blocks):
            core = np.array(block) * 128
            stop = np.minimum(plane.shape[1:], core + 128)
            start = np.maximum(0, core - 2 * pad)
            end = np.minimum(plane.shape[1:], stop + 2 * pad)
            region = tuple(slice(first, last) for first, last in zip(start, end))
            selected = cells[(cells >= start).all(axis=1) & (cells < end).all(axis=1)]
            local = tuple((selected - start).T)
            old = old_ground[region].astype("f4")
            new = old.copy()
            new[local] = [patches[(layer_index, *cell)] for cell in selected]
            mapping.initMappingEnv([0.0, 0.0], *old.shape, 1, 0.0)
            ceiling = plane[4][region].astype("f4")
            before, after = costs(mapping, old, ceiling), costs(mapping, new, ceiling)
            dest = tuple(slice(first, last) for first, last in zip(core, stop))
            src = tuple(slice(first, last) for first, last in zip(core - start, stop - start))
            changed = before[src] != after[src]
            plane[0][dest][changed] = after[src][changed]
            plane[3][dest] = new[src]
            cost_changed[dest] = changed
        values = plane[0].astype("f4")
        dx = cost_changed[2:] | cost_changed[:-2]
        dy = cost_changed[:, 2:] | cost_changed[:, :-2]
        plane[1, 1:-1][dx] = (values[2:] - values[:-2])[dx]
        plane[2, :, 1:-1][dy] = (values[:, 2:] - values[:, :-2])[dy]
