"""Support estimates for user-confirmed floor sampling intervals."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter, minimum_filter
from scipy.spatial import Delaunay


def reconstruct_tiles(points, cells, shape, level, band, resolution, footprint):
    tile_size=192
    if np.all(shape<=tile_size*3):
        return reconstruct(points,cells,shape,level,band,resolution,footprint)
    halo=math.ceil((footprint+.4+.4)/resolution)+3
    partitions=cells//tile_size
    order=np.lexsort((partitions[:,1],partitions[:,0]));keys=partitions[order]
    starts=np.r_[0,np.flatnonzero(np.any(keys[1:]!=keys[:-1],axis=1))+1];ends=np.r_[starts[1:],len(order)]
    groups={tuple(keys[start]):order[start:end] for start,end in zip(starts,ends)}
    arrays=np.zeros((5,1,*shape),'f2');arrays[0]=100;arrays[3:]=np.nan
    inferred=np.zeros(shape,bool);active=(points[:,2]>=band[0])&(points[:,2]<=band[1]);observed=np.zeros(shape,bool);observed[tuple(cells[active].T)]=True
    targets=set()
    for row,col in groups:
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                key=(row+dx,col+dy)
                if min(key)>=0 and (np.array(key)*tile_size<shape).all():targets.add(key)
    for row,col in targets:
        start=np.array([row,col])*tile_size;stop=np.minimum(shape,start+tile_size);low=np.maximum(0,start-halo);high=np.minimum(shape,stop+halo)
        neighbors=[groups[key] for key in ((row+dx,col+dy) for dx in (-1,0,1) for dy in (-1,0,1)) if key in groups];indices=np.concatenate(neighbors)
        local=cells[indices];indices=indices[((local>=low)&(local<high)).all(axis=1)]
        supported=cells[indices[active[indices]]]
        if len(supported)<3 or np.linalg.matrix_rank(supported-supported[0])<2:continue
        values,missing,_=reconstruct(points[indices],cells[indices]-low,high-low,level,band,resolution,footprint)
        dst=tuple(slice(a,b) for a,b in zip(start,stop));src=tuple(slice(a,b) for a,b in zip(start-low,stop-low));arrays[(slice(None),slice(None),*dst)]=values[(slice(None),slice(None),*src)];inferred[dst]=missing[src]
    return arrays,inferred,dict(points=int(active.sum()),observed_cells=int(observed.sum()),selectable_cells=int((arrays[0]<=20).sum()))


def reconstruct(points, cells, shape, level, band, resolution, footprint):
    elevations_m = points[:, 2].astype(float)
    inside = (elevations_m >= band[0]) & (elevations_m <= band[1])
    flat = np.ravel_multi_index(cells[inside].T, shape)
    z = points[inside, 2]
    order = np.lexsort((z, flat))
    ids, first, counts = np.unique(flat[order], return_index=True, return_counts=True)
    heights = np.full(shape, np.nan)
    heights.flat[ids] = (z[order[first + counts//2]] + z[order[first + (counts-1)//2]]) / 2
    observed = np.isfinite(heights)
    samples = np.argwhere(observed)
    if len(samples) < 3 or np.linalg.matrix_rank(samples-samples[0]) < 2:
        raise ValueError(f'区间 {band} m 内的采样不足以构成地面')
    mesh = Delaunay(samples)
    distance = distance_transform_edt(~observed) * resolution
    queries = np.argwhere((distance > 0) & (distance <= .40))
    simplex = mesh.find_simplex(queries)
    queries, simplex = queries[simplex >= 0], simplex[simplex >= 0]
    corners = samples[mesh.simplices[simplex]]
    elevations = heights[tuple(corners.transpose(2, 0, 1))]
    span = np.linalg.norm(corners - np.roll(corners, 1, axis=1), axis=2).max(axis=1) * resolution
    keep = (span <= .40) & (np.ptp(elevations, axis=1) <= .055)
    transform = mesh.transform[simplex[keep]]
    bary = np.einsum('nij,nj->ni', transform[:, :2], queries[keep]-transform[:, 2])
    weights = np.column_stack((bary, 1-bary.sum(axis=1)))
    heights[tuple(queries[keep].T)] = (weights * elevations[keep]).sum(axis=1)
    reference = np.where(np.isfinite(heights), heights, level)
    relative = points[:, 2] - reference[tuple(cells.T)]
    obstacle = np.zeros(shape, bool)
    obstacle[tuple(cells[(relative > .07) & (relative <= .65)].T)] = True
    drop = np.zeros(shape, bool)
    drop[tuple(cells[relative < -.08].T)] = True
    heights[drop & ~observed] = np.nan
    upper = maximum_filter(np.nan_to_num(heights, nan=-100), 3)
    lower = minimum_filter(np.nan_to_num(heights, nan=100), 3)
    blocked = obstacle | (upper-lower > .075) | ~np.isfinite(heights)
    clearance = distance_transform_edt(np.pad(~blocked, 1))[1:-1, 1:-1]*resolution - resolution/np.sqrt(2)
    allowed = clearance >= footprint
    cost = np.where(allowed, 1 + 15*np.maximum(0, 1-(clearance-footprint)/.4), 100)
    ceiling = np.full(shape, np.inf)
    overhead = relative > .07
    np.minimum.at(ceiling, tuple(cells[overhead].T), points[overhead, 2])
    ceiling[~np.isfinite(ceiling)] = np.nan
    grad_x, grad_y = np.zeros_like(cost), np.zeros_like(cost)
    grad_x[1:-1] = cost[2:]-cost[:-2]
    grad_y[:, 1:-1] = cost[:, 2:]-cost[:, :-2]
    arrays = np.stack((cost, grad_x, grad_y, heights, ceiling)).astype('f2')[:, None]
    return arrays, ~observed & ~blocked, dict(points=int(inside.sum()), observed_cells=len(ids), selectable_cells=int(allowed.sum()))
