"""Review horizontal surfaces in a gravity-aligned PCD before building a PCT map."""
import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np
import yaml
from scipy.ndimage import binary_erosion, binary_opening, label
from scipy.signal import find_peaks

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS.parent / 'src/legbot_bringup/scripts'))
from map_match_core import read_pcd


def candidates(points):
    xy = points[:, :2]
    resolution = max(.16, float(np.ptp(xy, axis=0).max()) / 800)
    cells = np.floor((xy - xy.min(axis=0)) / resolution).astype(int)
    shape = tuple(cells.max(axis=0) + 1)
    flat = np.ravel_multi_index(cells.T, shape)
    z = points[:, 2]
    order = np.argsort(z)
    z, flat = z[order], flat[order]
    bin_width = .025
    bottom = float(z[0]) - bin_width
    bands = np.floor((z-bottom) / bin_width).astype(np.int64)
    occupied = np.unique(bands * math.prod(shape) + flat) // math.prod(shape)
    counts = np.bincount(occupied)
    peaks, _ = find_peaks(np.pad(counts, 1), height=max(8, .002 * counts.max()), prominence=4)
    choices = []
    for peak in peaks - 1:
        center = bottom + (peak + .5) * bin_width
        start, end = z.searchsorted([center - .023, center + .023])
        height = float(np.median(z[start:end]))
        lo, hi, ceiling = z.searchsorted([height - .035, height + .035, height + .52])
        support = np.zeros(shape, bool)
        support.flat[flat[lo:hi]] = True
        support.flat[flat[hi:ceiling]] = False
        patches, count = label(binary_opening(support, structure=np.ones((3, 3))))
        if not count:
            continue
        interior = binary_erosion(support, iterations=max(1, math.ceil(.38/resolution)))
        broad = np.unique(patches[interior])
        broad = broad[broad > 0]
        if not len(broad):
            continue
        area = np.bincount(patches.ravel())[broad].max() * resolution**2
        if area >= 1.0:
            choices.append((float(area), height))
    result = []
    for area, height in sorted(choices, reverse=True):
        if all(abs(height - item['height']) > .12 for item in result):
            result.append(dict(height=round(height, 3), area=round(area, 1)))
    return sorted(result, key=lambda item: item['height'])


def validate(rows, bounds):
    if not isinstance(rows, list) or not 1 <= len(rows) <= 256:
        raise ValueError('请至少选择一个楼层，最多可选 256 层')
    floors = []
    for row in rows:
        height, name = row
        if type(height) not in (float, int) or not math.isfinite(height) or not bounds[0] <= height <= bounds[1]:
            raise ValueError('标高超出点云范围')
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 64:
            raise ValueError('请输入 1–64 个字符的楼层名称')
        floors.append([round(height, 3), name.strip()])
    floors.sort()
    if len({row[0] for row in floors}) != len(floors) or len({row[1] for row in floors}) != len(floors):
        raise ValueError('请区分重复的楼层名称或标高')
    return floors


def read_selection(payload, bounds, elevations):
    if isinstance(payload, list):
        return validate(payload, bounds), None
    if not isinstance(payload, dict) or not isinstance(payload.get('layers'), list):
        raise ValueError('请选择楼层和地面采样范围')
    layers = payload['layers']
    if not all(isinstance(layer, dict) for layer in layers):
        raise ValueError('楼层数据无效')
    floors = validate([[layer.get('height'), layer.get('name')] for layer in layers], bounds)
    bands = []
    for layer, (height, name) in zip(sorted(layers, key=lambda item: item['height']), floors):
        ends = [layer.get(key) for key in ('lower', 'upper')]
        if not all(type(value) in (float, int) and math.isfinite(value) for value in ends):
            raise ValueError(f'{name}的上下限必须是有效数字')
        low, high = [round(value, 3) for value in ends]
        if not bounds[0] <= low < high <= bounds[1] or not low <= height <= high:
            raise ValueError(f'{name}的上下限无效，范围须包含标高')
        if bands and bands[-1][1] >= low:
            raise ValueError(f'{name}的采样范围与其他已选楼层重叠')
        if elevations.searchsorted(high, side='right') == elevations.searchsorted(low):
            raise ValueError(f'{name}的采样范围内没有点')
        bands.append([low, high])
    return floors, bands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--port', type=int, default=8798)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--map-id')
    parser.add_argument('--infer-isolated', action='store_true')
    args = parser.parse_args()
    source = args.pcd.resolve()
    with source.open('rb') as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    map_id = args.map_id or 'map-' + digest.hexdigest()[:12]
    identity = hashlib.sha256(json.dumps([digest.hexdigest(), map_id, args.frame, args.infer_isolated]).encode()).hexdigest()[:20]
    work = args.workspace.resolve() / identity
    work.mkdir(parents=True, exist_ok=True)
    points = read_pcd(source)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        parser.error('PCD 没有有效点')
    bounds = [math.floor(points[:, 2].min()*1000)/1000, math.ceil(points[:, 2].max()*1000)/1000]
    meta = dict(source=source.name, count=len(points), bounds=bounds, candidates=candidates(points))
    elevations = np.sort(points[:, 2].astype(float))
    preview_points = points
    if len(points) > 250000:
        sample = np.random.default_rng(164).choice(len(points), 250000, replace=False)
        preview_points = points[sample]
    preview = preview_points.astype('<f4').tobytes()
    del preview_points
    del points
    state_path = work / 'confirmed.json'
    state = json.loads(state_path.read_text()) if state_path.exists() else dict(status='ready')
    lock = threading.Lock()

    def build(floors, bands, job):
        try:
            with (job/'build.log').open('w') as log:
                command = [sys.executable, str(TOOLS/'build_pcd_map.py'), str(source), str(job/'bundle'), '--profile', str(job/'profile.yaml')]
                if args.infer_isolated:
                    command.append('--infer-isolated')
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            report = json.loads((job/'bundle/report.json').read_text())
            if report['source_sha256'] != digest.hexdigest():
                raise ValueError('点云文件已变化，请重新导入')
            result = dict(status='done', floors=floors, manifest=str(job/'bundle/map.yaml'))
            if bands is not None:
                result['floor_ranges'] = bands
            temp = job/'confirmed.tmp'
            temp.write_text(json.dumps(result, ensure_ascii=False))
            os.replace(temp, state_path)
        except Exception as exc:
            result = dict(status='error', floors=floors, message=f'生成失败，请检查 {job}/build.log（{exc}）')
        with lock:
            state.update(result)

    class Handler(BaseHTTPRequestHandler):
        def send_data(self, status, content_type, body):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def payload(self, status, data):
            self.send_data(status, 'application/json; charset=utf-8', json.dumps(data, ensure_ascii=False).encode())

        def do_GET(self):
            if self.path == '/':
                self.send_data(200, 'text/html; charset=utf-8', (TOOLS/'floor_review.html').read_bytes())
            elif self.path == '/points':
                self.send_data(200, 'application/octet-stream', preview)
            elif self.path in ('/map', '/status'):
                with lock:
                    data = dict(meta, state=dict(state)) if self.path == '/map' else dict(state)
                self.payload(200, data)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != '/confirm':
                return self.send_error(404)
            host = self.headers.get('Host', '')
            if host not in (f'localhost:{args.port}', f'127.0.0.1:{args.port}') or urlsplit(self.headers.get('Origin', '')).netloc != host:
                return self.send_error(403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 65536:
                    raise ValueError('无效的楼层列表')
                floors, bands = read_selection(json.loads(self.rfile.read(length)), bounds, elevations)
            except (ValueError, TypeError) as exc:
                return self.payload(400, dict(message=str(exc)))
            with lock:
                if state['status'] == 'building':
                    return self.payload(409, dict(message='正在生成地图'))
                job = work / uuid.uuid4().hex
                job.mkdir()
                profile = dict(id=map_id, frame=args.frame, floors=floors, slice_reference='cloud_min')
                if bands is not None:
                    profile['floor_ranges'] = bands
                (job/'profile.yaml').write_text(yaml.safe_dump(profile, allow_unicode=True))
                state.clear()
                state.update(status='building', floors=floors, floor_ranges=bands)
                threading.Thread(target=build, args=(floors, bands, job), daemon=True).start()
            self.payload(202, dict(status='building'))

    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
