"""Local web workspace for importing point clouds and editing PCT maps."""

import argparse
import hashlib
import json
import os
import pickle
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback
import uuid
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np
import yaml

from floor_review import read_selection
from studio_geometry import (
    FloorMap,
    checked_edit,
    flights_at,
    in_polygon,
    make_revision,
    plan,
    prepare,
    read_json,
    save_json,
    same_map,
    tile_points,
)
from studio_support import support_patch

TOOLS = Path(__file__).resolve().parent
LOCK = threading.RLock()
BUILD = threading.Lock()


def token():
    return uuid.uuid4().hex


def initialize(project, name):
    prepare(project, name)
    save_json(project / "state.json", dict(status="ready", epoch=token()))


def generate(project, rows, filtered):
    meta = read_json(project / "meta.json")
    floors, bands = read_selection(rows, meta["bounds"], np.load(project / "z.npy", mmap_mode="r"))
    extent = np.asarray(meta["clean_extent" if filtered else "extent"])
    dims = np.ceil(extent[:2] / 0.1) + 4
    layers = max(1, int(np.ceil(extent[2] / 0.5)) + 1) if len(floors) > 1 else 1
    if dims.prod() * layers > 95_000_000 or dims.prod() > 16_000_000:
        raise ValueError("该范围超出当前整图 PCT 的内存预算。请核对离群点、单位，或分区导入。")
    generation = project / token()
    generation.mkdir()
    state = read_json(project / "state.json")
    previous_edits = []
    if rows.get("replay") and "gen" in state:
        previous_edits = read_json(project / state["gen"] / state["history"][state["cursor"]] / "edits.json")
        if previous_edits and (
            [item[0] for item in floors] != [item[0] for item in state["floors"]]
            or bands != state["floor_ranges"]
        ):
            raise ValueError("重放修正需要保持楼层标高及采样范围一致；调整范围后请重新确认编辑。")
    profile = dict(
        id="map-" + project.name[:12],
        frame="map",
        floors=floors,
        floor_ranges=bands,
        slice_reference="cloud_min",
        tile_platforms=True,
    )
    (generation / "profile.yaml").write_text(yaml.safe_dump(profile, allow_unicode=True))
    save_json(project / "state.json", dict(state, status="building", epoch=token()))

    def work():
        try:
            with BUILD, (generation / "build.log").open("w") as log:
                subprocess.run(
                    [
                        sys.executable,
                        str(TOOLS / "build_pcd_map.py"),
                        str(project / ("filtered.pcd" if filtered else "finite.pcd")),
                        str(generation / "base"),
                        "--profile",
                        str(generation / "profile.yaml"),
                    ],
                    check=True,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            base = generation / "base"
            source = project / "localization.pcd"
            source = source if source.exists() else project / "original.pcd"
            (base / "source.pcd").unlink()
            os.link(source, base / "source.pcd")
            report = read_json(base / "report.json")
            report["planning_source_sha256"] = report["source_sha256"]
            report["source_sha256"] = file_hash(source)
            report["original_sha256"] = file_hash(project / "original.pcd")
            report["filter_isolated"] = filtered
            save_json(base / "report.json", report)
            save_json(base / "edits.json", [])
            history = ["base"]
            if previous_edits:
                old = read_json(project / state["gen"] / "base/report.json")
                if any("flight" in item for item in previous_edits) and old["stairs"] != report["stairs"]:
                    raise ValueError("新旧梯段几何不同，请重新确认楼梯修正")
                for idx in range(len(previous_edits)):
                    key = token()
                    dest = generation / key
                    dest.mkdir()
                    make_revision(project, generation, dest, previous_edits[: idx + 1])
                    history.append(key)
            with LOCK:
                save_json(
                    project / "state.json",
                    dict(
                        status="done",
                        gen=generation.name,
                        floors=floors,
                        floor_ranges=bands,
                        manifest=str(generation / history[-1] / "map.yaml"),
                        history=history,
                        cursor=len(history) - 1,
                        epoch=token(),
                    ),
                )
        except Exception as error:
            log = generation / "build.log"
            detail = log.read_text(errors="replace").strip().splitlines() if log.exists() else []
            with LOCK:
                save_json(
                    project / "state.json",
                    dict(
                        state,
                        status="error",
                        message="生成失败：" + (detail[-1] if detail else str(error)),
                        epoch=token(),
                    ),
                )

    threading.Thread(target=work, daemon=True).start()


def file_hash(path):
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atlas_payload(bundle, floors):
    atlas = FloorMap(str(bundle / "planning.pickle"), floors)
    display = read_json(bundle / "display.json")
    return dict(json.loads(atlas.payload), manual=display.get("manual_runs", []), stairs=display["stairs"])


class Handler(BaseHTTPRequestHandler):
    def send(self, body, kind="application/json; charset=utf-8", code=200):
        if kind.startswith("application/json"):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def file(self, path, kind, download=False):
        with path.open("rb") as stream:
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(path.stat().st_size))
            if download:
                self.send_header("Content-Disposition", 'attachment; filename="navigation-map.zip"')
            self.end_headers()
            shutil.copyfileobj(stream, self.wfile)

    def handle_request(self, post):
        target = urlsplit(self.path)
        path = target.path
        query = parse_qs(target.query)
        store = self.server.store
        if post:
            host = self.headers.get("Host", "")
            origin = urlsplit(self.headers.get("Origin", "")).netloc
            if origin != host or host not in (
                f"127.0.0.1:{self.server.server_port}",
                f"localhost:{self.server.server_port}",
            ):
                return self.send({"message": "请从本机工作室提交操作"}, code=403)
        if path == "/" and not post:
            return self.file(TOOLS / "studio_home.html", "text/html; charset=utf-8")
        if path == "/projects" and not post:
            records = []
            with LOCK:
                for entry in store.glob("*/meta.json"):
                    if not re.fullmatch("[0-9a-f]{32}", entry.parent.name):
                        continue
                    state = read_json(entry.parent / "state.json")
                    records.append(
                        dict(
                            id=entry.parent.name,
                            generated="gen" in state,
                            epoch=state["epoch"],
                            building=state["status"] == "building",
                            imported=entry.stat().st_mtime,
                            **read_json(entry),
                        )
                    )
            records.sort(key=lambda record: record["imported"], reverse=True)
            return self.send(records)
        if path == "/import" and post:
            size = int(self.headers.get("Content-Length", 0))
            name = unquote(self.headers.get("X-Filename", "map.pcd"))
            if not name.lower().endswith(".pcd") or size <= 0:
                raise ValueError("请选择有效的 PCD 文件")
            with tempfile.TemporaryDirectory(dir=store) as temp:
                project = Path(temp) / "upload"
                project.mkdir()
                with (project / "original.pcd").open("wb") as stream:
                    while size:
                        block = self.rfile.read(min(size, 1024 * 1024))
                        if not block:
                            raise ValueError("上传中断")
                        size -= len(block)
                        stream.write(block)
                initialize(project, Path(name).name)
                identifier = token()
                project.rename(store / identifier)
            return self.send(dict(url=f"/p/{identifier}/"))
        match = re.fullmatch(r"/p/([0-9a-f]{32})/([a-z]*)", path)
        if not match:
            raise FileNotFoundError(path)
        project = store / match[1]
        action = match[2]
        statepath = project / "state.json"
        state = read_json(statepath)
        if not post:
            if action == "":
                return self.file(TOOLS / "floor_review.html", "text/html; charset=utf-8")
            if action == "editor":
                return self.file(TOOLS / "studio_editor.html", "text/html; charset=utf-8")
            if action == "editorjs":
                return self.file(TOOLS / "studio_editor.js", "text/javascript; charset=utf-8")
            if action == "map":
                return self.send(dict(read_json(project / "meta.json"), state=state))
            if action == "status":
                return self.send(state)
            if action == "points":
                return self.file(project / "overview.bin", "application/octet-stream")
            if action == "count":
                z = np.load(project / "z.npy", mmap_mode="r")
                return self.send(
                    dict(
                        count=int(
                            z.searchsorted(float(query["hi"][0]), side="right")
                            - z.searchsorted(float(query["lo"][0]))
                        )
                    )
                )
        if post:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 512 * 1024:
                raise ValueError("请求大小无效")
            body = json.loads(self.rfile.read(size))
            if action == "delete":
                with tempfile.TemporaryDirectory(dir=store) as temp:
                    with LOCK:
                        current = read_json(statepath)
                        if current["status"] == "building" or body.get("epoch") != current["epoch"]:
                            return self.send(dict(message="项目正在生成或已更新，请刷新列表后重试"), code=409)
                        project.rename(Path(temp) / "deleted")
                return self.send(dict(deleted=match[1]))
            if action == "confirm":
                with LOCK:
                    if read_json(statepath)["status"] == "building":
                        raise ValueError("正在生成地图")
                    generate(project, body, bool(body.get("filtered", True)))
                return self.send(dict(status="building"), code=202)
        if "gen" not in state:
            raise ValueError("请先生成规划地图")
        gen = project / state["gen"]
        bundle = gen / state["history"][state["cursor"]]
        floors = state["floors"]
        if not post:
            if action == "atlas":
                chosen = gen / "base" if query.get("base") == ["1"] else bundle
                edits = read_json(chosen / "edits.json")
                for edit in edits:
                    edit.pop("support_cells", None)
                return self.send(
                    dict(
                        atlas=atlas_payload(chosen, floors),
                        state=state,
                        edits=edits,
                    )
                )
            if action == "routes":
                return self.send(read_json(gen / "routes.json") if (gen / "routes.json").exists() else [])
            if action == "cloud":
                coords = np.array([float(query[key][0]) for key in ("x", "y")])
                radius = float(query.get("radius", ["3"])[0])
                if not np.isfinite(coords).all() or not 0 < radius <= 32:
                    raise ValueError("局部范围无效")
                pts = tile_points(project, coords - radius, coords + radius)
                pts = pts[:: max(1, int(np.ceil(len(pts) / 90000)))]
                return self.send(pts.tobytes(), "application/octet-stream")
            if action == "probe":
                xy = np.array([float(query[key][0]) for key in ("x", "y")])
                floor = int(query["floor"][0])
                atlas = FloorMap(str(bundle / "planning.pickle"), floors)
                if not 0 <= floor < len(floors) or not np.isfinite(xy).all() or not atlas.contains_base(xy):
                    raise ValueError("请在地图内检查")
                cell = tuple((np.rint((xy - atlas.center) / atlas.resolution).astype(int) + atlas.offset))
                heights, allowed = atlas.surfaces[floor]
                level = heights[cell]
                if not np.isfinite(level):
                    return self.send(dict(info="当前规划地图在此格没有有效楼面。"))
                center = (np.array(cell) - atlas.offset) * atlas.resolution + atlas.center
                half = atlas.resolution * 0.5
                pts = tile_points(project, center - half, center + half)
                z = pts[:, 2] - level
                hits = int(((z > 0.07) & (z <= 0.65)).sum())
                status = "可通行" if allowed[cell] else "受限"
                return self.send(
                    dict(info=f"{status} · 楼面 {level:.3f} m · 本格原始离地 7–65 cm 回波 {hits} 个。")
                )
            if action == "export":
                with tempfile.TemporaryDirectory(dir=gen) as temp:
                    archive = Path(temp) / "map.zip"
                    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=1) as out:
                        for entry in bundle.iterdir():
                            out.write(entry, entry.name)
                        out.write(gen / "profile.yaml", "profile.yaml")
                        if (project / "localization.pcd").exists():
                            out.write(project / "original.pcd", "original.pcd")
                        if (gen / "routes.json").exists():
                            out.write(gen / "routes.json", "routes.json")
                    return self.file(archive, "application/zip", True)
        if post and action == "plan":
            return self.send(plan(bundle, floors, body["points"]))
        if post and action == "routes":
            with LOCK:
                if body.get("epoch") != read_json(statepath)["epoch"]:
                    raise ValueError("地图已变更，请刷新")
                routes = read_json(gen / "routes.json") if (gen / "routes.json").exists() else []
                name = body.get("name", "").strip()
                if not 1 <= len(name) <= 64:
                    raise ValueError("请输入 1–64 字的路线名称")
                routes = [item for item in routes if item["name"] != name]
                if not body.get("remove"):
                    endpoints = body["points"]
                    plan(bundle, floors, endpoints)
                    routes.append(dict(name=name, points=endpoints))
                save_json(gen / "routes.json", routes)
            return self.send(routes)
        if post and action in ("edit", "preview", "undo", "redo"):
            with LOCK:
                state = read_json(statepath)
                if state["epoch"] != body.get("epoch") or state["status"] == "building":
                    return self.send(dict(message="地图版本已更新，请刷新后重试"), code=409)
                history = state["history"]
                cursor = state["cursor"]
                if action == "edit" and body.get("preview"):
                    key = body["preview"]
                    ticket = read_json(gen / "preview.json")
                    if ticket != dict(epoch=state["epoch"], key=key, format=2):
                        raise ValueError("请重新预览选区")
                    history = history[: cursor + 1] + [key]
                    cursor += 1
                elif action in ("edit", "preview"):
                    with (bundle / "planning.pickle").open("rb") as stream:
                        current = pickle.load(stream)
                    checked = checked_edit(body, floors, current)
                    stairs = read_json(gen / "base/report.json")["stairs"]
                    choices = flights_at(checked, stairs, floors)
                    if choices and "flight" not in body:
                        return self.send(dict(choices=choices))
                    if body.get("flight", -1) != -1:
                        if body["flight"] not in [item["index"] for item in choices]:
                            raise ValueError("请选择选区内的梯段")
                        checked["flight"] = body["flight"]
                    if checked["action"] == "fill" and "flight" not in checked:
                        low, high = checked["z"]
                        if not low <= checked["height"] <= high:
                            raise ValueError("楼面标高必须在选区的高度范围内")
                    if checked["action"] == "fill" and len(floors) > 1:
                        flight = stairs[checked["flight"]] if "flight" in checked else None
                        cells = support_patch(
                            current,
                            checked,
                            flight,
                            floors,
                            in_polygon,
                            lambda low, high: tile_points(project, low, high),
                        )
                        if not cells:
                            message = (
                                "所选梯段没有可修补的小缺口。请完整圈住缺口，并确认同一级踏面两侧都有采样。"
                                if flight
                                else "选区内没有需要补建且有楼层依据的缺测格。已测楼面会保留，请核对所选楼层和高度。"
                            )
                            return self.send(dict(unchanged=True, message=message))
                        checked["native_support"] = True
                        checked["support_version"] = read_json(gen / "base/report.json")["version"]
                        checked["support_cells"] = cells
                    del current
                    edits = read_json(bundle / "edits.json") + [checked]
                    key = token()
                    with tempfile.TemporaryDirectory(dir=gen) as temp:
                        dest = Path(temp) / "result"
                        dest.mkdir()
                        make_revision(project, gen, dest, edits)
                        if same_map(bundle, dest):
                            message = "地图没有变化，本次未计入历史。"
                            if checked["action"] == "free":
                                message += "请检查是否已可通行，或限制来自选区外。若现场确认缺测处有连续楼面，可用「补建地面」。"
                            return self.send(dict(unchanged=True, message=message))
                        dest.rename(gen / key)
                    if action == "preview":
                        previous = (
                            read_json(gen / "preview.json") if (gen / "preview.json").exists() else None
                        )
                        save_json(gen / "preview.json", dict(epoch=state["epoch"], key=key, format=2))
                        if previous and previous["key"] not in history:
                            shutil.rmtree(gen / previous["key"], ignore_errors=True)
                        return self.send(dict(key=key, atlas=atlas_payload(gen / key, floors)))
                    history = history[: cursor + 1] + [key]
                    cursor += 1
                else:
                    cursor += 1 if action == "redo" else -1
                    if not 0 <= cursor < len(history):
                        raise ValueError("此处没有可撤销或重做的操作")
                state.update(
                    history=history,
                    cursor=cursor,
                    epoch=token(),
                    manifest=str(gen / history[cursor] / "map.yaml"),
                )
                save_json(statepath, state)
            return self.send(state)
        raise FileNotFoundError(path)

    def dispatch(self, post):
        try:
            self.handle_request(post)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except FileNotFoundError:
            self.send(dict(message="项目或资源不存在"), code=404)
        except (ValueError, KeyError, TypeError, IndexError) as err:
            self.send(dict(message=str(err)), code=400)
        except Exception as err:
            traceback.print_exc()
            self.send(dict(message=f"处理失败：{err}"), code=500)

    def do_GET(self):
        self.dispatch(False)

    def do_POST(self):
        self.dispatch(True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    parser.add_argument("--port", type=int, default=8846)
    args = parser.parse_args()
    store = args.workspace.resolve()
    store.mkdir(parents=True, exist_ok=True)
    for entry in store.glob("*/state.json"):
        state = read_json(entry)
        if state["status"] == "building":
            save_json(entry, dict(state, status="error", message="上次生成被中断，可以重新生成"))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.store = store
    server.serve_forever()
