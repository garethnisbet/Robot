import http.server
import json
import re
import shutil
import threading
import traceback
import webbrowser
from pathlib import Path
from functools import partial

from stitcher import (stitch_images, composite_from_seams, render_with_seams,
                      reproject_all, prepare_editor_data)

_current_panorama = {"path": None}
_editor_state = {"dir": None, "image_paths": None}
_render_cancel = threading.Event()
_editor_prepare_lock = threading.Lock()


def _editor_status():
    """Whether the editor can be opened, and if not, why.

    "ready" means the layer/seam data exists and can be served. "preparable"
    means we know the source images and can build that data on demand, which is
    the case when the viewer was launched on a folder without -e."""
    if _editor_state["dir"]:
        return {"ready": True, "preparable": False, "reason": ""}
    if _editor_state["image_paths"]:
        return {
            "ready": False,
            "preparable": True,
            "n_images": len(_editor_state["image_paths"]),
            "reason": "Editor data has not been built for these images yet.",
        }
    return {
        "ready": False,
        "preparable": False,
        "reason": "No source images are loaded — the editor needs the original "
                  "photos. Relaunch with a folder and -e, e.g. "
                  "python main.py /path/to/photos/ -e",
    }


def _content_type_for(path):
    suffix = Path(path).suffix.lower()
    return {
        ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
        ".png": "image/png", ".tif": "image/tiff",
        ".tiff": "image/tiff", ".bmp": "image/bmp",
    }.get(suffix, "application/octet-stream")


SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def parse_multipart(body, boundary):
    parts = []
    boundary_bytes = boundary.encode()
    segments = body.split(b"--" + boundary_bytes)
    for segment in segments[1:]:
        if segment.strip() == b"--" or segment.strip() == b"":
            continue
        header_end = segment.find(b"\r\n\r\n")
        if header_end == -1:
            continue
        header_block = segment[:header_end].decode("utf-8", errors="replace")
        file_data = segment[header_end + 4:]
        if file_data.endswith(b"\r\n"):
            file_data = file_data[:-2]

        filename = None
        match = re.search(r'filename="([^"]*)"', header_block)
        if match:
            filename = match.group(1)

        if filename:
            parts.append({"filename": filename, "data": file_data})
    return parts


class PanoramaHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def end_headers(self):
        # SimpleHTTPRequestHandler sends only Last-Modified, so browsers apply
        # heuristic freshness and can run a stale viewer/editor script against a
        # newer server. Never cache the app itself; the data endpoints are
        # cache-busted with ?t= where it matters.
        p = self.path.split("?")[0]
        if p.endswith(("/", ".html", ".js", ".css")):
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json_response(self, code, data):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        body = json.dumps(data).encode()
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def _serve_file(self, filepath, content_type=None):
        p = Path(filepath)
        if not p.exists():
            self.send_error(404, f"Not found: {p.name}")
            return
        data = p.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type or _content_type_for(p))
        self.send_header("Content-Length", len(data))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]

        if path == "/panorama-info":
            self._json_response(200, {"path": _current_panorama["path"]})
            return

        if path == "/panorama-image":
            if not _current_panorama["path"]:
                self.send_error(404, "No panorama loaded")
                return
            pano = Path(_current_panorama["path"])
            if pano.exists():
                self._serve_file(pano)
                return
            self.send_error(404, "Panorama not found")
            return

        if path == "/editor/status":
            self._json_response(200, _editor_status())
            return

        if path == "/editor/data":
            if not _editor_state["dir"]:
                self._json_response(404, _editor_status())
                return
            self._serve_file(Path(_editor_state["dir"]) / "metadata.json",
                           "application/json")
            return

        if path == "/editor/seams":
            if not _editor_state["dir"]:
                self._json_response(404, _editor_status())
                return
            self._serve_file(Path(_editor_state["dir"]) / "seams.png", "image/png")
            return

        if path == "/editor/composite":
            if not _editor_state["dir"]:
                self._json_response(404, _editor_status())
                return
            self._serve_file(Path(_editor_state["dir"]) / "composite.jpg")
            return

        if path == "/projects/list":
            projects_dir = Path(__file__).parent / "output" / "projects"
            projects = []
            if projects_dir.exists():
                for p in sorted(projects_dir.iterdir()):
                    if p.is_dir() and (p / "metadata.json").exists():
                        projects.append(p.name)
            self._json_response(200, {"projects": projects})
            return

        layer_match = re.match(r"^/editor/(layer|mask|source)/(\d+)$", path)
        if layer_match:
            if not _editor_state["dir"]:
                self._json_response(404, _editor_status())
                return
            kind = layer_match.group(1)
            idx = int(layer_match.group(2))
            ext = "jpg" if kind == "source" else "png"
            self._serve_file(
                Path(_editor_state["dir"]) / f"{kind}_{idx:02d}.{ext}")
            return

        super().do_GET()

    def _read_body(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = b""
        remaining = content_length
        while remaining > 0:
            chunk = self.rfile.read(min(remaining, 1024 * 1024))
            if not chunk:
                break
            body += chunk
            remaining -= len(chunk)
        return body

    def do_POST(self):
        if self.path == "/upload":
            self._handle_upload()
            return
        if self.path == "/editor/prepare":
            self._handle_editor_prepare()
            return
        if self.path == "/editor/preview":
            self._handle_editor_preview()
            return
        if self.path == "/editor/render":
            self._handle_editor_render()
            return
        if self.path == "/editor/cancel-render":
            _render_cancel.set()
            self._json_response(200, {"status": "ok"})
            return
        if self.path == "/editor/apply-rotations":
            self._handle_apply_rotations()
            return
        if self.path == "/editor/save-seams":
            self._handle_save_seams()
            return
        if self.path == "/projects/save":
            self._handle_project_save()
            return
        if self.path == "/projects/load":
            self._handle_project_load()
            return
        self.send_error(404)

    def _handle_editor_prepare(self):
        """Build editor data from the source images the viewer was launched with.

        Lets the editor be opened from a plain `python main.py <folder>` session
        without relaunching with -e. Takes minutes, so the client shows progress."""
        try:
            with _editor_prepare_lock:
                if _editor_state["dir"]:
                    self._json_response(200, {"status": "ok"})
                    return

                image_paths = _editor_state["image_paths"]
                if not image_paths:
                    self._json_response(404, _editor_status())
                    return

                editor_dir = Path(__file__).parent / "output" / "editor"
                print(f"Preparing editor data for {len(image_paths)} images...")
                meta = prepare_editor_data(image_paths, str(editor_dir))
                if meta is None:
                    self._json_response(
                        422, {"error": "Could not prepare editor data — the images "
                                       "need DJI gimbal metadata."})
                    return

                _editor_state["dir"] = str(editor_dir)
                print(f"Editor data ready in {editor_dir}")
            self._json_response(200, {"status": "ok"})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Server error preparing editor data"})

    def _handle_editor_preview(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            import cv2
            import numpy as np
            body = self._read_body()
            arr = np.frombuffer(body, dtype=np.uint8)
            seam_img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
            if seam_img is None:
                self._json_response(400, {"error": "Invalid image data"})
                return
            jpeg_data = composite_from_seams(_editor_state["dir"], seam_img)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", len(jpeg_data))
            self.end_headers()
            self.wfile.write(jpeg_data)
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Preview failed"})

    def _handle_editor_render(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            import cv2
            import numpy as np
            body = self._read_body()
            arr = np.frombuffer(body, dtype=np.uint8)
            seam_img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
            if seam_img is None:
                self._json_response(400, {"error": "Invalid image data"})
                return
            output_path = str(Path(_editor_state["dir"]).parent / "panorama.jpg")
            _render_cancel.clear()
            result = render_with_seams(_editor_state["dir"], seam_img, output_path,
                                      cancel_event=_render_cancel)
            if _render_cancel.is_set():
                self._json_response(499, {"error": "Render cancelled"})
            elif result:
                _current_panorama["path"] = str(Path(result).resolve())
                self._json_response(200, {"status": "ok", "path": result})
            else:
                self._json_response(500, {"error": "Render failed"})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Render failed"})

    def _handle_apply_rotations(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            body = self._read_body()
            data = json.loads(body)
            updates = data.get("rotations", [])
            if not updates:
                self._json_response(400, {"error": "No rotation updates"})
                return
            meta = reproject_all(_editor_state["dir"], updates)
            self._json_response(200, {"status": "ok", "n_images": meta["n_images"]})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Re-projection failed"})

    def _handle_save_seams(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            body = self._read_body()
            dest = Path(_editor_state["dir"]) / "seams.png"
            dest.write_bytes(body)
            print(f"  Seams saved ({len(body)} bytes)")
            self._json_response(200, {"status": "ok"})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Save failed"})

    def _handle_project_save(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            body = self._read_body()
            try:
                data = json.loads(body)
            except (json.JSONDecodeError, TypeError) as e:
                self._json_response(400, {"error": f"Invalid JSON: {e}"})
                return
            name = data.get("name", "").strip()
            if not name or "/" in name or "\\" in name:
                self._json_response(400, {"error": "Invalid project name"})
                return
            editor_dir = Path(_editor_state["dir"])
            projects_dir = Path(__file__).parent / "output" / "projects"
            dest = projects_dir / name
            dest.mkdir(parents=True, exist_ok=True)
            for f in editor_dir.iterdir():
                if f.is_file():
                    shutil.copy2(f, dest / f.name)
            print(f"  Project saved: {name}")
            self._json_response(200, {"status": "ok", "name": name})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Save failed"})

    def _handle_project_load(self):
        try:
            if not _editor_state["dir"]:
                self._json_response(404, {"error": "Editor not initialized"})
                return
            body = self._read_body()
            try:
                data = json.loads(body)
            except (json.JSONDecodeError, TypeError) as e:
                self._json_response(400, {"error": f"Invalid JSON: {e}"})
                return
            name = data.get("name", "").strip()
            if not name or "/" in name or "\\" in name:
                self._json_response(400, {"error": "Invalid project name"})
                return
            projects_dir = Path(__file__).parent / "output" / "projects"
            src = projects_dir / name
            if not src.exists() or not (src / "metadata.json").exists():
                self._json_response(404, {"error": "Project not found"})
                return
            editor_dir = Path(_editor_state["dir"])
            for f in src.iterdir():
                if f.is_file():
                    shutil.copy2(f, editor_dir / f.name)
            print(f"  Project loaded: {name}")
            self._json_response(200, {"status": "ok", "name": name})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Load failed"})

    def _handle_upload(self):
        try:
            content_type = self.headers.get("Content-Type", "")
            if "multipart/form-data" not in content_type:
                self._json_response(400, {"error": "Expected multipart/form-data"})
                return

            match = re.search(r'boundary="?([^"\s;]+)"?', content_type)
            if not match:
                self._json_response(400, {"error": "No boundary in Content-Type"})
                return
            boundary = match.group(1)

            content_length = int(self.headers.get("Content-Length", 0))
            body = b""
            remaining = content_length
            while remaining > 0:
                chunk = self.rfile.read(min(remaining, 1024 * 1024))
                if not chunk:
                    break
                body += chunk
                remaining -= len(chunk)
            parts = parse_multipart(body, boundary)

            valid_files = [
                p for p in parts
                if Path(p["filename"]).suffix.lower() in SUPPORTED_EXTENSIONS
            ]
            if not valid_files:
                self._json_response(400, {"error": "No valid image files received"})
                return

            output_dir = Path(__file__).parent / "output"
            output_dir.mkdir(exist_ok=True)

            if len(valid_files) == 1:
                name = Path(valid_files[0]["filename"]).name
                dest = output_dir / f"imported_{name}"
                dest.write_bytes(valid_files[0]["data"])
                _current_panorama["path"] = str(dest.resolve())
                print(f"Imported panorama: {dest.name}")
                self._json_response(200, {"status": "ok", "mode": "single"})
                return

            # Uploads are kept rather than discarded: the editor works from the
            # original photos, so throwing them away leaves a browser-imported
            # session with the editor permanently unavailable.
            uploads_dir = output_dir / "uploads"
            if uploads_dir.exists():
                shutil.rmtree(uploads_dir)
            uploads_dir.mkdir(parents=True)
            saved_paths = []
            for f in valid_files:
                p = uploads_dir / Path(f["filename"]).name
                p.write_bytes(f["data"])
                saved_paths.append(p)

            # A new source set invalidates any editor data prepared for the old
            # one; it gets rebuilt on demand from these images.
            with _editor_prepare_lock:
                _editor_state["dir"] = None
                _editor_state["image_paths"] = [str(p) for p in saved_paths]

            print(f"Stitching {len(saved_paths)} uploaded images...")
            output_path = str(output_dir / "panorama_stitched.jpg")
            result = stitch_images(saved_paths, output_path)

            if result is None:
                self._json_response(422, {"error": "Stitching failed — images may not overlap enough"})
                return

            _current_panorama["path"] = str(Path(result).resolve())
            self._json_response(200, {"status": "ok", "mode": "stitched", "count": len(saved_paths)})
        except Exception:
            traceback.print_exc()
            self._json_response(500, {"error": "Server error during upload"})

    def log_message(self, format, *args):
        if args and (str(args[0]).startswith("POST") or "/editor/" in str(args[0])):
            print(format % args)

    def log_error(self, format, *args):
        print("ERROR:", format % args)


def serve(panorama_path, port=8420, editor_dir=None, image_paths=None):
    _current_panorama["path"] = panorama_path
    if editor_dir:
        _editor_state["dir"] = str(editor_dir)
    if image_paths:
        # Remembered so the editor can be prepared on demand later, without
        # having to relaunch with -e.
        _editor_state["image_paths"] = [str(p) for p in image_paths]
    viewer_dir = Path(__file__).parent / "viewer"
    handler = partial(PanoramaHandler, directory=str(viewer_dir))
    import socketserver
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    server = ThreadedHTTPServer(("127.0.0.1", port), handler)

    page = "editor.html" if editor_dir else ""
    url = f"http://127.0.0.1:{port}/{page}"
    print(f"{'Editor' if editor_dir else 'Viewer'} running at {url}")
    print("Press Ctrl+C to stop.")

    webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        server.shutdown()
