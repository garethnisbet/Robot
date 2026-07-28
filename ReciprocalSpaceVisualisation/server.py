#!/usr/bin/env python3
"""
Reciprocal Space Visualisation — HTTP Server

Serves the Three.js reciprocal-space viewer (static files) and a single JSON
API route, ``/api/dms``, that computes DMS (multiple-scattering) loci by reusing
the crystallography in the sibling ``DMS`` project (``DMSAnalysis/ts_quasi.py``).

Usage:
    pip install aiohttp
    npm install                 # provides node_modules/three
    python server.py [--port 8080]

Then open  http://localhost:8080

Everything else is client-side: the reciprocal lattice (conventional crystal)
and the 6D cut-and-project (icosahedral quasicrystal), the draggable detector
panel and the panel→Q mapping are all computed in the browser for instant
interaction, so the server only serves files and the one DMS route.
"""

import argparse
import asyncio
import json
import logging
import os
from pathlib import Path

from aiohttp import web

ROOT = Path(__file__).resolve().parent
log = logging.getLogger("recip-vis")

MAX_JSON_BYTES = 32 * 1024 * 1024


def _resolve_roots(cands):
    roots = []
    for c in cands:
        if not c:
            continue
        try:
            r = Path(c).expanduser().resolve()
        except OSError:
            continue
        if r.is_dir() and r not in roots:
            roots.append(r)
    return roots


def _env_roots(name):
    return [p.strip() for p in os.environ.get(name, "").split(os.pathsep) if p.strip()]


def _json_roots():
    """Directories ``/api/reflections`` may read reflection-list JSON from.

    This project and the sibling ``DMS`` project (where the lists are written),
    plus anything in ``RECIP_JSON_ROOTS`` (os.pathsep-separated). The server can
    bind a public interface, so it will not serve arbitrary paths.
    """
    return _resolve_roots(
        [ROOT, ROOT.parent / "DMS", Path.home() / "Dropbox/ClaudeCodeProjects/DMS",
         os.environ.get("DMS_PATH")] + _env_roots("RECIP_JSON_ROOTS"))


def _scan_roots():
    """Directories ``/api/scanmeta`` may read beamline ``.dat`` scan files from.

    Scan data lives outside this project (on the beamline store), so the
    reflection-list roots are rarely enough — point ``RECIP_SCAN_ROOTS`` at the
    data directory to let a loaded DMS session recover its azimuthal reference.
    Only sessions saved before slider-state version 3 need this; newer ones
    carry their own ``azir``.
    """
    return _resolve_roots(_env_roots("RECIP_SCAN_ROOTS")) + _json_roots()


async def index_handler(_request):
    """Serve the viewer page."""
    return web.FileResponse(ROOT / "reciprocal_space.html")


async def healthz_handler(_request):
    return web.json_response({"status": "ok"})


async def dms_handler(request):
    """Compute DMS multiple-scattering curves in reciprocal space (Phase 2).

    Reuses ``ts_quasi`` from the sibling DMS project via ``dms_compute``. Returns
    per-secondary-reflection Q-curves (the reciprocal-space "obstacles"), the
    aligned lattice points, and a schematic detector plane. See ``dms_compute``.
    """
    try:
        params = await request.json()
    except Exception:
        params = {}
    try:
        # Imported lazily so Phase-1-only installs (no numpy/scipy/DMS) still run.
        from dms_compute import compute_dms
        result = await asyncio.get_event_loop().run_in_executor(None, compute_dms, params)
        return web.json_response(result)
    except ImportError as e:
        log.warning("DMS unavailable: %s", e)
        return web.json_response(
            {"error": "DMS backend unavailable", "detail": str(e)}, status=501)
    except ValueError as e:
        return web.json_response({"error": "bad request", "detail": str(e)}, status=400)
    except Exception as e:
        log.exception("DMS computation failed")
        return web.json_response(
            {"error": "DMS computation failed", "detail": str(e)}, status=500)


async def reflections_handler(request):
    """Read a reflection-list JSON file from disk: ``POST {"path": "..."}``.

    The browser's file picker covers the usual case; this route exists so a
    DMS output can be loaded by its path (they live in the sibling project) and
    re-loaded without re-picking. Parsing is done client-side (js/reflist.js) —
    this only hands back the document. Reads are confined to `_json_roots()`.
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    raw = str(body.get("path", "")).strip()
    if not raw:
        return web.json_response({"error": "bad request", "detail": "no path given"}, status=400)

    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    try:
        path = path.resolve()
    except OSError as e:
        return web.json_response({"error": "bad path", "detail": str(e)}, status=400)

    if path.suffix.lower() != ".json":
        return web.json_response(
            {"error": "forbidden", "detail": "only .json files can be loaded"}, status=403)
    roots = _json_roots()
    if not any(path == r or r in path.parents for r in roots):
        return web.json_response(
            {"error": "forbidden",
             "detail": "path is outside the allowed roots (%s); set RECIP_JSON_ROOTS to add one"
                       % ", ".join(str(r) for r in roots)}, status=403)
    if not path.is_file():
        return web.json_response({"error": "not found", "detail": "no such file: %s" % path},
                                 status=404)
    if path.stat().st_size > MAX_JSON_BYTES:
        return web.json_response({"error": "too large", "detail": "file exceeds 32 MB"}, status=413)

    try:
        data = json.loads(path.read_text())
    except Exception as e:
        return web.json_response({"error": "bad json", "detail": str(e)}, status=400)
    return web.json_response({"name": path.name, "path": str(path), "data": data})


async def scanmeta_handler(request):
    """Recover a DMS session's beamline metadata: ``POST {scanpath, scannum,
    datapoint, datapoint0}`` → ``{lattice, energy, energy0, azir}``.

    A DMS slider state saved before version 3 records only what the sliders hold;
    the azimuthal reference direction (and the scan energy) come from the scan's
    ``.dat`` file, which the slider re-reads on restore. Without ``azir`` the
    multiple-scattering condition is computed about the wrong axis and none of
    the DMS lines land where the slider draws them, so for those files the viewer
    recovers it the same way. Version-3 states carry ``azir`` themselves and the
    viewer never calls this route for them.

    Reads are confined to ``_scan_roots()`` and to ``.dat`` files.
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    raw = str(body.get("scanpath", "")).strip()
    scannum = body.get("scannum")
    if not raw or scannum is None:
        return web.json_response(
            {"error": "bad request", "detail": "scanpath and scannum are required"}, status=400)

    try:
        path = Path(raw).expanduser().resolve() / ("%d.dat" % int(scannum))
    except (OSError, TypeError, ValueError) as e:
        return web.json_response({"error": "bad path", "detail": str(e)}, status=400)

    roots = _scan_roots()
    if not any(r in path.parents for r in roots):
        return web.json_response(
            {"error": "forbidden",
             "detail": "%s is outside the allowed roots (%s); set RECIP_SCAN_ROOTS to add one"
                       % (path.parent, ", ".join(str(r) for r in roots))}, status=403)
    if not path.is_file():
        return web.json_response({"error": "not found", "detail": "no such scan file: %s" % path},
                                 status=404)

    def _read():
        from dms_compute import scan_metadata
        return scan_metadata(str(path), body.get("datapoint"), body.get("datapoint0"))

    try:
        meta = await asyncio.get_event_loop().run_in_executor(None, _read)
    except ImportError as e:
        return web.json_response({"error": "DMS backend unavailable", "detail": str(e)}, status=501)
    except Exception as e:
        log.warning("scan metadata unreadable (%s): %s", path, e)
        return web.json_response({"error": "unreadable", "detail": str(e)}, status=400)
    return web.json_response({"path": str(path), **meta})


def build_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/healthz", healthz_handler)
    app.router.add_post("/api/dms", dms_handler)
    app.router.add_post("/api/reflections", reflections_handler)
    app.router.add_post("/api/scanmeta", scanmeta_handler)
    app.router.add_get("/", index_handler)
    # Serve js/, node_modules/, viewer.css, etc. from the project root.
    app.router.add_static("/", ROOT, show_index=False)
    return app


def main():
    parser = argparse.ArgumentParser(description="Reciprocal Space Visualisation server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    log.info("Starting server on http://%s:%s", args.host, args.port)
    log.info("Open http://localhost:%s in your browser", args.port)

    web.run_app(build_app(), host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
