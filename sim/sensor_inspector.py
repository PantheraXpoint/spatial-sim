"""S12: a text readout of what the selected sensor is actually reading.

The demo's whole claim is that heterogeneous sensors react to one moving body.
Until now there was no way to tell from inside the GUI whether any sensor saw
the avatar at all -- point counts existed only in headless logs. This puts them
on screen, next to the thing you are looking at.

Select a sensor prim; the panel shows that sensor's frame counter, point count,
min/max depth, semantic classes and timestamp, refreshed a few times a second.

Design note: :func:`read_stats` is a PURE FUNCTION of the sensor record and is
where every number comes from. The omni.ui window only formats what it returns.
That split is deliberate -- it means the panel can be verified at Play without
clicking anything, by calling read_stats directly while the avatar moves, which
is exactly how it was checked. A panel that renders beautifully and reports a
constant is the failure mode this project keeps hitting.
"""

from __future__ import annotations

import time
from typing import Any

import omni.usd
from pxr import UsdGeom

POLL_HZ = 4.0


def _lidar_stats(rec: dict, stage) -> dict:
    """Point count and depth extremes from a range sensor's live buffer."""
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

    import sensor_factory as sf

    out: dict[str, Any] = {"kind": "lidar"}
    sensor = rec.get("sensor")
    if sensor is None:
        return {**out, "error": "no sensor handle"}
    try:
        buf, _ = sensor.get_data("generic-model-output")
    except Exception as exc:
        return {**out, "error": f"get_data failed: {type(exc).__name__}"}
    if buf is None:
        return {**out, "error": "no buffer yet -- press Play"}
    gmo = parse_generic_model_output_data(buf)
    if gmo is None:
        return {**out, "error": "buffer did not parse"}

    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(rec["prim_path"]))
    dec = sf.decode_gmo(gmo, m)
    points = dec.pop("_points", None)

    # THE NUMBER THE DEMO IS ABOUT. Total point count does not answer "does
    # this sensor see me": the avatar is ~1,400 returns out of ~290,000, and
    # the total wanders +/-0.15% with scan phase, so walking about moves it
    # less than the noise does. Measured over a walk toward the sensor:
    # 290,023 / 290,376 / 290,247 / 290,102 -- indistinguishable. Likewise
    # depth_min sits pinned at the profile's 1.0 m near range regardless of
    # where anyone stands. So the panel reports returns landing INSIDE the
    # avatar's bounds, and how far away the nearest of them is.
    if points is not None:
        char = stage.GetPrimAtPath("/Root/Avatar/character")
        if char.IsValid():
            import numpy as np

            cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
            rng = cache.ComputeWorldBound(char).ComputeAlignedRange()
            lo, hi = rng.GetMin(), rng.GetMax()
            pad = 0.15
            inside = np.all(
                (points >= np.array([lo[0] - pad, lo[1] - pad, lo[2] - pad]))
                & (points <= np.array([hi[0] + pad, hi[1] + pad, hi[2] + pad])), axis=1)
            n = int(inside.sum())
            out["points_on_avatar"] = n
            if n:
                sensor_pos = np.array([m[3][0], m[3][1], m[3][2]])
                d = np.linalg.norm(points[inside] - sensor_pos, axis=1)
                out["avatar_range_m"] = round(float(d.min()), 3)
            else:
                out["avatar_range_m"] = None
    out.update({
        "points": dec.get("real", 0),
        "raw_elements": dec.get("numElements", 0),
        "depth_min_m": dec.get("range_min"),
        "depth_max_m": dec.get("range_max"),
        "sentinel": dec.get("sentinel_hits", 0),
    })
    try:
        out["timestamp_ns"] = int(gmo.timestampNs)
        out["frame"] = int(gmo.frameId)
    except Exception:
        pass
    return out


def _camera_stats(rec: dict, stage) -> dict:
    """Depth extremes and semantic classes, when annotators are attached.

    Cameras in a GUI session have no annotators by default -- each one is a
    render product, and render products were what cost the frame rate. So this
    reports honestly that there is nothing to read rather than inventing zeros.
    """
    import numpy as np

    out: dict[str, Any] = {"kind": "camera"}
    anns = rec.get("annotators") or {}
    if not anns:
        return {**out, "error": "no annotators attached (GUI panels do not create them)"}

    depth = anns.get("distance_to_camera")
    if depth is not None:
        arr = np.asarray(depth.get_data())
        finite = arr[np.isfinite(arr)] if arr.size else arr
        if finite.size:
            out["depth_min_m"] = round(float(finite.min()), 3)
            out["depth_max_m"] = round(float(finite.max()), 3)

    seg = anns.get("semantic_segmentation")
    if seg is not None:
        data = seg.get_data()
        if isinstance(data, dict) and data.get("data") is not None:
            ids = np.unique(np.asarray(data["data"]))
            labels = (data.get("info") or {}).get("idToLabels") or {}
            names = []
            for i in ids:
                entry = labels.get(str(int(i))) or labels.get(int(i))
                if isinstance(entry, dict):
                    names.append(str(entry.get("class", entry)))
                elif entry:
                    names.append(str(entry))
            out["classes"] = sorted({n for n in names if n not in ("BACKGROUND", "UNLABELLED")})

    rgb = anns.get("rgb")
    if rgb is not None:
        arr = np.asarray(rgb.get_data())
        out["pixels"] = int((arr != 0).sum()) if arr.size else 0
    return out


def read_stats(rec: dict, stage) -> dict:
    """Everything the panel displays, for one sensor record.

    Pure: no UI, no globals. This is the function the Play-time verification
    calls directly.
    """
    stats = _lidar_stats(rec, stage) if rec.get("kind") == "lidar" else _camera_stats(rec, stage)
    stats["prim_path"] = rec.get("prim_path")
    stats["wall_clock"] = round(time.time(), 2)
    return stats


def find_selected(made: dict, selected_paths: list[str]) -> tuple[str, dict] | tuple[None, None]:
    """Map the current selection onto a registered sensor.

    Accepts a selected prim at or below the sensor, because clicking a sensor
    in the viewport often selects a child.
    """
    for path in selected_paths or []:
        for sensor_id, rec in made.items():
            prim_path = rec.get("prim_path") or ""
            if path == prim_path or path.startswith(prim_path + "/"):
                return sensor_id, rec
    return None, None


def format_lines(sensor_id: str | None, stats: dict | None) -> list[str]:
    if sensor_id is None:
        return ["No sensor selected.", "", "Select a registered sensor prim",
                "(e.g. .../INFRA_01/lidar_01) in the Stage tree."]
    lines = [f"{sensor_id}", f"{stats.get('prim_path','')}", ""]
    if stats.get("error"):
        lines.append(f"! {stats['error']}")
        return lines
    order = ("frame", "points", "points_on_avatar", "avatar_range_m",
             "raw_elements", "sentinel", "pixels",
             "depth_min_m", "depth_max_m", "classes", "timestamp_ns")
    for key in order:
        if key in stats and stats[key] is not None:
            value = stats[key]
            if isinstance(value, list):
                value = ", ".join(value) if value else "(none)"
            elif isinstance(value, float):
                value = f"{value:.3f}"
            elif isinstance(value, int) and key in ("points", "raw_elements", "pixels",
                                                    "points_on_avatar"):
                value = f"{value:,}"
            lines.append(f"{key:14s} {value}")
    return lines


def install_inspector(stage, made: dict, poll_hz: float = POLL_HZ):
    """Create the panel and start polling. Returns (window, subscription).

    KEEP BOTH REFERENCES: dropping the subscription stops the readout, and
    dropping the window closes it -- silently, in both cases.
    """
    import omni.kit.app
    import omni.ui as ui

    window = ui.Window("Sensor Inspector", width=420, height=300)
    state = {"labels": [], "last": 0.0}

    with window.frame:
        with ui.VStack(spacing=3, height=0):
            ui.Label("Sensor Inspector", height=20)
            ui.Separator(height=4)
            for _ in range(14):
                state["labels"].append(ui.Label("", height=16))

    interval = 1.0 / max(0.5, poll_hz)

    def _poll(_e) -> None:
        now = time.time()
        if now - state["last"] < interval:
            return
        state["last"] = now
        try:
            paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
            sensor_id, rec = find_selected(made, paths)
            stats = read_stats(rec, stage) if rec is not None else None
            lines = format_lines(sensor_id, stats)
        except Exception as exc:
            lines = [f"! inspector error: {type(exc).__name__}: {exc}"]
        for i, label in enumerate(state["labels"]):
            label.text = lines[i] if i < len(lines) else ""

    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _poll, name="sensor_inspector"
    )
    print(f"[inspector] panel installed, polling at {poll_hz} Hz over "
          f"{len(made)} registered sensors", flush=True)
    return window, sub
