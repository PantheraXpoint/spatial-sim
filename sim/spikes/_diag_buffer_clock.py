"""Throwaway: what, if anything, in a sensor buffer says WHICH frame it is.

Written to settle one question before `sim/observation_adapter.py` is changed.
The 2026-08-26 object-motion spike measured that the RTX lidar's
`generic-model-output` buffer changes once every six application frames and
that `get_data()` hands back the same buffer in between. Replacing the
adapter's fixed settle with "sample until the buffer actually changes" needs a
CHANGE SIGNAL, and the obvious one -- compare the contents -- is unusable if
it is the only one:

    **In a static scene two consecutive refreshes are identical.** Content
    comparison then cannot tell "the buffer has not refreshed yet" from "it
    refreshed and the world had not moved", and a source built on it would
    block forever the first time nothing happened to be moving.

So this probe holds the scene STILL on purpose and looks for a field that
ticks anyway: a frame id, a timestamp, a sequence number -- anything that
counts refreshes rather than describing the scene. It reports, for every
scalar attribute of the parsed buffer and for every candidate handle around
it, how many distinct values appeared over N frames and the run-length pattern
of those values, because a field that ticks on the lidar's cadence shows up as
runs of exactly six.

Answers three things and then this file has no further use:

    1. is there an explicit per-refresh counter, and what is it called
    2. what is the second element of `sensor.get_data(...)`
    3. does the CAMERA have a cadence of its own, or is it current every frame

Exec mode, no SimulationApp. Run::

    docker compose -f docker/docker-compose.yml run --rm -T sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_buffer_clock.py

Env: BC_FRAMES (default 72), BC_STAGE, BC_OUT.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd

REPO = Path(__file__).resolve().parent.parent.parent
SIM = REPO / "sim"
for _p in (str(REPO), str(SIM)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ.setdefault("SF_NO_AUTORUN", "1")
os.environ.setdefault("OA_NO_AUTORUN", "1")

import sensor_factory as sf  # noqa: E402
from core.observation import Modality, MountType  # noqa: E402

STAGE = os.environ.get("BC_STAGE", str(SIM / "observatory_avatar.usd"))
FRAMES = int(os.environ.get("BC_FRAMES", "72"))
OUT_DIR = Path(os.environ.get("BC_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WARMUP = 300


def log(msg: str) -> None:
    print(f"[buffer_clock] {msg}", flush=True)


def fingerprint(arr) -> str:
    """A cheap, order-sensitive digest of an array's bytes."""
    try:
        a = np.asarray(arr)
        if a.size == 0:
            return "empty"
        return hashlib.blake2b(np.ascontiguousarray(a).tobytes(),
                               digest_size=8).hexdigest()
    except Exception as exc:                                      # noqa: BLE001
        return f"<{type(exc).__name__}>"


def scalars_of(obj, skip: set[str]) -> dict:
    """Every attribute of `obj` that reads as a single number or short string.

    Read through dir() rather than a list of names, because the point is to
    find a field nobody has thought to look for. The per-element arrays are
    skipped by size, not by name.
    """
    out = {}
    for name in dir(obj):
        if name.startswith("_") or name in skip:
            continue
        try:
            value = getattr(obj, name)
        except Exception:                                         # noqa: BLE001
            continue
        if callable(value):
            continue
        if isinstance(value, (bool, int, float, str)):
            out[name] = value
            continue
        # A one-element array or a small tuple is still a scalar for this.
        try:
            arr = np.asarray(value)
        except Exception:                                         # noqa: BLE001
            continue
        if arr.ndim == 0:
            out[name] = arr.item()
        elif arr.size <= 8 and arr.dtype.kind in "iuf":
            out[name] = [round(float(v), 6) for v in arr.ravel()]
    return out


class Probe:
    def __init__(self) -> None:
        self.frame = 0
        self.warm = 0
        self.phase = "loading"
        self.ctx = omni.usd.get_context()
        self.lidar = None
        self.lidar_id = None
        self.extract = None
        self.camera_annotators: dict = {}
        self.camera_id = None
        self.rows: list[dict] = []
        self.sub = None
        self.out = OUT_DIR / "buffer_clock.json"

    # -- setup -------------------------------------------------------------
    def setup(self) -> None:
        stage = self.ctx.get_stage()
        registry = sf.load_registry()
        for path, pos in sf.create_stations(stage).items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")
        created = sf.create_registry_sensors(stage, registry)
        if not created:
            raise RuntimeError("no sensors created")

        for sensor_id, rec in created.items():
            spec = registry.get(sensor_id)
            if spec.mount is not MountType.FIXED:
                continue
            if rec["kind"] == "lidar" and self.lidar is None:
                self.lidar, self.lidar_id = rec["sensor"], sensor_id
            if rec["kind"] == "camera" and self.camera_id is None:
                self.camera_id = sensor_id
                self.camera_annotators = rec.get("annotators") or {}
        log(f"lidar={self.lidar_id} camera={self.camera_id} "
            f"annotators={list(self.camera_annotators)}")

        # The same extract annotator the adapter attaches, on the same render
        # product, so its cadence is measured rather than assumed to match.
        try:
            import omni.replicator.core as rep
            from isaacsim.core.experimental.utils.app import enable_extension

            enable_extension("isaacsim.sensors.rtx.nodes")
            rp = str(self.lidar.render_product.GetPath())
            self.extract = rep.AnnotatorRegistry.get_annotator(
                "IsaacExtractRTXSensorPointCloud")
            self.extract.attach(rp)
            log(f"extract annotator attached to {rp}")
        except Exception as exc:                                  # noqa: BLE001
            log(f"! extract annotator unavailable: {exc!r}")

        # NOTHING MOVES. No character follow, no walk, no pose writes. That is
        # the experimental condition: any field that still ticks is counting
        # refreshes rather than describing the scene, and that is exactly the
        # field the adapter needs.
        omni.timeline.get_timeline_interface().play()
        log("play() called; the scene is deliberately static from here on")

    # -- per frame ---------------------------------------------------------
    def sample(self) -> dict:
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        row: dict = {"frame": self.frame}
        buf, second = None, None
        try:
            got = self.lidar.get_data("generic-model-output")
            buf, second = got if isinstance(got, tuple) else (got, None)
        except Exception as exc:                                  # noqa: BLE001
            row["gmo_error"] = repr(exc)
            return row

        row["buf_type"] = type(buf).__name__
        row["buf_id"] = id(buf)
        row["second_type"] = type(second).__name__
        if isinstance(second, (dict, list, tuple)):
            row["second_repr"] = str(second)[:400]
        elif isinstance(second, (int, float, str, bool)) or second is None:
            row["second_value"] = second
        else:
            row["second_scalars"] = scalars_of(second, skip=set())

        gmo = parse_generic_model_output_data(buf)
        if gmo is None:
            row["gmo"] = None
            return row
        row["gmo_type"] = type(gmo).__name__
        row["gmo_scalars"] = scalars_of(gmo, skip={"x", "y", "z", "flags",
                                                   "scalar", "rv_ms"})
        n = int(getattr(gmo, "numElements", 0) or 0)
        row["numElements"] = n
        if n:
            # Content digest: what a naive "has it changed" test would see.
            row["content"] = fingerprint(np.asarray(gmo.z[:n]))

        if self.extract is not None:
            try:
                raw = self.extract.get_data()
                data = raw.get("data") if isinstance(raw, dict) else raw
                row["extract_points"] = int(np.asarray(data).shape[0]) if data is not None else 0
                row["extract_content"] = fingerprint(data)
                if isinstance(raw, dict) and isinstance(raw.get("info"), dict):
                    row["extract_info_keys"] = sorted(raw["info"])
            except Exception as exc:                              # noqa: BLE001
                row["extract_error"] = repr(exc)

        for name, ann in self.camera_annotators.items():
            try:
                data = ann.get_data()
                if isinstance(data, dict):
                    data = data.get("data")
                row[f"cam_{name}"] = fingerprint(data)
            except Exception as exc:                              # noqa: BLE001
                row[f"cam_{name}"] = f"<{type(exc).__name__}>"
        return row

    # -- report ------------------------------------------------------------
    def report(self) -> None:
        log(f"sampled {len(self.rows)} frames with a static scene")
        series: dict[str, list] = {}
        for row in self.rows:
            for key, value in row.items():
                if key in ("frame", "gmo_scalars", "second_scalars"):
                    continue
                series.setdefault(key, []).append(value)
            for holder in ("gmo_scalars", "second_scalars"):
                for key, value in (row.get(holder) or {}).items():
                    series.setdefault(f"{holder[:-8]}.{key}", []).append(
                        tuple(value) if isinstance(value, list) else value)

        print("\n" + "=" * 78, flush=True)
        print("WHICH FIELDS TICK WHILE THE SCENE IS STILL", flush=True)
        print("=" * 78, flush=True)
        print(f"  {'field':<34s}{'distinct':>9s}  run-lengths (first 10)", flush=True)
        summary = {}
        for key in sorted(series):
            values = series[key]
            distinct = len({str(v) for v in values})
            runs = [len(list(g)) for _, g in itertools.groupby(str(v) for v in values)]
            summary[key] = {"distinct": distinct, "runs": runs[:20],
                            "first": str(values[0])[:60]}
            mark = ""
            if distinct > 1:
                inner = runs[1:-1] or runs
                if inner and len(set(inner)) == 1:
                    mark = f"  <-- ticks every {inner[0]} frames"
                else:
                    mark = "  <-- changes, irregularly"
            print(f"  {key:<34s}{distinct:>9d}  {runs[:10]}{mark}", flush=True)
        print("=" * 78, flush=True)

        ticking = {k: v for k, v in summary.items() if v["distinct"] > 1}
        print(f"\n  fields that changed at all: {sorted(ticking)}", flush=True)
        print(f"  fields that never changed : {len(summary) - len(ticking)}", flush=True)
        try:
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            self.out.write_text(json.dumps(
                {"frames": len(self.rows), "summary": summary,
                 "rows": self.rows[:12]}, indent=1, default=str))
            log(f"wrote {self.out}")
        except Exception as exc:                                  # noqa: BLE001
            log(f"! could not write results: {exc!r}")

    # -- pump --------------------------------------------------------------
    def on_update(self, _event) -> None:
        self.frame += 1
        try:
            if self.phase == "loading":
                status = self.ctx.get_stage_loading_status()
                if self.frame > 5 and not any(status[1:]):
                    self.setup()
                    self.phase = "warmup"
                return
            if self.phase == "warmup":
                self.warm += 1
                row = self.sample()
                if row.get("numElements") or self.warm >= WARMUP:
                    log(f"first data after {self.warm} frames of warm-up")
                    self.phase = "sampling"
                return
            if self.phase == "sampling":
                self.rows.append(self.sample())
                if len(self.rows) >= FRAMES:
                    self.report()
                    self.finish()
                return
        except Exception as exc:                                  # noqa: BLE001
            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
            self.finish()

    def finish(self) -> None:
        log("DONE")
        self.sub = None
        omni.kit.app.get_app().post_quit(0)


def main() -> None:
    log(f"stage={STAGE} frames={FRAMES}")
    log(f"open_stage -> {omni.usd.get_context().open_stage(STAGE)}")
    probe = Probe()
    probe.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        probe.on_update, name="buffer_clock")


if os.environ.get("BC_NO_AUTORUN") != "1":
    main()
