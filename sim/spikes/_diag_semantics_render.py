"""Does the 6.0.1 segmentation annotator read the DEPRECATED semantics schema?

The one question `_diag_semantics_audit.py` could not answer, because it reads
USD and renders nothing. The audit found 3,480 label entries on this stage and
**3,467 of them are on the deprecated `Semantics.SemanticsAPI`** -- only the
avatar's 13 are on the current `UsdSemantics.LabelsAPI`. So the audit's
headline is conditional:

    deprecated schema IS read   ->  semantic coverage is 98.5%
    deprecated schema NOT read  ->  semantic coverage is 0.4%, the avatar alone

Nothing in USD distinguishes those. `libomni.syntheticdata.plugin.so` carries
strings for *both* schemas, which is suggestive and is not proof. The only
thing that settles it is a render.

THE MEASUREMENT
---------------
Point a camera at the warehouse racking -- geometry whose labels are entirely
on the deprecated schema -- render, and read `idToLabels`. If `box` and `rack`
come back, the deprecated schema is read. Then point a camera at the avatar,
whose labels are entirely on the current schema, and confirm `person` comes
back: that is the positive control which proves the pipeline works at all, so
that an empty racking result means "schema not read" rather than "annotator
broken" or "nothing in frame".

**`idToLabels` alone is not enough** and is the trap this file exists to avoid.
A class can appear in the map without a single pixel carrying it. So every
probe reports PER-ID PIXEL COUNTS, and the number that actually answers the
question is the **labelled pixel fraction**: what share of the frame is
something other than `BACKGROUND`/`UNLABELLED`. That is coverage as the
annotator sees it, which is the only definition that matters to a benchmark.

Two controls per probe, because "empty" has more than one cause:

    rgb                        is anything rendering at all
    instance_id_segmentation   is the TARGET in frame -- it maps prim PATHS and
                               does not consult semantics, so it separates
                               "the annotator ignored these labels" from
                               "the camera was looking at a wall"

A THIRD PROBE, AND A CORRECTION IT MAY FORCE
--------------------------------------------
The audit reported `/Root/Worker` as 11 renderable meshes with 0 direct labels
and 11 inherited ones -- its labels sit on the parent Xforms, one level above
the meshes. From `set_default_semantic_filter(..., hierarchical_labels=False)`
being the shipped default I inferred the Worker would read as unlabelled. That
was an inference from source, not a measurement, and `s7_camera_capture.jsonl`
(2026-08-15) already disagrees with it: its `idToLabels` contains `fieldjacket`,
`basebody` and `baseballcap`, which exist nowhere but on the Worker's parent
Xforms. So the third probe frames whichever subtree the scan finds in exactly
that state -- discovered, never hardcoded -- and settles it with pixels.

REPORT ONLY
-----------
No semantics are authored, nothing is migrated, no layer is saved, and **no
prim is created under the stage root**. Probe cameras are made by
`rep.create.camera`, which puts them under `/Replicator`, and render products
land under `/Render` -- Replicator's own scratch scopes, exactly as every
capture in this project already uses. The set of prim paths under the default
prim is snapshotted before and after setup and diffed, so "the stage was not
touched" is a MEASUREMENT printed in the report rather than a promise made in
a docstring.

Targets are read off the stage: the largest prim labelled `rack`, the avatar
path from `config/scene.yaml`, and the ancestor-only subtree the scan finds.
No prim path is invented (hard rule 1).

The timeline is left STOPPED first. If the annotators fill anyway, nothing is
ever played and the scene is not perturbed at all; only if they stay empty is
`play()` called. Which of the two produced the data is reported, because
"does Replicator capture while stopped in exec mode" is worth knowing for
capture mode and costs nothing to find out here.

EXECUTION MODEL
---------------
Exec mode, per CLAUDE.md and `sim/spikes/_diag_exec.py`: no `SimulationApp`
(offscreen capture returns 0 px under it on this host -- that is the whole
reason exec mode exists), frames come from the update event stream, never
`app.update()`, and `post_quit()` when done. Results are written incrementally
and fsync'd.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_semantics_render.py

`docker stop` the container afterwards -- it does not exit on DONE, and the
next launch fails on the port it is still holding.

Environment (argv is ambiguous after ``--exec``, so config is env vars):

    SR_STAGE      stage to open      (default /workspace/sim/observatory_avatar.usd)
    SR_OUT        report directory   (default /workspace/sim/spikes/logs)
    SR_RES        probe resolution   (default 480)
    SR_FRAMES     max frames to sample after setup (default 150)
    SR_STOPPED    frames to try with the timeline STOPPED before playing (default 40)
    SR_DIRS       candidate standoff directions per target (default 12)
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

STAGE = os.environ.get("SR_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("SR_OUT", str(REPO / "sim" / "spikes" / "logs")))
FALLBACK_DIR = Path("/isaac-sim/.nvidia-omniverse/logs")
RES = int(os.environ.get("SR_RES", "480"))
MAX_FRAMES = int(os.environ.get("SR_FRAMES", "150"))
STOPPED_FRAMES = int(os.environ.get("SR_STOPPED", "40"))
N_DIRS = int(os.environ.get("SR_DIRS", "12"))
MAX_WAIT = int(os.environ.get("SR_MAX_WAIT", "3000"))

STEM = "s10_semantics_render"

# Labels the annotator uses for "no class here". Everything else is a real
# class, and the split between them IS the coverage number.
UNLABELLED_TOKENS = {"BACKGROUND", "UNLABELLED", "UNLABELED", ""}

_LOG: list[str] = []


def log(msg: str = "") -> None:
    _LOG.append(msg)
    print(f"[semrender] {msg}" if msg else "", flush=True)


def resolve_out() -> tuple[Path, str]:
    for cand, why in ((OUT_DIR, "requested (SR_OUT)"),
                      (FALLBACK_DIR, "FALLBACK -- the logs volume")):
        try:
            cand.mkdir(parents=True, exist_ok=True)
            probe = cand / f".{STEM}.probe"
            probe.write_text("x", encoding="utf-8")
            probe.unlink()
            return cand, why
        except Exception as exc:
            print(f"[semrender] {cand} not writable ({exc!r})", flush=True)
    raise RuntimeError("no writable output directory")


def durable_write(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".part")
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class Report:
    def __init__(self, out: Path) -> None:
        self.json_path = out / f"{STEM}.json"
        self.log_path = out / f"{STEM}.log"
        self.data: dict = {"schema_version": 1, "complete": False}

    def section(self, name: str, payload) -> None:
        self.data[name] = payload
        self.flush()

    def flush(self) -> None:
        durable_write(self.json_path, json.dumps(self.data, indent=1, default=str))
        durable_write(self.log_path, "\n".join(_LOG) + "\n")


# ---------------------------------------------------------------------------
# Label scan -- the compact half of _diag_semantics_audit.py, enough to CHOOSE
# targets. The audit is the inventory; this only needs to find things to look at.
# ---------------------------------------------------------------------------
LABELS_API = "UsdSemantics.LabelsAPI"
LEGACY_API = "Semantics.SemanticsAPI"


def direct_entries(prim: Usd.Prim) -> list[dict]:
    out: list[dict] = []
    applied = set(prim.GetAppliedSchemas())

    for name in (p.GetName() for p in prim.GetProperties()):
        if name.startswith("semantics:labels:"):
            tax = name[len("semantics:labels:"):]
            attr = prim.GetAttribute(name)
            out.append({"schema": LABELS_API, "taxonomy": tax,
                        "labels": [str(v) for v in (attr.Get() or [])],
                        "api_applied": f"SemanticsLabelsAPI:{tax}" in applied})

    insts = {s.split(":", 1)[1] for s in applied if s.startswith("SemanticsAPI:")}
    for prop in prim.GetProperties():
        parts = prop.SplitName()
        if len(parts) == 4 and parts[0] == "semantic" and parts[2] == "params":
            insts.add(parts[1])
    for inst in sorted(insts):
        t = prim.GetAttribute(f"semantic:{inst}:params:semanticType")
        d = prim.GetAttribute(f"semantic:{inst}:params:semanticData")
        data = str(d.Get()) if d and d.Get() is not None else ""
        out.append({"schema": LEGACY_API,
                    "taxonomy": str(t.Get()) if t and t.Get() is not None else "<unset>",
                    "labels": [s.strip() for s in data.split(",") if s.strip()],
                    "api_applied": f"SemanticsAPI:{inst}" in applied})
    return out


PRED = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
RENDERABLE_PURPOSES = {str(UsdGeom.Tokens.default_), str(UsdGeom.Tokens.render)}


def renderable(prim: Usd.Prim) -> bool:
    if not (prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.PointInstancer)):
        return False
    img = UsdGeom.Imageable(prim)
    return (str(img.ComputeVisibility()) != str(UsdGeom.Tokens.invisible)
            and str(img.ComputePurpose()) in RENDERABLE_PURPOSES)


def subtree_key(path: str, root: str) -> str:
    if root and root != "/" and path.startswith(root + "/"):
        return f"{root}/{path[len(root) + 1:].split('/', 1)[0]}"
    return path if path == root else "/" + path.lstrip("/").split("/", 1)[0]


def has_labelled_ancestor(prim: Usd.Prim, direct: dict) -> bool:
    p = prim.GetParent()
    while p and p.IsValid() and not p.IsPseudoRoot():
        if any(e["labels"] for e in direct.get(p.GetPath().pathString, [])):
            return True
        p = p.GetParent()
    return False


# ---------------------------------------------------------------------------
# Camera placement. Derived from the target's own world bbox and from where
# there is actually room to stand -- a camera parked inside a shelf renders a
# brown rectangle and would read as "the annotator returned nothing".
# ---------------------------------------------------------------------------
def point_to_box_distance(p: np.ndarray, mins: np.ndarray, maxs: np.ndarray) -> np.ndarray:
    d = np.maximum(np.maximum(mins - p, 0.0), np.maximum(p - maxs, 0.0))
    return np.linalg.norm(d, axis=1)


def choose_eye(center: Gf.Vec3d, size, mins: np.ndarray, maxs: np.ndarray,
               n_dirs: int = 12) -> tuple[tuple[float, float, float], float]:
    """Stand off far enough to frame the target, in the direction with room.

    Tries `n_dirs` compass bearings and keeps the one whose camera position is
    furthest from every renderable bounding box. Returns the eye point and the
    clearance it won, so a bad placement is visible in the report rather than
    being mistaken for a semantics result.
    """
    extent = max(float(size[0]), float(size[1]), float(size[2]))
    standoff = max(2.5, 1.9 * extent)
    best, best_clear = None, -1.0
    for i in range(n_dirs):
        a = 2.0 * np.pi * i / n_dirs
        eye = np.array([float(center[0]) + np.cos(a) * standoff,
                        float(center[1]) + np.sin(a) * standoff,
                        float(center[2]) + 0.25 * extent])
        clear = float(point_to_box_distance(eye[None, :], mins, maxs).min())
        if clear > best_clear:
            best_clear, best = clear, eye
    return (float(best[0]), float(best[1]), float(best[2])), best_clear


def make_probe_camera(position, look_at):
    """A Replicator camera. Lands under /Replicator, never under the stage root.

    focal_length and clipping_range are passed defensively: they are the two
    that matter (a narrow lens frames nothing, and USD's 1 m default near plane
    hides the warehouse) and a signature change would otherwise take the run
    down rather than degrade it.
    """
    for kwargs in ({"focal_length": 18.0, "clipping_range": (0.05, 1_000_000.0)},
                   {"focal_length": 18.0},
                   {}):
        try:
            return rep.create.camera(position=position, look_at=look_at, **kwargs), kwargs
        except TypeError:
            continue
    return rep.create.camera(position=position, look_at=look_at), {}


# ---------------------------------------------------------------------------
# Reading one segmentation frame
# ---------------------------------------------------------------------------
def label_text(value) -> str:
    """idToLabels values are {'class': 'box'} in 6.x and bare strings elsewhere."""
    if isinstance(value, dict):
        return ",".join(str(v) for v in value.values())
    return str(value)


def read_semantic(ann) -> dict | None:
    seg = ann.get_data()
    if not isinstance(seg, dict) or seg.get("data") is None:
        return None
    data = np.asarray(seg["data"])
    if data.size == 0:
        return None
    raw = (seg.get("info") or {}).get("idToLabels") or {}
    ids, counts = np.unique(data, return_counts=True)
    total = int(data.size)

    per_id, labelled_px, unknown_px = [], 0, 0
    for i, c in zip(ids.tolist(), counts.tolist()):
        entry = raw.get(str(i), raw.get(i))
        text = label_text(entry) if entry is not None else None
        is_real = text is not None and text.strip() not in UNLABELLED_TOKENS
        if text is None:
            unknown_px += int(c)
        if is_real:
            labelled_px += int(c)
        per_id.append({"id": int(i), "label": text, "pixels": int(c),
                       "pixel_pct": 100.0 * c / total, "counts_as_labelled": bool(is_real)})
    per_id.sort(key=lambda r: -r["pixels"])
    classes = sorted({r["label"] for r in per_id
                      if r["label"] and r["label"].strip() not in UNLABELLED_TOKENS})
    return {
        "shape": list(data.shape),
        "idToLabels": {str(k): label_text(v) for k, v in raw.items()},
        "idToLabels_size": len(raw),
        "per_id": per_id,
        "classes_with_pixels": classes,
        "labelled_pixels": labelled_px,
        "labelled_pixel_pct": 100.0 * labelled_px / total,
        # ids in the image with no entry in the map. tests/contract.py forbids
        # exactly this for the observation adapter; worth knowing if the
        # annotator itself does it.
        "ids_missing_from_map_pixels": unknown_px,
    }


def read_instances(ann, target_path: str) -> dict | None:
    seg = ann.get_data()
    if not isinstance(seg, dict) or seg.get("data") is None:
        return None
    data = np.asarray(seg["data"])
    if data.size == 0:
        return None
    raw = (seg.get("info") or {}).get("idToLabels") or {}
    ids = {int(i) for i in np.unique(data).tolist()}
    present = {str(i): str(raw.get(str(i), raw.get(i, ""))) for i in ids}
    hit_px = 0
    for i in ids:
        path = present.get(str(i), "")
        if target_path and path.startswith(target_path):
            hit_px += int((data == i).sum())
    return {
        "distinct_instances_in_frame": len(ids),
        "target_path": target_path,
        "target_in_frame": hit_px > 0,
        "target_pixels": hit_px,
        "target_pixel_pct": 100.0 * hit_px / data.size,
        "sample_paths": sorted(set(present.values()))[:12],
    }


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------
class Run:
    def __init__(self) -> None:
        self.ctx = omni.usd.get_context()
        self.frame = 0
        self.phase = "loading"
        self.sub = None
        self.probes: list[dict] = []
        self.report: Report | None = None
        self.setup_frame = 0
        self.played = False
        self.play_frame = None
        self.root_before: set[str] = set()

    # -- setup ------------------------------------------------------------
    def setup(self) -> None:
        stage = self.ctx.get_stage()
        default_prim = stage.GetDefaultPrim()
        root = default_prim.GetPath().pathString if default_prim else "/"
        mpu = float(UsdGeom.GetStageMetersPerUnit(stage))
        log("=" * 78)
        log("DOES THE 6.0.1 ANNOTATOR READ THE DEPRECATED SEMANTICS SCHEMA?")
        log("=" * 78)
        log(f"stage        {STAGE}")
        log(f"default prim {root}   {mpu} m/unit   probes at {RES}x{RES}")

        # --- scan ---------------------------------------------------------
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                                  useExtentsHint=True)
        direct: dict[str, list[dict]] = {}
        rend: list[Usd.Prim] = []
        t0 = time.perf_counter()
        for prim in Usd.PrimRange.Stage(stage, PRED):
            e = direct_entries(prim)
            if e:
                direct[prim.GetPath().pathString] = e
            if renderable(prim):
                rend.append(prim)
        log(f"scanned: {len(direct)} labelled prims, {len(rend)} renderable, "
            f"{time.perf_counter() - t0:.1f}s")
        self.root_before = {p.GetPath().pathString for p in Usd.PrimRange.Stage(stage, PRED)
                            if p.GetPath().pathString.startswith(root)}

        # Every renderable bbox, once: the obstacle field the cameras stand in.
        mins, maxs, paths = [], [], []
        for prim in rend:
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            mins.append([float(v) * mpu for v in r.GetMin()])
            maxs.append([float(v) * mpu for v in r.GetMax()])
            paths.append(prim.GetPath().pathString)
        mins_a, maxs_a = np.asarray(mins), np.asarray(maxs)

        by_schema: dict[str, int] = {}
        for entries in direct.values():
            for e in entries:
                if e["labels"]:
                    by_schema[e["schema"]] = by_schema.get(e["schema"], 0) + 1
        log(f"label entries on {LEGACY_API}: {by_schema.get(LEGACY_API, 0)}   "
            f"on {LABELS_API}: {by_schema.get(LABELS_API, 0)}")

        # --- target 1: the largest prim labelled 'rack' (deprecated schema) --
        def labelled_with(word: str, schema: str | None = None) -> list[str]:
            out = []
            for path, entries in direct.items():
                for e in entries:
                    if schema and e["schema"] != schema:
                        continue
                    if any(word == lab.lower() for lab in e["labels"]):
                        out.append(path)
            return out

        racks = labelled_with("rack", LEGACY_API)
        boxes = labelled_with("box", LEGACY_API)
        targets: list[dict] = []

        def bbox_of(path: str):
            prim = stage.GetPrimAtPath(path)
            if not (prim and prim.IsValid()):
                return None
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                return None
            c = Gf.Vec3d(*[(float(r.GetMin()[i]) + float(r.GetMax()[i])) * 0.5 * mpu
                           for i in range(3)])
            s = [abs(float(r.GetSize()[i])) * mpu for i in range(3)]
            return c, s

        if racks:
            scored = []
            for path in racks:
                bb = bbox_of(path)
                if bb:
                    scored.append((bb[1][0] * bb[1][1] * bb[1][2], path, bb))
            scored.sort(reverse=True)
            vol, path, (c, s) = scored[0]
            # How many 'box' prims sit within 4 m of it -- so the report can say
            # what SHOULD be in frame before it says what was.
            near = 0
            for bpath in boxes:
                bb = bbox_of(bpath)
                if bb and (np.linalg.norm(np.array([float(c[i]) for i in range(3)])
                                          - np.array([float(bb[0][i]) for i in range(3)])) < 4.0):
                    near += 1
            log(f"target 'racking' -> {path}  bbox {['%.2f' % v for v in s]} m  "
                f"({len(racks)} rack prims on the stage, {near} 'box' prims within 4 m)")
            targets.append({"name": "racking", "schema_under_test": LEGACY_API,
                            "prim_path": path, "center": c, "size": s,
                            "expect": ["rack", "box"],
                            "note": f"{len(racks)} rack / {len(boxes)} box prims on the stage, "
                                    f"{near} boxes within 4 m of this one"})
        else:
            log("! no prim labelled 'rack' on the deprecated schema -- probe skipped")

        # --- target 2: the avatar (current schema) --------------------------
        import yaml

        try:
            cfg = yaml.safe_load((REPO / "config" / "scene.yaml").read_text(encoding="utf-8"))
        except Exception as exc:
            log(f"! scene.yaml unreadable ({exc!r})")
            cfg = {}
        av = (cfg.get("avatar") or {}).get("prim_path", "")
        want = str((cfg.get("avatar") or {}).get("semantic_class", "person"))
        if av and stage.GetPrimAtPath(av).IsValid():
            bb = bbox_of(av)
            if bb:
                log(f"target 'avatar'  -> {av}  bbox {['%.2f' % v for v in bb[1]]} m  "
                    f"expect {want!r}")
                targets.append({"name": "avatar", "schema_under_test": LABELS_API,
                                "prim_path": av, "center": bb[0], "size": bb[1],
                                "expect": [want],
                                "note": "positive control for the current schema"})
        else:
            log(f"! avatar path {av!r} not on the stage -- probe skipped")

        # --- target 3: whichever subtree is labelled ONLY on ancestors ------
        anc: dict[str, dict] = {}
        for prim in rend:
            path = prim.GetPath().pathString
            key = subtree_key(path, root)
            s = anc.setdefault(key, {"renderable": 0, "direct": 0, "ancestor_only": 0})
            s["renderable"] += 1
            if any(e["labels"] for e in direct.get(path, [])):
                s["direct"] += 1
            elif has_labelled_ancestor(prim, direct):
                s["ancestor_only"] += 1
        cand = [(v["ancestor_only"], k) for k, v in anc.items()
                if v["ancestor_only"] and not v["direct"]]
        cand.sort(reverse=True)
        if cand:
            _, key = cand[0]
            bb = bbox_of(key)
            anc_labels = sorted({lab for p, entries in direct.items() if p.startswith(key + "/")
                                 for e in entries for lab in e["labels"]})
            if bb:
                log(f"target 'ancestor_only' -> {key}  ({anc[key]['ancestor_only']} meshes "
                    f"labelled only via a parent)  labels present: {anc_labels}")
                targets.append({"name": "ancestor_only", "schema_under_test": LEGACY_API,
                                "prim_path": key, "center": bb[0], "size": bb[1],
                                "expect": [lab.lower() for lab in anc_labels],
                                "note": "labels sit on parent Xforms, not on the meshes -- "
                                        "does the annotator propagate them?"})
        else:
            log("no subtree is labelled only through ancestors -- probe skipped")

        # --- build the probes ------------------------------------------------
        for t in targets:
            eye, clear = choose_eye(t["center"], t["size"], mins_a, maxs_a, N_DIRS)
            look = (float(t["center"][0]), float(t["center"][1]), float(t["center"][2]))
            cam, kwargs = make_probe_camera(eye, look)
            rp = rep.create.render_product(cam, resolution=(RES, RES))
            anns = {
                "semantic_segmentation": rep.AnnotatorRegistry.get_annotator(
                    "semantic_segmentation", init_params={"colorize": False}),
                "instance_id_segmentation": rep.AnnotatorRegistry.get_annotator(
                    "instance_id_segmentation", init_params={"colorize": False}),
                "rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
            }
            for a in anns.values():
                a.attach([rp])
            self.probes.append({**t, "eye": eye, "clearance_m": clear,
                                "camera_kwargs": kwargs, "render_product": rp,
                                "annotators": anns, "best": None, "rgb_px": 0,
                                "instances": None, "filled_at_frame": None})
            log(f"  probe {t['name']:<14} eye {[round(v, 2) for v in eye]} -> "
                f"look {[round(v, 2) for v in look]}  clearance {clear:.2f} m")

        # A stage camera as an independent cross-check: apparatus attached to
        # something that was already there, aimed by whoever built the stage.
        for cam_path in ("/Root/Avatar/body_mesh/cam_third_person",):
            p = stage.GetPrimAtPath(cam_path)
            if p and p.IsValid() and p.IsA(UsdGeom.Camera):
                rp = rep.create.render_product(cam_path, resolution=(RES, RES))
                anns = {
                    "semantic_segmentation": rep.AnnotatorRegistry.get_annotator(
                        "semantic_segmentation", init_params={"colorize": False}),
                    "instance_id_segmentation": rep.AnnotatorRegistry.get_annotator(
                        "instance_id_segmentation", init_params={"colorize": False}),
                    "rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
                }
                for a in anns.values():
                    a.attach([rp])
                self.probes.append({
                    "name": "stage_camera", "schema_under_test": "both",
                    "prim_path": av or cam_path, "expect": [want],
                    "note": f"existing stage camera {cam_path}, nothing created",
                    "eye": None, "clearance_m": None, "camera_kwargs": {},
                    "render_product": rp, "annotators": anns, "best": None,
                    "rgb_px": 0, "instances": None, "filled_at_frame": None})
                log(f"  probe {'stage_camera':<14} {cam_path} (existing, not created)")

        if not self.probes:
            raise RuntimeError("no probes could be built -- nothing to measure")

        # What did building the apparatus author under the stage root?
        after = {p.GetPath().pathString for p in Usd.PrimRange.Stage(stage, PRED)
                 if p.GetPath().pathString.startswith(root)}
        created_under_root = sorted(after - self.root_before)
        log(f"prims created under {root} by this setup: "
            f"{len(created_under_root)}  {created_under_root[:6]}")
        self.report.section("setup", {
            "stage": STAGE, "default_prim": root, "meters_per_unit": mpu,
            "resolution": [RES, RES],
            "label_entries_by_schema": by_schema,
            "probes": [{k: v for k, v in p.items()
                        if k not in ("render_product", "annotators", "best",
                                     "instances", "center", "size")}
                       for p in self.probes],
            "prims_created_under_root": created_under_root,
        })
        log("")
        log(f"sampling with the timeline STOPPED for up to {STOPPED_FRAMES} frames "
            f"before playing")

    # -- sampling ---------------------------------------------------------
    def sample(self) -> bool:
        any_filled = False
        for p in self.probes:
            rgb = np.asarray(p["annotators"]["rgb"].get_data())
            if rgb.size:
                p["rgb_px"] = max(p["rgb_px"], int((rgb != 0).sum()))
            sem = read_semantic(p["annotators"]["semantic_segmentation"])
            if sem is not None:
                any_filled = True
                if p["filled_at_frame"] is None:
                    p["filled_at_frame"] = self.frame
                    p["filled_while"] = "playing" if self.played else "stopped"
                # Keep the frame that saw the most labelled pixels: the question
                # is what the annotator CAN report, so the best view answers it.
                if p["best"] is None or sem["labelled_pixels"] > p["best"]["labelled_pixels"]:
                    p["best"] = sem
                    p["instances"] = read_instances(
                        p["annotators"]["instance_id_segmentation"], p["prim_path"])
        return any_filled

    # -- report -----------------------------------------------------------
    def finish(self) -> None:
        results = []
        log("")
        for p in self.probes:
            sem, inst = p["best"], p["instances"]
            log("-" * 78)
            log(f"PROBE {p['name']}   ({p['note']})")
            log("-" * 78)
            log(f"  target        {p['prim_path']}")
            log(f"  schema tested {p['schema_under_test']}")
            log(f"  expecting     {p['expect']}")
            log(f"  rgb non-zero  {p['rgb_px']:,} px"
                + ("   <-- NOTHING RENDERED" if not p["rgb_px"] else ""))
            if inst:
                log(f"  target in frame (instance_id, semantics-independent): "
                    f"{inst['target_in_frame']}  "
                    f"{inst['target_pixels']:,} px ({inst['target_pixel_pct']:.2f}%), "
                    f"{inst['distinct_instances_in_frame']} instances visible")
            if sem is None:
                log("  semantic_segmentation NEVER FILLED -- no data to judge")
                results.append({**{k: v for k, v in p.items()
                                   if k in ("name", "prim_path", "schema_under_test",
                                            "expect", "note", "eye", "clearance_m",
                                            "rgb_px", "filled_at_frame")},
                                "semantic": None, "instances": inst, "verdict": "NO DATA"})
                continue
            found = [e for e in p["expect"]
                     if any(e.lower() == (c or "").lower() for c in sem["classes_with_pixels"])]
            missing = [e for e in p["expect"] if e not in found]
            log(f"  idToLabels    {sem['idToLabels_size']} entries; "
                f"{len(sem['classes_with_pixels'])} classes actually carry pixels")
            log(f"  LABELLED PIXELS {sem['labelled_pixels']:,} of "
                f"{sem['shape'][0] * sem['shape'][1]:,}  "
                f"({sem['labelled_pixel_pct']:.2f}% of the frame)")
            log(f"  {'pixels':>10}{'pct':>8}  class")
            for r in sem["per_id"][:18]:
                log(f"  {r['pixels']:>10,}{r['pixel_pct']:>7.2f}%  {r['label']}")
            if len(sem["per_id"]) > 18:
                log(f"  ... {len(sem['per_id']) - 18} more ids (in the JSON)")
            if sem["ids_missing_from_map_pixels"]:
                log(f"  ! {sem['ids_missing_from_map_pixels']:,} px carry an id with no "
                    f"entry in idToLabels")
            verdict = (f"FOUND {found}" if found and not missing else
                       f"PARTIAL -- found {found}, missing {missing}" if found else
                       f"NOT FOUND -- none of {p['expect']} carry a pixel")
            log(f"  VERDICT: {verdict}")
            results.append({**{k: v for k, v in p.items()
                               if k in ("name", "prim_path", "schema_under_test",
                                        "expect", "note", "eye", "clearance_m",
                                        "rgb_px", "filled_at_frame", "filled_while")},
                            "semantic": sem, "instances": inst,
                            "expected_found": found, "expected_missing": missing,
                            "verdict": verdict})
        self.report.section("probes", results)

        # --- the answer -------------------------------------------------------
        def probe(name):
            return next((r for r in results if r["name"] == name), None)

        rack, avatar, anc = probe("racking"), probe("avatar"), probe("ancestor_only")
        deprecated_read = bool(rack and rack.get("expected_found"))
        current_read = bool(avatar and avatar.get("expected_found"))
        answer = {
            "deprecated_schema_read": deprecated_read,
            "current_schema_read": current_read,
            "captured_while": next((p.get("filled_while") for p in self.probes
                                    if p.get("filled_while")), None),
            "timeline_played": self.played,
        }
        log("")
        log("=" * 78)
        log("ANSWER")
        log("=" * 78)
        if rack:
            log(f"  deprecated Semantics.SemanticsAPI  -> "
                f"{'READ' if deprecated_read else 'NOT READ'}   ({rack['verdict']})")
        else:
            log("  deprecated schema: NOT TESTED (no rack target found)")
        if avatar:
            log(f"  current UsdSemantics.LabelsAPI     -> "
                f"{'READ' if current_read else 'NOT READ'}   ({avatar['verdict']})")
        else:
            log("  current schema: NOT TESTED (no avatar on the stage)")
        if anc:
            log(f"  labels on parent Xforms only       -> {anc['verdict']}")
        if deprecated_read and current_read:
            head = ("BOTH schemas are read. The audit's 98.5% direct coverage stands, "
                    "and no migration is required for segmentation to work.")
        elif current_read and rack and not deprecated_read:
            head = ("ONLY the current schema is read. Real coverage is the avatar's 13 "
                    "prims -- 0.4%, not 98.5%. Every warehouse class is absent from the "
                    "ground truth and upgrade_prim_semantics_to_labels is required.")
        elif deprecated_read and not current_read:
            head = ("The deprecated schema is read and the CURRENT one is not, which "
                    "would mean the avatar is the thing missing from the ground truth.")
        else:
            head = ("Neither schema produced a labelled pixel. Check the rgb and "
                    "instance_id controls above before reading this as a semantics "
                    "result -- an empty frame looks identical.")
        log(f"  {head}")
        answer["headline"] = head
        log(f"  captured with the timeline {answer['captured_while'] or 'n/a'}"
            f" (play() was {'called' if self.played else 'never called'})")

        stage = self.ctx.get_stage()
        dp = stage.GetDefaultPrim()
        root = dp.GetPath().pathString if dp else "/"
        after = {p.GetPath().pathString for p in Usd.PrimRange.Stage(stage, PRED)
                 if p.GetPath().pathString.startswith(root)}
        created = sorted(after - self.root_before)
        removed = sorted(self.root_before - after)
        answer["prims_created_under_root"] = created
        answer["prims_removed_under_root"] = removed
        log(f"  prims created under {root}: {len(created)}; removed: {len(removed)}. "
            f"Nothing was saved and no semantics were authored.")
        log("=" * 78)

        self.report.section("answer", answer)
        self.report.data["complete"] = True
        self.report.data["generated_epoch"] = time.time()
        self.report.flush()
        log(f"wrote {self.report.json_path} "
            f"({self.report.json_path.stat().st_size:,} bytes) and {self.report.log_path}")
        log("DONE")

    # -- driver -----------------------------------------------------------
    def on_update(self, _e) -> None:
        self.frame += 1
        try:
            if self.phase == "loading":
                still = any(self.ctx.get_stage_loading_status()[1:])
                if self.frame <= 5 or (still and self.frame < MAX_WAIT):
                    if still and self.frame % 300 == 0:
                        print(f"[semrender] still loading at frame {self.frame}", flush=True)
                    return
                out, why = resolve_out()
                self.report = Report(out)
                log(f"report       {self.report.json_path}   ({why})")
                self.setup()
                self.setup_frame = self.frame
                self.phase = "sampling"
                return

            if self.phase != "sampling":
                return

            elapsed = self.frame - self.setup_frame
            filled = self.sample()

            # Only play if capture stays empty. A stopped stage is the least
            # perturbed one this measurement can run on.
            if not filled and not self.played and elapsed >= STOPPED_FRAMES:
                log(f"nothing captured in {elapsed} stopped frames -- calling play()")
                omni.timeline.get_timeline_interface().play()
                self.played = True
                self.play_frame = self.frame
                return

            done = elapsed >= MAX_FRAMES
            if filled and self.played and self.play_frame is not None:
                done = done or (self.frame - self.play_frame) >= MAX_FRAMES // 2
            if filled and not self.played and elapsed >= STOPPED_FRAMES:
                done = True
            if not done:
                return

            self.phase = "reporting"
            self.finish()
            self.sub = None
            omni.kit.app.get_app().post_quit()
        except Exception as exc:
            import traceback

            print(f"[semrender] FAILED: {exc!r}", flush=True)
            print(traceback.format_exc(), flush=True)
            try:
                if self.report is not None:
                    self.report.flush()
            except Exception:
                pass
            self.sub = None
            omni.kit.app.get_app().post_quit()


RUN = Run()
print(f"[semrender] open_stage {STAGE} -> "
      f"{omni.usd.get_context().open_stage(STAGE)}", flush=True)
RUN.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    RUN.on_update, name="semantics_render"
)
