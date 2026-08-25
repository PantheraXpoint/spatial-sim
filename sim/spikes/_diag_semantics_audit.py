"""Semantic labelling audit of the observatory stage. REPORT ONLY.

Missing semantic labels is failure mode 6 in CLAUDE.md: segmentation and bbox
annotators return **empty, with no error**. Nothing in the project has ever
measured how much of this stage is labelled, and the one datum we have points
the wrong way -- the S5 reconnaissance (tasks/SERVER.md) found **11** labelled
prims on `full_warehouse_worker_and_anim_cameras.usd`, all of them parts of the
Worker character, against a plan that assumed "roughly 60% pre-labelled". This
answers the question properly, for the stage as it is actually composed.

Why it matters more here than in a demo: the deliverable is an embodied
navigation and exploration benchmark (CLAUDE.md hard rule 6). Partial semantic
coverage does not show up as a gap in the ground truth -- it shows up as ground
truth that is confidently wrong about a scene where most surfaces simply have
no name. An agent scored against it is scored against a warehouse in which the
racking, the floor and the walls officially do not exist.

WHAT IT DOES NOT DO
-------------------
Creates no prim, authors no attribute, applies no schema, saves no layer, and
loads no payload that the stage did not already load. Every USD call below is
a read. The stage is opened, measured, and abandoned; nothing is written but
the report. If you want the labels *fixed*, that is S10 and it is a different
script.

WHAT "LABELLED" MEANS HERE, AND THE TWO SCHEMAS
----------------------------------------------
Two incompatible semantics schemas ship in 6.0.1 and both are in circulation --
the same API-era trap CLAUDE.md warns about, one layer down. Verified by
reading the shipped `generatedSchema.usda` of each, not recalled:

    UsdSemantics.LabelsAPI      current. Multiple-apply `SemanticsLabelsAPI:<tax>`
    (omni.usd.libs)             carrying `token[] semantics:labels:<tax>`.

    Semantics.SemanticsAPI      deprecated. Multiple-apply `SemanticsAPI:<inst>`
    (omni.usd.schema.semantics) carrying `string semantic:<inst>:params:semanticType`
                                and `...:semanticData`. `pxr.Semantics` now warns
                                on import; the module is top-level `Semantics`.

A third case is reported separately and is the nastiest of the three: the
**attributes present with the API schema never applied**. That prim looks
labelled in the stage tree and in a text dump of the layer, and
`UsdSemantics.LabelsAPI.Get()` does not see it.

The upgrade path between the two, if S10 wants it, is
`isaacsim.core.experimental.utils.semantics.upgrade_prim_semantics_to_labels(
prim, include_descendants=True)` -- old `semanticType` becomes the new taxonomy
and old `semanticData` becomes the label.

DIRECT VERSUS INHERITED, AND WHY THE HEADLINE NUMBER IS THE DIRECT ONE
----------------------------------------------------------------------
USD's own rule is that labels inherit down namespace: `UsdSemantics.LabelsQuery
.ComputeUniqueInheritedLabels(prim)` returns an ancestor's labels for a child
that has none of its own (measured, not assumed). So a single label on
`/Root/Warehouse` would, on paper, cover the whole building.

**The annotators do not do that by default.** `omni.syntheticdata`'s
`set_default_semantic_filter(predicate="*:*", hierarchical_labels=False,
matching_labels=True)` -- "option to propagate semantic labels within the
hierarchy, from parent to children" -- defaults to False, and
`omni.replicator.core`'s annotator setup calls `set_semantic_filter` without
overriding it. So both numbers are reported, `direct` first, and they are not
interchangeable: the gap between them is exactly the geometry that would be
labelled if someone turned hierarchical labels on, and is unlabelled today.

That is a reading of the shipped source, not a measurement of a live
annotator. **This script does not render anything and cannot close it.** The
measurement that would: attach `semantic_segmentation` to a camera looking at a
prim whose only label is on an ancestor, and see whether the id map is empty.

WHAT COUNTS AS "RENDERABLE GEOMETRY"
------------------------------------
What an RTX sensor can trace, which is not the same as what is on the stage:
a `UsdGeom.Gprim` (or a `PointInstancer`), whose computed visibility is not
`invisible`, and whose computed purpose is `default` or `render` -- never
`guide` or `proxy`. That definition is load-bearing twice over in this project:
failure mode 1 is an avatar the sensors cannot see, and S6's own gate turns on
NVIDIA's character-controller demo shipping a `purpose="guide"` capsule that
looks fine in the stage tree and is invisible to every sensor.

Instance proxies are traversed (`Usd.TraverseInstanceProxies`). A warehouse
built from instanceable references hides most of its geometry inside
prototypes, and a plain `stage.Traverse()` would report a nearly empty building
and a coverage fraction that is wrong in the flattering direction.

Bounding-box volume is what was asked for and what is ranked, in world metres
cubed. It under-ranks flat things -- a wall or a floor slab has almost no
volume and is most of what a camera sees -- so a secondary ranking by bbox
surface area is printed next to it. Neither is occlusion-aware; bboxes of
sibling meshes may overlap, so the volume total is an upper bound, not a
partition.

EXECUTION MODEL
---------------
Exec mode, per CLAUDE.md and `sim/spikes/_diag_exec.py`: no `SimulationApp`,
frames come from the update event stream, `post_quit()` when done. This reads
USD properties only and would therefore also run under `SimulationApp` (as
`sim/verify_avatar.py` does) -- exec mode is used because it is the project
default and costs nothing here. Kit is needed either way for its asset
resolver, or the warehouse's referenced props never compose.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_semantics_audit.py

`docker stop` the container afterwards: an exec-mode container does not exit
when the script prints DONE, and the next launch fails on the port it holds.

Environment (argv is ambiguous after ``--exec``, so config is env vars):

    SA_STAGE          stage to open   (default /workspace/sim/observatory_avatar.usd)
    SA_OUT            report directory (default /workspace/sim/spikes/logs)
    SA_TOP            how many largest unlabelled entries to name (default 25)
    SA_MAX_PRINT      cap on labelled-prim lines printed; all go to JSON (default 400)
    SA_ROBOT_ASSETS   1 to open robot assets not present on the stage (default 1)
    SA_MAX_WAIT       frames to wait for the stage to finish loading (default 3000)
"""

from __future__ import annotations

import json
import os
import time
from collections import defaultdict
from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, UsdSemantics

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

STAGE = os.environ.get("SA_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("SA_OUT", str(REPO / "sim" / "spikes" / "logs")))
# The logs VOLUME, which CLAUDE.md confirms is container-writable. Only used if
# the requested directory is not -- /workspace is the bind mount, owned by uid
# 1004 while the container runs as 1234.
FALLBACK_DIR = Path("/isaac-sim/.nvidia-omniverse/logs")
TOP_N = int(os.environ.get("SA_TOP", "25"))
MAX_PRINT = int(os.environ.get("SA_MAX_PRINT", "400"))
ROBOT_ASSETS = os.environ.get("SA_ROBOT_ASSETS", "1") != "0"
MAX_WAIT = int(os.environ.get("SA_MAX_WAIT", "3000"))

STEM = "s10_semantics_audit"

LABELS_API = "UsdSemantics.LabelsAPI"
LEGACY_API = "Semantics.SemanticsAPI"

_LEGACY_PREFIX = "semantic:"
_LEGACY_TYPE = "semanticType"
_LEGACY_DATA = "semanticData"
_LABELS_PREFIX = "semantics:labels:"

_LOG_LINES: list[str] = []


def log(msg: str = "") -> None:
    """Print, and keep a copy: Kit's stdout is a firehose and the summary is
    the point of the run. The kept copy is what lands next to the JSON."""
    _LOG_LINES.append(msg)
    print(f"[semantics] {msg}" if msg else "", flush=True)


# ---------------------------------------------------------------------------
# Output. Incremental and fsync'd, never write-at-exit: this renderer dies
# mid-run and a write-at-exit design loses everything (CLAUDE.md, exec mode).
# tmp-then-replace on top of that, so a crash mid-write leaves the previous
# complete report rather than half a JSON document.
# ---------------------------------------------------------------------------
def resolve_out() -> tuple[Path, str]:
    candidates = ((OUT_DIR, "requested (SA_OUT)"),
                  (FALLBACK_DIR, "FALLBACK -- the logs volume"))
    for cand, why in candidates:
        try:
            cand.mkdir(parents=True, exist_ok=True)
            probe = cand / f".{STEM}.probe"
            probe.write_text("x", encoding="utf-8")
            probe.unlink()
            return cand, why
        except Exception as exc:
            print(f"[semantics] {cand} is not writable ({exc!r})", flush=True)
    raise RuntimeError("no writable output directory")


def durable_write(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".part")
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class Report:
    """The JSON inventory, flushed to disk after every section completes."""

    def __init__(self, out: Path) -> None:
        self.json_path = out / f"{STEM}.json"
        self.log_path = out / f"{STEM}.log"
        self.data: dict = {"schema_version": 1, "complete": False}

    def section(self, name: str, payload) -> None:
        self.data[name] = payload
        self.flush()

    def flush(self) -> None:
        durable_write(self.json_path, json.dumps(self.data, indent=1, default=str))
        durable_write(self.log_path, "\n".join(_LOG_LINES) + "\n")


# ---------------------------------------------------------------------------
# Label reading. Both schemas, and the orphan-attribute case.
# ---------------------------------------------------------------------------
def labels_api_entries(prim: Usd.Prim) -> list[dict]:
    """Labels from the current `UsdSemantics.LabelsAPI`, one entry per taxonomy.

    Instance names are collected from the applied schemas AND from the authored
    attribute names, so that an attribute present without `SemanticsLabelsAPI`
    applied is reported rather than silently skipped -- it is invisible to
    `LabelsAPI.Get()` and it is the case that looks correct in a layer dump.
    """
    applied = {s.split(":", 1)[1] for s in prim.GetAppliedSchemas()
               if s.startswith("SemanticsLabelsAPI:")}
    authored = {p.GetName()[len(_LABELS_PREFIX):] for p in prim.GetProperties()
                if p.GetName().startswith(_LABELS_PREFIX)}
    out: list[dict] = []
    for tax in sorted(applied | authored):
        attr = prim.GetAttribute(f"{_LABELS_PREFIX}{tax}")
        value = list(attr.Get() or []) if attr else []
        out.append({
            "schema": LABELS_API,
            "taxonomy": tax,
            "labels": [str(v) for v in value],
            "api_applied": tax in applied,
            "attribute": f"{_LABELS_PREFIX}{tax}",
        })
    return out


def legacy_entries(prim: Usd.Prim) -> list[dict]:
    """Labels from the deprecated `Semantics.SemanticsAPI`.

    Read from property names rather than through the `Semantics` module: the
    applied-schema string and the attribute names are authored data and are
    there whether or not that schema plugin registered in this process, which
    is one fewer way for the audit to under-report.
    """
    applied = {s.split(":", 1)[1] for s in prim.GetAppliedSchemas()
               if s.startswith("SemanticsAPI:")}
    authored: set[str] = set()
    for prop in prim.GetProperties():
        parts = prop.SplitName()
        if len(parts) == 4 and parts[0] == "semantic" and parts[2] == "params" \
                and parts[3] in (_LEGACY_TYPE, _LEGACY_DATA):
            authored.add(parts[1])
    out: list[dict] = []
    for inst in sorted(applied | authored):
        t_attr = prim.GetAttribute(f"{_LEGACY_PREFIX}{inst}:params:{_LEGACY_TYPE}")
        d_attr = prim.GetAttribute(f"{_LEGACY_PREFIX}{inst}:params:{_LEGACY_DATA}")
        sem_type = str(t_attr.Get()) if t_attr and t_attr.Get() is not None else ""
        sem_data = str(d_attr.Get()) if d_attr and d_attr.Get() is not None else ""
        # semanticData was conventionally a comma-separated list in the 4.x
        # tooling; the upgrade helper moves the whole string across as one
        # label, so both readings are recorded rather than one being assumed.
        labels = [s.strip() for s in sem_data.split(",") if s.strip()]
        out.append({
            "schema": LEGACY_API,
            "taxonomy": sem_type or "<semanticType unset>",
            "labels": labels,
            "raw_semantic_data": sem_data,
            "instance": inst,
            "api_applied": inst in applied,
            "attribute": f"{_LEGACY_PREFIX}{inst}:params:{_LEGACY_DATA}",
        })
    return out


def direct_entries(prim: Usd.Prim) -> list[dict]:
    return labels_api_entries(prim) + legacy_entries(prim)


# ---------------------------------------------------------------------------
# Renderability. What an RTX sensor can trace, not what is on the stage.
# ---------------------------------------------------------------------------
PROXY_PRED = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)

DEFAULT_PURPOSE = str(UsdGeom.Tokens.default_)
RENDERABLE_PURPOSES = {DEFAULT_PURPOSE, str(UsdGeom.Tokens.render)}


def classify(prim: Usd.Prim) -> dict:
    is_gprim = prim.IsA(UsdGeom.Gprim)
    is_pi = prim.IsA(UsdGeom.PointInstancer)
    if not (is_gprim or is_pi):
        return {"geometry": False}
    img = UsdGeom.Imageable(prim)
    visibility = str(img.ComputeVisibility())
    purpose = str(img.ComputePurpose())
    return {
        "geometry": True,
        "point_instancer": is_pi,
        "visibility": visibility,
        "purpose": purpose,
        "renderable": visibility != str(UsdGeom.Tokens.invisible)
        and purpose in RENDERABLE_PURPOSES,
    }


def subtree_key(path: str, root: str) -> str:
    """The top-level branch a prim belongs to, e.g. /Root/Warehouse."""
    if root and root != "/" and path.startswith(root + "/"):
        return f"{root}/{path[len(root) + 1:].split('/', 1)[0]}"
    if path == root:
        return root
    head = path.lstrip("/").split("/", 1)[0]
    return f"/{head}" if head else "/"


def asset_root_of(prim: Usd.Prim, default_root: str) -> str:
    """The nearest ancestor that looks like a discrete *thing* rather than a
    mesh piece: an instance root, or a prim carrying a reference or payload arc.

    Ranking raw leaf gprims names 25 shelf brackets. Rolling them up to the arc
    that brought them in names the shelf, which is the object a human would have
    to go and label and the object an agent would have to name.
    """
    p = prim
    while p and p.IsValid() and not p.IsPseudoRoot():
        if p.IsInstance() or p.HasAuthoredReferences() or p.HasAuthoredPayloads():
            return p.GetPath().pathString
        p = p.GetParent()
    return subtree_key(prim.GetPath().pathString, default_root)


# ---------------------------------------------------------------------------
# The audit itself
# ---------------------------------------------------------------------------
def bbox_metrics(rng, mpu: float) -> dict | None:
    if rng is None or rng.IsEmpty():
        return None
    size = rng.GetSize()
    x, y, z = (abs(float(size[i])) * mpu for i in range(3))
    mn, mx = rng.GetMin(), rng.GetMax()
    return {
        "size_m": [round(x, 4), round(y, 4), round(z, 4)],
        "volume_m3": x * y * z,
        "area_m2": 2.0 * (x * y + y * z + z * x),
        "diag_m": (x * x + y * y + z * z) ** 0.5,
        "center_m": [round((float(mn[i]) + float(mx[i])) * 0.5 * mpu, 3) for i in range(3)],
    }


def audit_stage(stage: Usd.Stage, report: Report) -> dict:
    default_prim = stage.GetDefaultPrim()
    root = default_prim.GetPath().pathString if default_prim else "/"
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage))

    facts = {
        "stage": STAGE,
        "default_prim": root,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": mpu,
        "layers": [str(la.identifier) for la in stage.GetLayerStack()],
        "root_children": [c.GetName() for c in default_prim.GetChildren()] if default_prim else [],
    }
    log("=" * 78)
    log("SEMANTIC LABELLING AUDIT -- report only, nothing on this stage was changed")
    log("=" * 78)
    log(f"stage          {STAGE}")
    log(f"default prim   {root}   up {facts['up_axis']}   {mpu} m/unit")
    log(f"root children  {', '.join(facts['root_children']) or '<none>'}")
    report.section("stage_facts", facts)

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )

    labelled_rows: list[dict] = []          # one row per (prim, taxonomy, schema)
    direct_by_path: dict[str, list[dict]] = {}
    geo: list[dict] = []                    # every geometry prim, renderable or not
    taxonomies: set[str] = set()
    counts = {
        "prims_total": 0, "instance_proxies": 0, "geometry_prims": 0,
        "renderable": 0, "invisible": 0, "non_render_purpose": 0,
        "point_instancers": 0, "unloaded_payloads": 0,
    }

    t0 = time.perf_counter()
    for prim in Usd.PrimRange.Stage(stage, PROXY_PRED):
        counts["prims_total"] += 1
        if counts["prims_total"] % 20000 == 0:
            log(f"  ... {counts['prims_total']} prims traversed")
        path = prim.GetPath().pathString
        if prim.IsInstanceProxy():
            counts["instance_proxies"] += 1
        if prim.HasAuthoredPayloads() and not prim.IsLoaded():
            counts["unloaded_payloads"] += 1

        entries = direct_entries(prim)
        if entries:
            direct_by_path[path] = entries
            for e in entries:
                taxonomies.add(e["taxonomy"])

        cls = classify(prim)
        if not cls["geometry"]:
            continue
        counts["geometry_prims"] += 1
        if cls["point_instancer"]:
            counts["point_instancers"] += 1
        if cls["visibility"] == str(UsdGeom.Tokens.invisible):
            counts["invisible"] += 1
        if cls["purpose"] not in RENDERABLE_PURPOSES:
            counts["non_render_purpose"] += 1
        rec = {
            "path": path,
            "type": str(prim.GetTypeName()),
            "renderable": cls["renderable"],
            "visibility": cls["visibility"],
            "purpose": cls["purpose"],
            "instance_proxy": prim.IsInstanceProxy(),
            "point_instancer": cls["point_instancer"],
            "subtree": subtree_key(path, root),
        }
        if cls["renderable"]:
            counts["renderable"] += 1
            rec["bbox"] = bbox_metrics(
                bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange(), mpu)
        geo.append(rec)

    log(f"traversed {counts['prims_total']} prims "
        f"({counts['instance_proxies']} instance proxies) in "
        f"{time.perf_counter() - t0:.1f}s")

    # --- inherited labels ---------------------------------------------------
    # LabelsQuery understands the current schema only, so the legacy schema gets
    # an explicit ancestor walk. Both answer the same question: does anything at
    # or above this prim carry a label?
    queries = {}
    for tax in sorted(taxonomies):
        try:
            queries[tax] = UsdSemantics.LabelsQuery(tax, Usd.TimeCode.Default())
        except Exception as exc:      # a taxonomy name that is not a valid token
            log(f"! no LabelsQuery for taxonomy {tax!r}: {exc!r}")

    def effective_labels(prim: Usd.Prim) -> dict[str, list[str]]:
        out: dict[str, list[str]] = {}
        for tax, q in queries.items():
            try:
                vals = [str(v) for v in q.ComputeUniqueInheritedLabels(prim)]
            except Exception:
                vals = []
            if vals:
                out[tax] = sorted(vals)
        # legacy: nearest labelled ancestor, self included
        p = prim
        while p and p.IsValid() and not p.IsPseudoRoot():
            for e in direct_by_path.get(p.GetPath().pathString, []):
                if e["schema"] == LEGACY_API and e["labels"]:
                    out.setdefault(e["taxonomy"], [])
                    for lab in e["labels"]:
                        if lab not in out[e["taxonomy"]]:
                            out[e["taxonomy"]].append(lab)
            p = p.GetParent()
        return out

    # --- section 1: every labelled prim ------------------------------------
    for path in sorted(direct_by_path):
        prim = stage.GetPrimAtPath(path)
        cls = classify(prim) if prim and prim.IsValid() else {"geometry": False}
        for e in direct_by_path[path]:
            labelled_rows.append({
                "path": path,
                "prim_type": str(prim.GetTypeName()) if prim and prim.IsValid() else "?",
                "schema": e["schema"],
                "taxonomy": e["taxonomy"],
                "labels": e["labels"],
                "api_applied": e["api_applied"],
                "attribute": e["attribute"],
                "raw_semantic_data": e.get("raw_semantic_data"),
                "geometry": cls["geometry"],
                "renderable": cls.get("renderable", False),
                "visibility": cls.get("visibility"),
                "purpose": cls.get("purpose"),
                "subtree": subtree_key(path, root),
            })

    log("")
    log("-" * 78)
    log(f"1. LABELLED PRIMS -- {len(direct_by_path)} prims, {len(labelled_rows)} label entries")
    log("-" * 78)
    if not labelled_rows:
        log("  NONE. Every segmentation and bbox annotator on this stage returns empty.")
    for i, r in enumerate(labelled_rows):
        if i >= MAX_PRINT:
            log(f"  ... {len(labelled_rows) - MAX_PRINT} more entries; all of them are in the JSON")
            break
        flag = "" if r["api_applied"] else "  <-- ATTRIBUTE ONLY, API SCHEMA NOT APPLIED"
        kind = ("renderable" if r["renderable"]
                else "geometry" if r["geometry"] else "non-geometry")
        log(f"  {r['schema']:<26} {r['taxonomy']}={','.join(r['labels']) or '<empty>':<18} "
            f"[{kind}] {r['path']}{flag}")
    report.section("labelled_prims", labelled_rows)

    # --- section 2: classes -------------------------------------------------
    classes: dict[str, dict] = {}
    for r in labelled_rows:
        tax = classes.setdefault(r["taxonomy"], {})
        for lab in (r["labels"] or ["<empty>"]):
            entry = tax.setdefault(lab, {"prims": 0, "renderable_prims": 0, "schemas": {}})
            entry["prims"] += 1
            entry["renderable_prims"] += int(bool(r["renderable"]))
            entry["schemas"][r["schema"]] = entry["schemas"].get(r["schema"], 0) + 1

    by_schema = defaultdict(int)
    orphans = 0
    for r in labelled_rows:
        by_schema[r["schema"]] += 1
        orphans += int(not r["api_applied"])

    log("")
    log("-" * 78)
    log(f"2. CLASSES -- {sum(len(v) for v in classes.values())} distinct labels "
        f"across {len(classes)} taxonomies")
    log("-" * 78)
    for tax in sorted(classes):
        log(f"  taxonomy {tax!r}")
        for lab in sorted(classes[tax], key=lambda k: -classes[tax][k]["prims"]):
            e = classes[tax][lab]
            schemas = ", ".join(f"{k.split('.')[-1]}x{v}" for k, v in sorted(e["schemas"].items()))
            log(f"     {lab:<24} {e['prims']:>5} prims  "
                f"({e['renderable_prims']} renderable)   via {schemas}")
    log("  carried by:")
    for schema in (LABELS_API, LEGACY_API):
        log(f"     {schema:<26} {by_schema.get(schema, 0):>5} label entries")
    log(f"     attributes with NO API schema applied: {orphans}"
        + ("   <-- invisible to LabelsAPI.Get()" if orphans else ""))
    report.section("classes", {
        "by_taxonomy": classes,
        "by_schema": dict(by_schema),
        "entries_without_applied_api_schema": orphans,
    })

    # --- section 3: coverage of renderable geometry -------------------------
    renderable = [g for g in geo if g["renderable"]]
    total_vol = sum((g["bbox"] or {}).get("volume_m3", 0.0) for g in renderable)

    per_subtree: dict[str, dict] = {}
    direct_n = eff_n = 0
    direct_vol = eff_vol = 0.0
    unlabelled: list[dict] = []

    for g in renderable:
        prim = stage.GetPrimAtPath(g["path"])
        has_direct = bool(direct_by_path.get(g["path"]))
        eff = effective_labels(prim) if prim and prim.IsValid() else {}
        has_eff = has_direct or bool(eff)
        vol = (g["bbox"] or {}).get("volume_m3", 0.0)
        g["labelled_direct"] = has_direct
        g["labelled_effective"] = has_eff
        g["effective_labels"] = eff
        direct_n += has_direct
        eff_n += has_eff
        direct_vol += vol if has_direct else 0.0
        eff_vol += vol if has_eff else 0.0
        s = per_subtree.setdefault(g["subtree"], {
            "renderable_prims": 0, "labelled_direct": 0, "labelled_effective": 0,
            "volume_m3": 0.0, "labelled_direct_volume_m3": 0.0})
        s["renderable_prims"] += 1
        s["labelled_direct"] += has_direct
        s["labelled_effective"] += has_eff
        s["volume_m3"] += vol
        s["labelled_direct_volume_m3"] += vol if has_direct else 0.0
        if not has_eff:
            unlabelled.append(g)

    def pct(a, b):
        return (100.0 * a / b) if b else 0.0

    coverage = {
        "definition": ("renderable = UsdGeom.Gprim or PointInstancer, computed "
                       "visibility != invisible, computed purpose in (default, render); "
                       "instance proxies included"),
        "counts": counts,
        "renderable_prims": len(renderable),
        "renderable_bbox_volume_m3": total_vol,
        "direct": {"prims": direct_n, "prims_pct": pct(direct_n, len(renderable)),
                   "volume_m3": direct_vol, "volume_pct": pct(direct_vol, total_vol)},
        "effective": {"prims": eff_n, "prims_pct": pct(eff_n, len(renderable)),
                      "volume_m3": eff_vol, "volume_pct": pct(eff_vol, total_vol)},
        "unlabelled": {"prims": len(renderable) - eff_n,
                       "prims_pct": pct(len(renderable) - eff_n, len(renderable)),
                       "volume_m3": total_vol - eff_vol,
                       "volume_pct": pct(total_vol - eff_vol, total_vol)},
        "by_subtree": per_subtree,
    }

    log("")
    log("-" * 78)
    log("3. COVERAGE OF RENDERABLE GEOMETRY")
    log("-" * 78)
    log(f"  {counts['prims_total']} prims on the stage; {counts['geometry_prims']} carry geometry; "
        f"{len(renderable)} of those are renderable")
    log(f"  excluded: {counts['invisible']} invisible, "
        f"{counts['non_render_purpose']} guide/proxy purpose, "
        f"{counts['unloaded_payloads']} prims with unloaded payloads")
    log(f"  total bbox volume {total_vol:,.1f} m3 (upper bound -- sibling boxes may overlap)")
    log("")
    log(f"  {'':<26}{'prims':>16}{'bbox volume':>22}")
    log(f"  {'labelled, direct':<26}{direct_n:>7} {pct(direct_n, len(renderable)):>6.1f}%"
        f"{direct_vol:>14,.1f} m3 {pct(direct_vol, total_vol):>5.1f}%")
    log(f"  {'labelled, incl. inherited':<26}{eff_n:>7} {pct(eff_n, len(renderable)):>6.1f}%"
        f"{eff_vol:>14,.1f} m3 {pct(eff_vol, total_vol):>5.1f}%")
    log(f"  {'UNLABELLED':<26}{len(renderable) - eff_n:>7} "
        f"{pct(len(renderable) - eff_n, len(renderable)):>6.1f}%"
        f"{total_vol - eff_vol:>14,.1f} m3 {pct(total_vol - eff_vol, total_vol):>5.1f}%")
    log("")
    log("  Annotators do NOT propagate labels down the hierarchy by default")
    log("  (omni.syntheticdata set_default_semantic_filter hierarchical_labels=False),")
    log("  so 'direct' is the row that predicts what segmentation returns. Read from")
    log("  the shipped source, not measured here -- this script renders nothing.")
    log("")
    # The subtree's own world extent, printed so the volume column is
    # CHECKABLE rather than merely reported: a warehouse that comes out 4 m
    # wide means the bbox pipeline is wrong, and nothing else here would say so.
    for key, s in per_subtree.items():
        prim = stage.GetPrimAtPath(key)
        s["extent"] = bbox_metrics(
            bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange(), mpu) \
            if prim and prim.IsValid() else None

    log(f"  {'subtree':<26}{'renderable':>11}{'direct':>9}{'+inherit':>10}"
        f"{'volume m3':>12}   extent m")
    for key in sorted(per_subtree, key=lambda k: -per_subtree[k]["volume_m3"]):
        s = per_subtree[key]
        ext = "x".join(f"{v:.1f}" for v in (s["extent"] or {}).get("size_m", [])) or "-"
        log(f"  {key:<26}{s['renderable_prims']:>11}{s['labelled_direct']:>9}"
            f"{s['labelled_effective']:>10}{s['volume_m3']:>12,.1f}   {ext}")

    # Geometry the sensors cannot see. Only a handful, and worth NAMING rather
    # than counting: failure mode 1 in CLAUDE.md is an avatar the sensors trace
    # straight through, and its signature is exactly a mesh that is on the
    # stage, looks right in the tree, and is invisible or purpose=guide.
    excluded_geo = [g for g in geo if not g["renderable"]]
    coverage["excluded_geometry"] = excluded_geo
    log("")
    log(f"  geometry the sensors CANNOT trace ({len(excluded_geo)} prims) -- "
        f"invisible or guide/proxy purpose:")
    for g in excluded_geo[:40]:
        log(f"     {g['visibility']:<10} purpose={g['purpose']:<8} {g['type']:<10} {g['path']}")
    if len(excluded_geo) > 40:
        log(f"     ... {len(excluded_geo) - 40} more (in the JSON)")
    if not excluded_geo:
        log("     none")
    report.section("coverage", coverage)

    # --- section 4: largest unlabelled --------------------------------------
    by_vol = sorted(unlabelled, key=lambda g: -(g["bbox"] or {}).get("volume_m3", 0.0))[:TOP_N]
    by_area = sorted(unlabelled, key=lambda g: -(g["bbox"] or {}).get("area_m2", 0.0))[:TOP_N]

    groups: dict[str, dict] = {}
    for g in unlabelled:
        prim = stage.GetPrimAtPath(g["path"])
        key = asset_root_of(prim, root) if prim and prim.IsValid() else g["subtree"]
        entry = groups.setdefault(key, {"path": key, "unlabelled_gprims": 0,
                                        "sum_leaf_volume_m3": 0.0})
        entry["unlabelled_gprims"] += 1
        entry["sum_leaf_volume_m3"] += (g["bbox"] or {}).get("volume_m3", 0.0)
    for key, entry in groups.items():
        prim = stage.GetPrimAtPath(key)
        if prim and prim.IsValid():
            entry["prim_type"] = str(prim.GetTypeName())
            entry["bbox"] = bbox_metrics(
                bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange(), mpu)
    top_groups = sorted(groups.values(),
                        key=lambda e: -((e.get("bbox") or {}).get("volume_m3", 0.0)))[:TOP_N]

    log("")
    log("-" * 78)
    log(f"4. LARGEST UNLABELLED RENDERABLE GEOMETRY -- {len(unlabelled)} prims, "
        f"{len(groups)} distinct assets")
    log("-" * 78)
    log("  by whole asset (nearest reference/payload/instance ancestor):")
    log(f"  {'bbox m3':>12}{'gprims':>8}  {'size m':<22} path")
    for e in top_groups:
        b = e.get("bbox") or {}
        size = "x".join(f"{v:.1f}" for v in b.get("size_m", [])) or "-"
        log(f"  {b.get('volume_m3', 0.0):>12,.1f}{e['unlabelled_gprims']:>8}  "
            f"{size:<22} {e['path']}")
    log("")
    log("  by single prim:")
    log(f"  {'bbox m3':>12}{'area m2':>10}  {'size m':<22} path")
    for g in by_vol:
        b = g["bbox"] or {}
        size = "x".join(f"{v:.1f}" for v in b.get("size_m", [])) or "-"
        log(f"  {b.get('volume_m3', 0.0):>12,.1f}{b.get('area_m2', 0.0):>10,.1f}  "
            f"{size:<22} {g['path']}")
    log("")
    log("  by bbox surface area -- a wall or a floor slab has almost no volume and is")
    log("  most of what a camera sees, so volume alone mis-ranks flat geometry:")
    log(f"  {'area m2':>12}{'bbox m3':>10}  {'size m':<22} path")
    for g in by_area:
        b = g["bbox"] or {}
        size = "x".join(f"{v:.1f}" for v in b.get("size_m", [])) or "-"
        log(f"  {b.get('area_m2', 0.0):>12,.1f}{b.get('volume_m3', 0.0):>10,.1f}  "
            f"{size:<22} {g['path']}")
    # Top-N is what gets PRINTED. The JSON carries the whole list -- an
    # inventory that names 25 of 41 unlabelled prims leaves 16 anonymous, and
    # the point of writing it to disk is that the next session does not have to
    # re-run the stage to see them. Capped only so a badly labelled stage
    # cannot produce a report nobody can open.
    ALL_CAP = 5000
    report.section("largest_unlabelled", {
        "by_asset_group": top_groups,
        "by_prim_volume": by_vol,
        "by_prim_area": by_area,
        "unlabelled_renderable_prims": len(unlabelled),
        "distinct_unlabelled_assets": len(groups),
        "all_unlabelled": sorted(
            unlabelled, key=lambda g: -(g["bbox"] or {}).get("volume_m3", 0.0))[:ALL_CAP],
        "all_unlabelled_omitted": max(0, len(unlabelled) - ALL_CAP),
        "all_asset_groups": sorted(
            groups.values(),
            key=lambda e: -((e.get("bbox") or {}).get("volume_m3", 0.0)))[:ALL_CAP],
    })

    return {"root": root, "mpu": mpu, "direct_by_path": direct_by_path,
            "effective_labels": effective_labels, "geo": geo, "bbox_cache": bbox_cache}


# ---------------------------------------------------------------------------
# The two named questions: the avatar, and the three robots
# ---------------------------------------------------------------------------
def audit_avatar(stage: Usd.Stage, ctx: dict, cfg: dict, report: Report) -> dict:
    avatar_cfg = cfg.get("avatar") or {}
    path = avatar_cfg.get("prim_path", "/Root/Avatar")
    want = str(avatar_cfg.get("semantic_class", "person"))
    direct_by_path = ctx["direct_by_path"]
    effective_labels = ctx["effective_labels"]

    out: dict = {"prim_path": path, "declared_class": want, "config": "config/scene.yaml"}
    log("")
    log("-" * 78)
    log(f"5. AVATAR -- does {path} carry its {want!r} label?")
    log("-" * 78)

    prim = stage.GetPrimAtPath(path)
    if not (prim and prim.IsValid()):
        out.update({"present": False, "carries_declared_class": False,
                    "verdict": f"{path} is not on this stage"})
        log(f"  ! {path} is not on this stage -- nothing to check")
        report.section("avatar", out)
        return out

    out["present"] = True
    labelled_here, gprims = [], []
    for p in Usd.PrimRange(prim, PROXY_PRED):
        pp = p.GetPath().pathString
        if pp in direct_by_path:
            labelled_here.append({"path": pp, "entries": direct_by_path[pp]})
        cls = classify(p)
        if cls["geometry"]:
            gprims.append({
                "path": pp, "type": str(p.GetTypeName()),
                "renderable": cls["renderable"], "visibility": cls["visibility"],
                "purpose": cls["purpose"],
                "direct": [lab for e in direct_by_path.get(pp, []) for lab in e["labels"]],
                "effective": effective_labels(p),
            })

    direct_all = {lab for e in labelled_here for ent in e["entries"] for lab in ent["labels"]}
    out["labelled_prims"] = labelled_here
    out["labels_in_subtree"] = sorted(direct_all)
    out["carries_declared_class"] = want in direct_all

    renderable_gprims = [g for g in gprims if g["renderable"]]
    with_direct = [g for g in renderable_gprims if want in g["direct"]]
    with_inherited = [g for g in renderable_gprims
                      if want not in g["direct"]
                      and any(want in v for v in g["effective"].values())]
    with_none = [g for g in renderable_gprims
                 if want not in g["direct"]
                 and not any(want in v for v in g["effective"].values())]
    out["renderable_gprims"] = len(renderable_gprims)
    out["gprims_with_direct_class"] = [g["path"] for g in with_direct]
    out["gprims_with_inherited_class_only"] = [g["path"] for g in with_inherited]
    out["gprims_with_no_class"] = [g["path"] for g in with_none]

    for e in labelled_here:
        for ent in e["entries"]:
            log(f"  {ent['schema']:<26} {ent['taxonomy']}="
                f"{','.join(ent['labels']) or '<empty>':<14} {e['path']}")
    if not labelled_here:
        log(f"  no semantic label anywhere under {path}")
    log(f"  renderable gprims under the avatar: {len(renderable_gprims)}")
    log(f"     carrying {want!r} directly     : {len(with_direct)}")
    log(f"     inheriting it from an ancestor : {len(with_inherited)}")
    log(f"     carrying nothing               : {len(with_none)}")
    for g in with_none[:12]:
        log(f"        {g['path']}")
    if len(with_none) > 12:
        log(f"        ... {len(with_none) - 12} more (in the JSON)")

    if not out["carries_declared_class"]:
        out["verdict"] = f"NO -- {want!r} appears nowhere under {path}"
    elif with_direct:
        out["verdict"] = (f"YES -- {want!r} is direct on {len(with_direct)} of "
                          f"{len(renderable_gprims)} renderable gprims")
    else:
        out["verdict"] = (f"PARTIAL -- {want!r} is present but on no renderable gprim "
                          f"directly; {len(with_inherited)} would need hierarchical labels")
    log(f"  VERDICT: {out['verdict']}")
    report.section("avatar", out)
    return out


def audit_robots(stage: Usd.Stage, ctx: dict, cfg: dict, report: Report) -> list[dict]:
    """Do the robots carry any label at all?

    Robots are not saved into the stage -- `sensor_factory.reference_robots()`
    references them at runtime, which is a write, and this script does not
    write. So a robot absent from the stage is audited by opening its ASSET as a
    separate read-only stage, which is where any shipped label would live
    anyway. Which route was taken is recorded per robot; do not read the two as
    the same measurement.
    """
    robots = cfg.get("robots") or []
    direct_by_path = ctx["direct_by_path"]
    out: list[dict] = []

    log("")
    log("-" * 78)
    log(f"6. ROBOTS -- do the {len(robots)} declared robots carry any label at all?")
    log("-" * 78)

    assets_root = None
    if ROBOT_ASSETS:
        try:
            from isaacsim.storage.native import get_assets_root_path

            assets_root = get_assets_root_path()
        except Exception as exc:
            log(f"  ! asset root unavailable ({exc!r}) -- assets cannot be opened")

    for r in robots:
        rid = r.get("id", "?")
        path = r.get("prim_path", "")
        rec: dict = {"id": rid, "prim_path": path, "asset": r.get("asset"),
                     "labelled_prims": [], "labels": [], "source": None}
        prim = stage.GetPrimAtPath(path) if path else None
        if prim and prim.IsValid():
            rec["source"] = "stage"
            rec["on_stage"] = True
            n_geo = 0
            for p in Usd.PrimRange(prim, PROXY_PRED):
                pp = p.GetPath().pathString
                n_geo += int(classify(p)["geometry"])
                if pp in direct_by_path:
                    rec["labelled_prims"].append({"path": pp, "entries": direct_by_path[pp]})
            rec["geometry_prims"] = n_geo
        else:
            rec["on_stage"] = False
            have_asset = bool(assets_root and r.get("asset"))
            url = f"{assets_root}/Isaac/{r.get('asset')}" if have_asset else None
            rec["asset_url"] = url
            if not url:
                rec["source"] = "unavailable"
                rec["verdict"] = "not on the stage and the asset could not be resolved"
                out.append(rec)
                log(f"  {rid:<8} not on the stage; asset not resolvable -- UNKNOWN")
                continue
            log(f"  {rid:<8} not on the stage (referenced at runtime); opening {url}")
            try:
                sub = Usd.Stage.Open(url)
            except Exception as exc:
                sub = None
                rec["open_error"] = repr(exc)
            if sub is None:
                rec["source"] = "unavailable"
                rec["verdict"] = f"asset would not open: {rec.get('open_error')}"
                log(f"  {rid:<8} asset would not open -- UNKNOWN")
                out.append(rec)
                continue
            rec["source"] = "asset"
            n_prims = n_geo = 0
            for p in Usd.PrimRange.Stage(sub, PROXY_PRED):
                n_prims += 1
                n_geo += int(classify(p)["geometry"])
                entries = direct_entries(p)
                if entries:
                    rec["labelled_prims"].append(
                        {"path": p.GetPath().pathString, "entries": entries})
            rec["asset_prims"] = n_prims
            rec["geometry_prims"] = n_geo

        rec["labels"] = sorted({lab for e in rec["labelled_prims"]
                                for ent in e["entries"] for lab in ent["labels"]})
        rec["labelled_prim_count"] = len(rec["labelled_prims"])
        rec["verdict"] = (f"NO LABEL of any kind ({rec.get('geometry_prims', 0)} geometry prims)"
                          if not rec["labelled_prims"]
                          else f"{len(rec['labelled_prims'])} labelled prims: "
                               f"{', '.join(rec['labels']) or '<empty labels>'}")
        log(f"  {rid:<8} {rec['source']:<8} geometry {rec.get('geometry_prims', 0):>5}   "
            f"{rec['verdict']}")
        for e in rec["labelled_prims"][:5]:
            for ent in e["entries"]:
                log(f"           {ent['schema']} {ent['taxonomy']}="
                    f"{','.join(ent['labels'])} {e['path']}")
        out.append(rec)

    report.section("robots", out)
    return out


# ---------------------------------------------------------------------------
# Drive
# ---------------------------------------------------------------------------
S: dict = {"frame": 0, "sub": None, "phase": "loading"}


def run() -> None:
    out_dir, why = resolve_out()
    report = Report(out_dir)
    log(f"report         {report.json_path}   ({why})")

    import yaml

    cfg_path = REPO / "config" / "scene.yaml"
    try:
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        log(f"! could not read {cfg_path}: {exc!r} -- avatar/robot sections will be thin")
        cfg = {}

    stage = omni.usd.get_context().get_stage()
    ctx = audit_stage(stage, report)
    avatar = audit_avatar(stage, ctx, cfg, report)
    robots = audit_robots(stage, ctx, cfg, report)

    cov = report.data["coverage"]
    cls = report.data["classes"]
    log("")
    log("=" * 78)
    log("VERDICT")
    log("=" * 78)
    log(f"  {cov['direct']['prims']} of {cov['renderable_prims']} renderable prims carry a "
        f"direct label ({cov['direct']['prims_pct']:.1f}%); "
        f"{cov['unlabelled']['prims']} carry none at all")
    inherit_only = cov["effective"]["prims"] - cov["direct"]["prims"]
    if inherit_only:
        where = [k for k, v in cov["by_subtree"].items()
                 if v["labelled_effective"] > v["labelled_direct"]]
        log(f"  {inherit_only} more are labelled ONLY through an ancestor, in "
            f"{', '.join(sorted(where))} -- those read as UNLABELLED to an annotator")
        log("    unless hierarchical labels are switched on. Labelled in USD, absent from")
        log("    the ground truth: the silent half of this failure mode.")
    log(f"  {sum(len(v) for v in cls['by_taxonomy'].values())} distinct classes exist across "
        f"{len(cls['by_taxonomy'])} taxonomies")
    log("  schemas: " + ", ".join(f"{k.split('.')[-1]} {v}" for k, v in cls["by_schema"].items())
        + (f", {cls['entries_without_applied_api_schema']} attribute-only"
           if cls["entries_without_applied_api_schema"] else ""))
    log(f"  avatar : {avatar.get('verdict')}")
    for r in robots:
        log(f"  {r['id']:<7}: {r['verdict']}  (from the {r['source']})")
    log("")
    log("  Nothing on this stage was created, changed, or saved.")
    log("=" * 78)

    report.data["complete"] = True
    report.data["generated_epoch"] = time.time()
    report.flush()
    size = report.json_path.stat().st_size
    log(f"wrote {report.json_path} ({size:,} bytes) and {report.log_path}")
    log("DONE")


def on_update(_e) -> None:
    S["frame"] += 1
    if S["phase"] != "loading":
        return
    ctx = omni.usd.get_context()
    still_loading = any(ctx.get_stage_loading_status()[1:])
    if S["frame"] <= 5 or (still_loading and S["frame"] < MAX_WAIT):
        if still_loading and S["frame"] % 300 == 0:
            print(f"[semantics] still loading at frame {S['frame']}: "
                  f"{ctx.get_stage_loading_status()}", flush=True)
        return
    if still_loading:
        print(f"[semantics] ! giving up waiting for the load at frame {S['frame']} -- "
              f"coverage below will UNDER-report", flush=True)
    S["phase"] = "running"
    try:
        run()
    except Exception as exc:
        import traceback

        print(f"[semantics] FAILED: {exc!r}", flush=True)
        print(traceback.format_exc(), flush=True)
    finally:
        S["sub"] = None
        omni.kit.app.get_app().post_quit()


print(f"[semantics] open_stage {STAGE} -> "
      f"{omni.usd.get_context().open_stage(STAGE)}", flush=True)
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="semantics_audit"
)
