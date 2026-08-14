# CLAUDE.md

Project invariants. Claude Code reads this file automatically at the start of
every session in this repo. Keep it short enough that it always gets read.

---

## What this project is

An Isaac Sim "sensor observatory": an instrumented warehouse/factory with static
multi-modal sensor stations, static robot observation platforms, and one
keyboard-driven avatar. **The avatar is the only moving entity.** The goal is
that sensor readings visibly change as it moves through the space.

Robots do not move. There is no task or algorithm yet. Making the robots static
deletes locomotion policies, teleop, and navmesh baking while *sharpening* the
demo: multiple heterogeneous sensors observing one dynamic element is the
research claim in miniature.

---

## Hard rules

1. **Never invent a prim path.** Read it from the stage, or from
   `config/sensors.yaml` once a human has confirmed it there, or stop and ask.
   A wrong prim path does not render badly — it crashes, or worse, silently
   does nothing.

2. **`core/` must never import `omni`, `pxr`, or `isaacsim`.** It consumes
   observation dicts only. This is what lets the same memory module run on
   Habitat for benchmark comparison, and lets the MacBook be a real dev
   machine. Enforced by `scripts/check_layer_boundary.sh` — run it before every
   commit that touches `core/`.

3. **`SimulationApp` must be constructed before any other omni/isaacsim
   import.** Not after. Not alongside. **This rule applies only to standalone
   scripts, which cannot read sensor data on this host — see "Exec mode" below.
   In exec mode there is no `SimulationApp` and therefore no import-ordering
   constraint at all.**

4. **Prefer built-in assets and menu examples over new code.** Isaac Examples,
   Robotics Examples, Synthetic Data Visualizer, Semantics Schema Editor. If an
   example exists, open and adapt it rather than writing from scratch.

5. **All sensors are declared in `config/sensors.yaml`.** Scripts read from the
   registry; they never hardcode a sensor. Adding a sensor means adding a YAML
   entry.

6. **This is a visual demo, not a data pipeline.** Skip calibration export,
   warm-up frame handling, and determinism checks.

7. **Never install anything on the host.** No `pip install`, no `npm install`,
   no `apt install` outside a container — on either machine.
   - Python packages for `core/`/`tests/` go in `docker/requirements-dev.txt`,
     then `make dev-build`.
   - Python packages for Isaac Sim go in `docker/requirements-sim.txt`, and are
     installed into the **bundled** interpreter via `./python.sh -m pip`, never
     the system python.
   - Run everything through the Makefile: `make test`, `make check`,
     `make verify`. These execute inside the dev container.
   - The **only** legitimate host-level installs are infrastructure that
     containers cannot provide for themselves: the NVIDIA driver, Docker, the
     NVIDIA Container Toolkit, and Tailscale. Nothing else — and none of them
     without asking first.

   If you find yourself wanting to install on the host to make a test pass, the
   dependency belongs in a requirements file and the image needs rebuilding.
   Say so instead of installing.

---

## Exec mode — the execution model for anything that reads sensor data

**Offscreen capture does not work under `SimulationApp` on this host.** Same
scene, same GPU, measured both ways:

```
SimulationApp     camera rgb      0 px   lidar GMO       0 points
runheadless.sh    camera rgb 32,700 px   lidar GMO 460,800 points
```

Every annotator stays empty under `SimulationApp` at every render config, GPU
count, and experience file tried, with no error logged. So:

> **Anything that reads sensor data runs in exec mode**, under the launcher
> that actually renders:
> ```
> ./runheadless.sh --exec /workspace/sim/<script>.py
> ```

`sim/spikes/_diag_exec.py` is the reference pattern. Three things follow from it:

- **No `SimulationApp`.** Kit is already up. Hard rule 3 does not apply —
  there is no import-ordering constraint in exec mode at all.
- **The equivalent constraint is different, and it is real:** the script runs
  *inside an already-built renderer*, so anything that must precede renderer
  init — **Motion BVH above all** — has to arrive as a **kit command-line
  argument** and cannot be set from Python at any point in the script. Setting
  it later throws no error and does nothing.
- **Drive frames from the update event stream**, not an `app.update()` loop —
  calling `update()` from inside an `--exec` script re-enters the main loop.
  Subscribe with `get_update_event_stream().create_subscription_to_pop(...)`
  and `post_quit()` when done.

Config comes from **environment variables**, not argv: Kit's
`--exec SCRIPT ARGS...` makes trailing-argument parsing ambiguous. Write results
**incrementally and `fsync`'d**, never once at the end — this renderer dies
mid-run, and a write-at-exit design loses everything.

Results must land in a container-writable path. The logs **volume**
(`/isaac-sim/.nvidia-omniverse/logs/`) works; `/workspace` is the bind mount,
owned by uid 1004 while the container runs as **1234**, so writes there fail
**silently**.

> **Caveat, deliberately kept:** exec mode is confirmed **necessary** for sensor
> readback. It is not yet proven **sufficient** — see the Motion BVH note under
> "Environment facts". Plan `sim/` scripts against exec mode anyway: a caveated
> invariant costs nothing, and S5/S7/S8/S10/S11 written against `SimulationApp`
> cost a week.

---

## The API-era trap — read before writing any Isaac code

There are **three** incompatible API eras in circulation. Most web examples and
most LLM training data are from the oldest one.

| Era | Namespace | Where you'll meet it |
|---|---|---|
| 4.x | `omni.isaac.*` | Most tutorials, most recalled snippets |
| 5.x | `isaacsim.sensors.rtx` | 5.x docs, recent forum posts |
| 6.x | `isaacsim.sensors.experimental.rtx` | Current docs — **what we target** |

**This is the single biggest time-waster in the project.** Check the 6.0
migration guide against anything you recall. Specifically in 6.x:

- Sensor creation is class-based, not command-based:
  `LidarSensor(Lidar.create(path, config=..., translations=[[...]]))`
  — note *plural* `translations`/`orientations`, shape `(N,3)`/`(N,4)`, N=1 only.
- Debug draw is `LidarSensor.attach_writer("draw-point-cloud")`, not the
  `RtxLidarDebugDrawPointCloudBuffer` Replicator writer.
- Data comes from `sensor.get_data("generic-model-output")`, then
  `parse_generic_model_output_data(...)`.
- **That buffer is NOT a point cloud, and the defaults are the trap.** Its
  per-element `x`/`y`/`z` are **azimuth degrees, elevation degrees, range
  metres** — because `elementsCoordsType` defaults to `SPHERICAL` — and they are
  **sensor-local**, because `frameOfReference` defaults to `SENSOR`. Both
  defaults are in the schema (`OmniSensorGenericRadarWpmDmatAPI`) and in
  NVIDIA's own `test_radar_sensor.py`, which decodes them as
  `cos(radians(gmo.x)), sin(radians(gmo.x)), sin(radians(gmo.y))` against
  `gmo.z` as range. Read as Cartesian metres they look entirely plausible and
  are silently wrong — this cost S4 a week. Three steps stand between the buffer
  and metric world points:
  1. check `elementsCoordsType`, convert spherical→Cartesian;
  2. check `frameOfReference`, apply the sensor pose;
  3. mask on `flags & VALID` (`ElementFlags.VALID == 64`).
  `IsaacExtractRTXSensorPointCloud` does 1 and 2 for you and hands back the
  sensor→world matrix — **prefer it whenever you want a metric cloud.**
  Both conventions are settable per prim
  (`omni:sensor:WpmDmat:elementsCoordsType`, `...:outputFrameOfReference`), so
  read them from the buffer header rather than assuming.
  **`VALID` does not mean "real".** With an empty scene the radar emits one
  element per frame at exactly `az 0°, el 0°, range 100 m`, flagged VALID. It is
  a no-detection placeholder; range-gating on `maxRangeM` (200) will not catch
  it.
- Radar needs Motion BVH, and it must be on **before renderer init** — the BVH
  is built there, so setting the carb values from Python afterwards is too late
  and silently does nothing. Two supported routes, one per execution model:
  - standalone: `SimulationApp({"enable_motion_bvh": True})`, which expands to
    the three settings internally;
  - exec mode: pass all three on the **kit command line**
    ```
    --/renderer/raytracingMotion/enabled=true
    --/renderer/raytracingMotion/enableHydraEngineMasking=true
    --/renderer/raytracingMotion/enabledForHydraEngines="'0'"
    ```
  `Radar._create_prim` raises only on the **first** of the three, so passing
  that one alone yields a radar that constructs cleanly and returns nothing.
  Assert all three at runtime; never infer BVH from the absence of an exception.
  *(Motion BVH itself is fine on this host — measured. The radar on top of it is
  not; see "Environment facts".)*
- Non-visual materials (what governs radar penetration) are USD attributes
  `omni:simready:nonvisual:*`, written by
  `isaacsim.core.experimental.materials.NonVisualMaterial`:
  ```python
  mat = NonVisualMaterial(path, bases="steel", coatings="paint", attributes="none")
  cube.apply_visual_materials(mat)   # yes, "visual" — that is the binding call
  mat.set_bases("cardboard")          # runtime swap, rewrites the USD attribute
  ```
  The attribute prefix comes from the carb setting
  `/rtx/materialDb/nonVisualMaterialSemantics/prefix`, not a hardcoded string.
  The 4.5-era CSV *material mapping* is gone, but three CSVs still ship as the
  authoritative name→index **specification** that `NonVisualMaterial` encodes
  into those attributes, at
  `exts/isaacsim.core.experimental.materials/data/specifications/`: **48 bases**,
  4 coatings (`none`, `paint`, `clearcoat`, `paint_clearcoat`), 5 attributes
  (`none`, `emissive`, `retroreflective`, `single_sided`,
  `visually_transparent`). Read the CSVs for the base list rather than guessing
  names. **`skin` is one of the bases — it is the honest base material for the
  avatar in S6.**

If you cannot verify an API against current docs, say so rather than guessing.

---

## Failure modes that produce NO error message

Ranked by how much time they cost.

1. **Avatar with no collision mesh.** The free-fly viewport camera is not a
   physical object — no mesh, no collision, no material. Lidar rays pass
   through it, cameras render nothing, radar returns nothing, segmentation has
   nothing to label. *Every sensor reading stays constant and nothing warns
   you.* This is the most likely failure of the entire design.
2. **RTX sensor without its own viewport.** It silently does not simulate.
3. **Missing semantic label.** Segmentation and bbox annotators return empty.
4. **Dynamic instead of kinematic avatar.** Ragdolls, trips on shelves, falls
   through floors.
5. **pip install into system python instead of `./python.sh`.** Package
   installs fine, Isaac Sim cannot see it.
6. **Cache volumes mapped to 4.5-era paths.** Docker creates them happily; they
   cache nothing; every restart costs ~10 minutes.

---

## Environment facts

- Isaac Sim **6.0.1**, container runs **rootless as uid 1234**. Host files it
  writes will be owned by 1234. That is expected, not a bug.
- Streaming needs **TCP 49100** (signaling) **and UDP 47998** (media). Opening
  only TCP gives a successful connection and a permanently black screen.
- `network_mode: host` is **mandatory**. Bridge networking lets signaling
  connect and media never arrive.
- **GPUs: four RTX 3090s, `nvidia-smi` indices 0–3.** (An earlier version of
  this file said three; the "cap at 2, reserve GPU 2 for inference" split was
  reasoned from that wrong picture and is re-derived below, not renumbered.)

  Two constraints, in priority order:

  1. **The rendering set must include `nvidia-smi` index 3.** NVENC needs the
     device with **minor 0** present in the container, and index 3
     (`0000:c1:00.0`) is that card. Without it every exposed GPU fails
     `nvEncOpenEncodeSessionEx` and streaming dies with signaling still
     connected — a permanently black client. This outranks any balancing
     preference. Re-derive after any driver or hardware change; do **not**
     assume it holds:
     ```
     grep -H 'Device Minor' /proc/driver/nvidia/gpus/*/information
     ```
     Confirmed 2026-08-12: smi 0→minor 3, smi 1→minor 1, smi 2→minor 2,
     **smi 3→minor 0**.
  2. **Cap rendering at 2 GPUs**, and prefer 1. Rendering scales sublinearly,
     multi-GPU does nothing for physics (largely CPU-bound), and on this host
     IOMMU is enabled — Isaac's own probe measures P2P at **11.2 GB/s vs
     830 GB/s** local, which cost ~30× per frame in practice. NVIDIA documents
     IOMMU-enabled P2P as unsupported on bare metal, so two rendering GPUs here
     is a config error, not a tuning knob.

  With four cards that leaves **two** GPUs free for inference, not one.
- **This is a SHARED machine — check GPU occupancy before believing any
  crash.** On 2026-08-12 another user's job held ~18.8 GB on GPUs 0, 1 and 3;
  Isaac got ~5 GB and died with `ERROR_OUT_OF_DEVICE_MEMORY`, which reads
  exactly like a renderer bug and is not one. First command after any GPU-side
  failure:
  ```
  nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv
  ```
- **Motion BVH is safe, and RTX radar works.** Measured in exec mode on an
  uncontended GPU: lidar + Motion BVH → 6.79 M points, **zero errors**; radar +
  Motion BVH → zero errors and correct geometry. *(An earlier version of this
  bullet said the radar "returns nothing usable" and does "not localize a 2 m
  cube 8 m away". That was wrong — it was the spike reading a spherical buffer
  as Cartesian metres. Corrected 2026-08-14, measured both ways.)* With the
  decode fixed the radar puts every return on the wall's near face at
  **3.900 m** — the true face is 3.90 — and tracks the target cube's near face
  across **5.500–6.501 m** as it oscillates ±0.5 m, with `rv_ms` ±9.27 m/s.
  Two real limits remain, neither a defect in the sensor:
  - it reports only its **strongest** returns (1–2 per frame under the default
    `cfarMode="2D"`), so a weak return behind an occluder never outranks the
    occluder's own echo;
  - **swapping the occluder's non-visual material changes nothing** — RCS is
    10.16017 dBsm for steel, concrete, cardboard, plastic, clear_glass and
    fabric alike. Any material claim from this rig is vacuous until that is
    explained.

  So: **radar is usable for geometry, unproven for material penetration.** See
  `sim/spikes/FINDINGS.md`. Nothing else in the project needs Motion BVH; S5–S9
  and S11 are unaffected.
- **Only one sim container at a time.** `network_mode: host` is mandatory (see
  above), and it means a second container dies binding Kit's `0.0.0.0:8011`
  with `[Errno 98] address already in use`. Putting them on different GPUs does
  not help — the collision is the port. Also: an exec-mode container does **not
  exit when the script prints `DONE`**; it lingers holding GPU memory until
  `docker stop`.
- Isaac Sim **cannot run on macOS**. The MacBook is a thin client plus a
  CPU-side dev machine. 6.x "multi-arch" means Linux ARM, not macOS.
- Tailscale on the server runs in **userspace mode** from
  `/home/quang/EmbodiedAgent/tailscale`, started with
  `--tun=userspace-networking`. There is deliberately no `tailscale0`
  interface, no systemd unit, and no binary on PATH — this is normal, not a
  broken install, and it will look absent to any check that greps for those.
  Do not install system Tailscale: there is no sudo on this host.
  Verify:  cd /home/quang/EmbodiedAgent/tailscale
           ./tailscale --socket=$HOME/.tailscaled.sock status
  Restart after reboot: see RESTART.md in that directory.
  Server has NO sudo. Host-level installs are impossible — ask the admin.
- **`ISAACSIM_HOST` is the LAN IP `143.248.57.94`, never the tailnet IP
  `100.96.11.37`.** This is the consequence of userspace mode above: with no
  `tailscale0`, the kernel has no tailnet route and hands that address to the
  default gateway, so it goes nowhere. Even this host cannot reach its own
  tailnet address. Measured on 2026-08-09:

  ```
  ip route get 100.96.11.37   ->  via 143.248.57.1 dev enp66s0f0   (default gw)
  connect 100.96.11.37:49100  ->  FAILED
  connect 143.248.57.94:49100 ->  CONNECTED
  ```

  `tailscale serve` is **not** a workaround — it forwards TCP only, so
  signaling would connect and UDP media would never arrive, producing exactly
  the black screen described three bullets up. **Consequence: streaming works
  only from a client that can reach the KAIST LAN.** Making the tailnet address
  usable needs a kernel TUN device, which needs root this server does not have
  — that is an admin request, not a task.

---

## Layers

```
Layer 4  AGENT / MODEL      core/memory/  — the closed loop; research goes here
Layer 3  OBSERVATION API    core/observation.py + core/mock_source.py
Layer 2  SENSOR REGISTRY    config/sensors.yaml + core/registry.py
Layer 1  SCENE (USD)        sim/ + scenes/
```

`sim/` is Isaac-specific and server-only. `core/` runs anywhere. Nothing in
`core/` may know that Isaac Sim exists.

**`tests/contract.py` must import neither the mock nor the simulator.** It is
the shared suite that `MockObservationSource` and the coming
`sim/observation_adapter.py` both have to pass; the moment it reaches for a
mock-only attribute it stops being a contract and becomes a second set of mock
tests. Mock-specific behaviour goes in `tests/test_mock_source.py`.

**Return real values from `sensor_ids` and `time` before wiring anything
else** (this will bite S11). `isinstance(src, ObservationSource)` calls
`hasattr` on every protocol member, `hasattr` evaluates a property, and a
property raising anything but `AttributeError` propagates. A half-built
adapter whose `sensor_ids` raises `NotImplementedError` therefore dies *inside*
`test_satisfies_the_protocol` with that error instead of failing readably —
and both are answerable from the constructor, with no simulator involved.

---

## Before every commit

```bash
make verify
```

Runs the layer-boundary check and the test suite inside the dev container.
Nothing touches the host environment.

Both must pass. If tests cannot run without a GPU, the boundary has leaked.