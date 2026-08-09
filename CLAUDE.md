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
   import.** Not after. Not alongside.

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
- Radar needs Motion BVH enabled via carb settings before creation:
  `/renderer/raytracingMotion/enabled`,
  `/renderer/raytracingMotion/enableHydraEngineMasking`,
  `/renderer/raytracingMotion/enabledForHydraEngines`.
- Non-visual materials (what governs radar penetration) are USD attributes
  `omni:simready:nonvisual:*`, not the old CSV mapping.

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
- Multi-GPU: cap at **2** rendering GPUs. GPU 2 is reserved for inference.
  Multi-GPU does nothing for physics, which is largely CPU-bound.
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

---

## Layers

```
Layer 4  AGENT / MODEL      core/memory/  — empty, research code goes here
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

---

## Before every commit

```bash
make verify
```

Runs the layer-boundary check and the test suite inside the dev container.
Nothing touches the host environment.

Both must pass. If tests cannot run without a GPU, the boundary has leaked.