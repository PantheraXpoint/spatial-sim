# Server task queue

Hand these to Claude Code on the Linux server, **one task per session**. Each
task is sized to fit one session and ends with a gate you can check yourself.

`CLAUDE.md` at the repo root is loaded automatically — you do not need to paste
its contents into any task.

---

## The actor split — read this first

Claude Code on the server can run bash, edit files, read logs, and run Isaac Sim
headless scripts. **It cannot click in the streamed viewport.** Your build order
is deliberately GUI-first, so tasks are labelled:

| Label | Who | Meaning |
|---|---|---|
| **[CC]** | Claude Code | Shell, files, scripts. Delegate freely. |
| **[YOU]** | You, in the GUI | Clicking, placing, discovering prim paths. |
| **[CC+YOU]** | Both | You act in the GUI, Claude Code turns the result into code. |

The **[YOU]** tasks are not overhead — they are where prim paths get discovered,
and prim paths are the one thing Claude Code must never guess. Every asset costs
10–20 minutes of clicking through the stage tree to learn its structure. That
cost is unavoidable and it is cheaper than debugging invented paths.

**Handoff protocol:** when you discover a prim path in the GUI, paste it into
`config/sensors.yaml` or `config/scene.yaml` yourself, or give it to Claude Code
to write there. That file is the contract. Claude Code reads paths from it and
never from memory.

---

## S0 — Host audit *(read-only)* **[CC]**

**Depends on:** nothing. Start here.

Ask Claude Code to produce a report, not changes:

> Audit this host for Isaac Sim 6.0.1 readiness. Report only, change nothing.
> Check: `ldd --version` (need ≥ 2.35), total RAM (`free -g`, need ≥ 32 GB,
> want 64), CPU core count, free disk, `nvidia-smi` present and how many GPUs
> it lists, driver version, whether Docker is installed and whether the current
> user is in the docker group, whether NVIDIA Container Toolkit is installed,
> and whether Isaac Sim is already installed anywhere on this machine. Then
> tell me which of these blocks progress and which are advisory.

**Gate:** you know whether the server is bare or already provisioned, and
whether RAM ≥ 32 GB. This resolves two of the plan's open decisions in about two
minutes and determines whether S1 is a checkbox or a full day.

---

## S1 — Host provisioning **[CC]**

**Depends on:** S0. **Skip entirely if S0 reports everything present.**

> Install what S0 found missing: NVIDIA production-branch driver, Docker, NVIDIA
> Container Toolkit. Use the official Isaac Sim container-installation
> instructions. Do not install the newest available driver — use a production
> branch release, since livestreaming has historically lagged on brand-new
> drivers. Then open firewall ports: TCP 49100, UDP 47998, TCP 8210.

**Gate:** `make verify-gpu` lists all **four** RTX 3090s (`nvidia-smi` indices
0–3). See CLAUDE.md → Environment facts for which of them may render: the
rendering set must include index 3, the minor-0 card, or NVENC fails and
streaming goes black.

> Note the UDP port explicitly when you check. Opening only TCP is the classic
> cause of "connects, black screen."

---

## S2 — Image build and compatibility **[CC]**

**Depends on:** S1. Requires you to have run `docker login nvcr.io` first —
Claude Code should not handle your NGC key.

> I have already run `docker login nvcr.io`. Copy `docker/.env.example` to
> `docker/.env` and set `ISAACSIM_HOST` to this machine's Tailscale IP (find it
> with `tailscale ip -4`). Then run `make dev-build` and `make verify`, and only
> if those pass, `make sim-build`. Then `make compat` and `make encoder-check`.
> Report what each one output.
>
> **Install nothing on this host.** Python dependencies belong in
> `docker/requirements-dev.txt` (dev container) or `docker/requirements-sim.txt`
> (installed into Isaac Sim's bundled python via `./python.sh`). The driver,
> Docker, and the Container Toolkit from S1 are the only host-level installs in
> this project.

**Gate:** compat prints `System checking result: PASSED`, and
`ldconfig -p | grep libnvidia-encode` returns a line. No NVENC means no
streaming, and you want to know that now rather than at S3.

---

## S3 — Streaming bring-up (M1) **[CC+YOU]**

**Depends on:** S2.

> Run `make stream` and watch the output. Tell me when the log line
> `Isaac Sim Full Streaming App is loaded.` appears — the first launch spends
> several minutes warming the shader cache, so distinguish warm-up from a hang.
> Then run `make ports` and confirm both TCP 49100 and UDP 47998 are listening.
> If the ports are not listening, diagnose before I try to connect.

**[YOU]:** connect from the MacBook. See MACBOOK.md M1.

**Gate:** empty stage visible on the MacBook, and you can fly with right-mouse
held plus WASD. Plain WASD does nothing — the right mouse button must be held.

**Then, immediately:**

> Stop and restart the stream. Time both launches. The second must be
> substantially faster. If it is not, the cache volumes are not mapping — check
> them against the rootless 6.x paths in docker-compose.yml.

Silent cache failure costs ten minutes per restart for the life of the project.
Catch it on day one.

---

## S4 — Radar spike *(out of milestone order, deliberately)* **[CC]**

**Depends on:** S3.

The see-through-shelf moment is the demo's entire headline, it is unverified,
and in the original plan it is scheduled last at M9. Verify it now in a
throwaway scene — thirty minutes here versus discovering at M9 that the
headline is fiction.

> Build a minimal throwaway scene — no warehouse, just a ground plane, one
> RTX radar, one RTX lidar, a simple box target, and one flat occluder between
> them. Use the 6.x `isaacsim.sensors.experimental.rtx` API and enable Motion
> BVH **before renderer init** — as a `SimulationApp({"enable_motion_bvh": True})`
> flag, or as kit command-line arguments under `runheadless.sh`. Setting the
> carb values from Python afterwards is too late and does nothing. Adapt
> `standalone_examples/api/isaacsim.sensors.**experimental**.rtx/create_radar_basic.py`
> rather than writing from scratch — read it first and tell me what it actually
> does.
> Then answer one question with evidence: does the radar return anything from
> the target when the occluder is between them, and does the lidar not?
> Test the occluder with different `omni:simready:nonvisual:*` material
> attributes and report which materials radar penetrates. Put this in
> `sim/spikes/` and do not integrate it into the main scene.

**Gate:** a yes/no answer with point counts.

> **STATUS 2026-08-12 — CLOSED AS NOT MEASURABLE.** The rig reports `INVALID`:
> radar runs without error but returns 1–2 detections per frame that do not
> localize the target and are unchanged when the target is deleted, so there is
> no valid clear-line-of-sight baseline to subtract an occluded reading from.
> Lidar in the same rig returns 6.79 M points cleanly. Motion BVH is fine; the
> earlier "renderer crash" was another user's job occupying the GPU. Full
> record and the one question that would reopen this:
> **`sim/spikes/FINDINGS.md`**.

**Interpreting it:** if radar penetrates nothing, the demo's key moment needs a
different framing — fall back to "radar sees a return where lidar sees only an
occlusion shadow," which is still a real multi-modal contrast. Also note that
real 77 GHz radar does not penetrate steel, so warehouse shelving is close to
the worst possible occluder. If penetration works at all, switch the demo
occluder to a plasterboard partition, cardboard stack, or plastic strip
curtain — both more honest and more likely to work.

---

## S5 — Scene load (M2) **[YOU]**, then **[CC]**

**Depends on:** S3.

**[YOU]** in the GUI: open
`Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd` if it
exists in your asset library. It ships with a warehouse, a worker character,
multiple cameras, and **semantic labels already applied** — roughly 60% of the
scene, pre-built and pre-labelled. Missing semantic labels is the usual reason
segmentation annotators return empty, so inheriting them is worth a lot.

Fall back to `Isaac/Environments/Simple_Warehouse/full_warehouse.usd`.
**`small_warehouse_digital_twin.usd` does not exist in the 6.0 asset set** —
verified 404. The real lighter-weight alternatives are `warehouse.usd`,
`warehouse_multiple_shelves.usd`, and `warehouse_with_forklifts.usd` in that
same directory.

> **Reconnaissance done headlessly 2026-08-12 — read before the GUI session:**
>
> - **You will need network.** The asset root is the online S3 bucket
>   `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`;
>   there is no local assets pack in the container.
> - **The primary resolves** (HTTP 200), at
>   `Isaac/Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd`
>   — note the leading `Isaac/`, which the path above omits.
> - **Stage:** default prim `/Root`, Z-up, 1.0 m/unit, ~9,000 prims. `/Root`
>   children: `forklift`, `GroundPlane`, `Lights`, `Warehouse`, `Cameras`.
>   Five cameras already exist: `Camera_01`, `Camera_02`, `Camera_Worker_Anim`,
>   `Camera_Forklift_Anim`, `Camera_Shelves_Anim`.
> - **CORRECTED 2026-08-25 — the warehouse IS labelled. This bullet was wrong.**
>   It read: *"Semantic labels are NOT already applied to the warehouse. Only 11
>   prims carry semantics, all of them parts of the worker character
>   (`/Root/Worker/ManRoot/Worker/CC_Base_Body`, `Field_Jacket`, `Baseball_cap`,
>   …). The shelving, floor, walls and forklift are unlabelled."*
>
>   Audited on the composed stage: **3,441 of 3,493 renderable prims carry a
>   direct label — 98.5% by count, 99.0% by bounding-box volume — across 37
>   classes.** `box` 1841, `rack` 432, `sign` 253, `pallet` 158, `wall` 122,
>   `bottle` 120, `ceiling` 72, `floor_decal` 72, `lamp` 60, `bracket` 59,
>   `pillar` 54, `floor` 54, `crate` 51, `barel` 37 (sic), and on down. Only
>   **41** renderable prims carry nothing at all: 40 ceiling beams
>   (`SM_BeamA_9M`) and one forklift fork.
>
>   **Why 2026-08-12 read 11:** it looked for the 6.x `UsdSemantics.LabelsAPI`
>   only. **3,467 of the 3,480 label entries are on the deprecated
>   `Semantics.SemanticsAPI`** (`semantic:<inst>:params:semanticType` /
>   `semanticData`). The 11 it found are the Worker's; the 13 on the current
>   schema are the avatar's, authored later by `sim/avatar.py`.
>
>   **Isaac Sim 6.0.1's annotators read BOTH schemas** — confirmed by render,
>   not by inspection: a camera on the racking returns `rack` at 24.1% of pixels
>   and `box` at 2.6%, frame 98.8% labelled; the avatar returns `person`. So the
>   "60% pre-labelled" premise held for the environment as well as the
>   character. `sim/spikes/_diag_semantics_audit.py`,
>   `sim/spikes/_diag_semantics_render.py`; derivation in
>   `sim/spikes/FINDINGS.md`.

Save as `scenes/observatory_base.usd`.

Then click through the stage tree and write down the real prim paths for
anything you plan to attach to.

**[CC]:**

> I saved `scenes/observatory_base.usd` and here are the real prim paths I read
> off the stage tree: [paste]. Update `config/scene.yaml` with the actual asset
> path I used and these confirmed paths. Then write `sim/load_scene.py` that
> opens the saved stage headless and prints the top three levels of the stage
> tree, so we have a scriptable way to re-check paths later. Do not create any
> prims.

**Gate:** warehouse visible in the stream; `sim/load_scene.py` prints a tree
whose paths match what you saw in the GUI.

---

## S6 — The avatar (M3) **[CC+YOU]** ← highest-risk task in the project

**Depends on:** S5. **Do this before placing any sensor.** Every sensor added
afterwards is then immediately testable. A broken M3 makes M4 undebuggable.

Give this one its own session and do not bundle it with anything.

> Build the avatar in `sim/avatar.py`. Structure:
>
> ```
> /World/Avatar              Xform, keyboard-driven, KINEMATIC
>   ├── body_mesh            capsule or character mesh
>   │     ├── collision mesh REQUIRED for ray-based sensors
>   │     └── semantic label class: person
>   ├── cam_first_person     head height, forward
>   └── cam_third_person     behind and above
> ```
>
> All four properties are load-bearing and each fails silently if missed:
> collision mesh (lidar/radar need geometry to bounce off), semantic label
> (segmentation returns empty without it), kinematic not dynamic (dynamic
> ragdolls and falls through floors), cameras parented to the Xform (so
> switching first/third person is just rebinding the viewport).
>
> Drive the Xform, not the cameras. Reuse existing wiring: Isaac Examples →
> Input Devices → **Omnigraph Keyboard** already connects keyboard input to prim
> motion — read it and point it at the avatar rather than writing new input
> handling.
>
> Then write `sim/verify_avatar.py` that asserts, headless: the collision API is
> present on the mesh, the semantic label exists and reads `person`, the rigid
> body is kinematic, and both cameras are descendants of `/World/Avatar`. Make
> it exit non-zero on any failure.

**Gate — two parts, both required:**

1. `sim/verify_avatar.py` exits 0.
2. **[YOU]:** in third person, you can see your own body, and walking into a
   shelf stops you rather than passing through or ragdolling.

> Do not advance to S7 on part 1 alone. The whole point of this task is that
> code-level checks and visual reality can disagree here.

---

## S7 — First station camera (M4) **[CC+YOU]**

**Depends on:** S6 passing both gates.

**[YOU]:** place the INFRA_01 station Xform in the GUI at ceiling height, note
its real prim path and transform.

**[CC]:**

> INFRA_01 is at prim path [paste], position [paste]. Update
> `config/sensors.yaml` with the confirmed paths. Then write
> `sim/sensor_factory.py` that reads the registry via `core.registry` and
> creates sensors from it — start with cameras only. Create one viewport per
> sensor, because **each RTX sensor must be attached to its own viewport or it
> silently does not simulate.** Build the panel layout before pressing Play.

**Gate:** a viewport panel showing you walking past, in the stream.

> **Do not re-dock or rearrange windows while an RTX Lidar sim is running — it
> will likely crash Isaac Sim.** Design the layout once, save it, then stop
> touching it. Pause before rearranging.

---

## S8 — Lidar and debug draw (M5) **[CC]**

**Depends on:** S7.

> Extend `sim/sensor_factory.py` to create the RTX lidar from the registry,
> using the 6.x API. Attach the debug draw writer —
> `LidarSensor.attach_writer("draw-point-cloud")` in 6.x, **not** the
> `RtxLidarDebugDrawPointCloudBuffer` Replicator writer that 5.x examples use.

**Gate:** point cloud rendering live in the 3D scene, and it changes as you
walk.

Expect a cluster of points **on** the avatar plus an occlusion shadow on the
floor behind it. The "person-shaped void" is the shadow, not the avatar. If you
see no cluster on the avatar at all, S6 regressed — go back, do not proceed.

This is the highest visual-impact-per-minute item in the project and the proof
that the avatar is physically real to the sensors.

---

## S9 — Robots (M6) **[YOU]**, then **[CC]**

**[YOU]:** place TurtleBot3 Burger (~0.2 m), Unitree Go2 (~0.4 m), Unitree H1
(~1.7 m). Note real prim paths. Legged robots **collapse on spawn** without a
locomotion policy — that is why their articulations are disabled. Confirm they
stay upright.

**[CC]:** add three RGB-D cameras from the registry at the confirmed paths.

**Gate:** three panels at three heights. The ceiling camera sees your top, the
Go2 sees your legs, the humanoid sees your face. Same event, radically different
observations — that contrast *is* the point of this task.

> **AMENDED 2026-09-02 — the platforms are no longer static.** Each robot now
> carries a dynamic physics proxy at its real hardware mass (1 / 15 / 47 kg,
> `sim/nav_obstacles.py`), so walking into one displaces it. The articulation
> stays disabled and the robot is written from its proxy every frame, so
> nothing has become self-propelled and nothing collapses.
>
> **What this task's gate now has to say out loud:** the three heights are an
> initial condition, not a constant. The contrast between 0.2 m, 0.4 m and
> 1.7 m is still the point, and it is still there on the opening frame of every
> episode — but if the avatar has shoved the Go2 across the aisle, the "Go2
> sees your legs" panel is looking somewhere else, and that is now a legitimate
> state of the world rather than a bug. Check the contrast BEFORE walking into
> anything, and treat a displaced platform as a new pose to be read from
> `observation_adapter` (which publishes it live) rather than as the pose in
> `config/scene.yaml`.
>
> `GUI_NAV_OBSTACLES=0` restores the old behaviour if you want the original
> gate back for a session.

---

## S10 — Semantics and radar (M7, M9) **[CC+YOU]**

**[YOU]:** apply semantic labels via Tools → Replicator → Semantics Schema
Editor. Enable the Synthetic Data Visualizer from the viewport visualizer icon.

**[CC]:** ~~wire the radar per the S4 findings.~~ **The S4 findings say radar is
not usable on this host today — see the STATUS block under S4 and
`sim/spikes/FINDINGS.md`.** Do not plan radar into this task. Semantics is the
whole of S10 unless S4 is reopened and answered first.

**Gate:** segmentation overlay tags you as `person`. ~~radar returns visible~~ —
dropped; see above.

> If radar is ever revived: add it **last** and toggle Motion BVH off when not
> demoing it. It raises VRAM and render cost for **all** sensors, so everything
> slows down after that lands. That slowdown is expected, not a regression.
> Measured caveat: Motion BVH itself is harmless here (lidar + BVH ran clean);
> it was never the problem.
>
> ~~**Semantics is the load-bearing half of this task and it is bigger than the
> plan assumed** — the warehouse asset ships with only the worker character
> labelled, not the environment.~~ **Retracted 2026-08-25: this repeated the S5
> reconnaissance error, corrected above.** The environment is 98.5% labelled and
> the annotators read the deprecated schema it is labelled with, so **S10 is
> much smaller than planned, not bigger.** What is actually left is a labelling
> *quality* question rather than a coverage one:
>
> - the avatar is `person`, but `/Root/Worker` — the other human in the scene —
>   is `fieldjacket` / `cargopant` / `basebody` / `baseballcap`, never `person`.
>   Two humans, two vocabularies. That is a ground-truth defect for a navigation
>   benchmark and the Semantics Schema Editor is the tool for it;
> - 41 prims carry nothing (40 ceiling beams, one forklift fork);
> - the asset spells it `barel`.
>
> Migrating the 3,467 deprecated-schema labels to `UsdSemantics.LabelsAPI` is
> **optional and undecided** — segmentation works without it. See
> `sim/spikes/FINDINGS.md`.

---

## S11 — Observation API (M10) **[CC]**

**Depends on:** S8 at minimum.

> Write `sim/observation_adapter.py` that reads any registry sensor and returns
> a `core.observation.Observation`. This is the *only* place simulator types
> convert to plain dicts. Check `MACBOOK.md` M4 first — the MacBook track has
> been building against this contract with a mock source, and the two must
> agree. Run `./scripts/check_layer_boundary.sh` when done.

**Gate:** the same test that passes against the MacBook's mock source passes
against the live simulator.

---

## S12 — Inspector panel (M11) **[CC]**

**Build this last**, after everything renders. It becomes the debugging tool you
use for years.

> Write the `ext/sensor_inspector/` omni.ui extension. It watches the current
> selection; if the selection is a registered sensor, it displays frame counter,
> point count, min/max depth, detected semantic classes, and timestamp. One
> panel, text readout, a few Hz. No images or charts — those already exist as
> viewports.

**Gate:** click a sensor, see live readings.

---

## Stopping points

- **S3–S7 done** is already a working demo of the plumbing.
- **S3–S10 done** is the complete, convincing demo.
- **S11–S12** is the foundation investment that makes the research code possible.

Do not let the demo's polish substitute for the research claim when presenting
it. This shows that sensors exist, data flows, and a human can navigate the
space. It shows nothing yet about shared memory, fusion, or change detection —
which is correct for stage one, but worth saying out loud.