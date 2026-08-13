# S4 — radar spike findings

## THE GMO COORDINATE BUG — 2026-08-14, and it supersedes the verdict below

**The GMO fields are spherical, in degrees and metres. The spike read them as
Cartesian metres. That single mistake produces every anomaly in this file, for
both sensors, and it is arithmetically sufficient — nothing else is needed to
explain the numbers.**

Established by reading only; no run was made.

### What the fields actually are

From the shipped `GenericModelOutput` structure documentation (`BasicElements`,
per-element arrays of length `numElements`):

| Field | When `coordsType == SPHERICAL` | When `CARTESIAN` |
|---|---|---|
| `x` | **azimuth, degrees**, [-180, 180] | x, metres |
| `y` | **elevation, degrees** | y, metres |
| `z` | **distance, metres** | z, metres |
| `scalar` | lidar: normalized intensity · **radar: RCS in dBsm** | same |
| `flags` | `ElementFlags` bitmask; `VALID` bit marks a usable element | same |

and the two defaults that matter:

```
coordsType        default = CoordsType::SPHERICAL
frameOfReference  default = FrameOfReference::SENSOR
```

So the buffer is **spherical by default**, and **sensor-local by default**. The
annotator `IsaacExtractRTXSensorPointCloud` exists precisely because of this: it
"performs spherical-to-Cartesian conversion when the GenericModelOutput buffer
contains spherical coordinates, and outputs a sensor-to-world transform matrix."
Reading `generic-model-output` raw, as this spike does, opts out of both steps.

Sources: [GenericModelOutput structure](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/docs/source/generic_model_output/generic_model_output.html),
[RTX Sensor Annotators](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_annotators.html).

### The masks were therefore nonsense, and guaranteed zero

`_sample()` in `radar_penetration_exec.py` applies, to every sensor:

```python
tmask = (x > 6.0) & (x < 10.5) & (np.abs(y) < 2.5) & (np.abs(z) < 2.5)
wmask = (x > 3.0) & (x < 5.0)
```

Decoded correctly, that reads: *azimuth between 6° and 10.5°, elevation within
±2.5°, and **range under 2.5 m***.

- The target sits at `x = 8, y = 0` — dead ahead, **azimuth 0°**. It can never
  satisfy `azimuth > 6°`.
- Independently, **`|z| < 2.5` accepts only returns closer than 2.5 metres.**
  Nothing in the scene is inside 2.5 m except ground directly beneath the
  sensor, which is at elevation steeper than −20° and so fails `|y| < 2.5`.

**`target_points == 0` was structurally guaranteed, in every condition, for
every material, for both sensors, before a single ray was cast.** Two
independent guarantees, either one sufficient. No occluder was ever being
measured.

`wmask` is worse than wrong, it is *informative*: it selects a fixed **2° wedge
of azimuth** and nothing else — no range bound at all. A rotary lidar puts
essentially the same number of returns into a fixed angular wedge every
revolution regardless of what is in it, which is exactly the observed
signature:

| Condition | occluder | lidar total pts | lidar `wall_points` |
|---|---|---|---|
| `no_occluder_moving` | 1,000 m away | 6,786,180 | **76,326** |
| `no_occluder_static` | 1,000 m away | 6,767,949 | **76,350** |
| `no_target` | at 4 m | 9,119,172 | **76,332** |
| `empty` | 1,000 m away | 6,768,801 | **76,317** |

Totals swing by 35% as geometry changes; the "wall" count moves by **33 points
out of 76,000 — 0.04%** — *including* the row where the wall is a kilometre
away. That was read in the previous session as "the lidar cloud is not in the
frame the spike assumes". It is better than that: it is a **solid-angle count**,
constant by construction, carrying no occlusion information whatsoever.

(The wedge holds 1.12% of the cloud where 2°/360° predicts 0.56% — a factor of
two, which multi-echo returns or a sub-360° scan would account for. The absolute
number is not the point; the near-constancy is.)

### Decoding the radar numbers — it was measuring, and it found the wall

Re-read the four control rows as (azimuth°, elevation°, **range m**):

| Condition | scene | azimuth | elevation | **range** |
|---|---|---|---|---|
| `no_occluder_static` | ground + target @ 8 m | −0.2845° | 0.0009° | **6.1551 m** |
| `empty` | ground only | −0.2845° | 0.0009° | **6.1551 m** |
| `no_target` | ground + **wall @ 4 m** | −0.1477° | 0.0006° | **3.9002 m** |

The wall is a cube centred at `x = 4.0` with `WALL_SCALE` x-component `0.1`. Its
near face is at **3.90 m** or 3.95 m depending on whether `scales` is read as
full extent or half-extent. The radar reports **3.9002 m**.

**The radar localizes the wall to within a millimetre. It was never broken.**

What it does *not* do is see the 2 m steel cube in clear line of sight: the
target-present and empty scenes return the identical 6.1551 m. So the honest
statement is not "radar returns nothing interpretable" but:

> The radar returns **one detection per frame — the single strongest return.**
> With the wall present, that is the wall, at the correct range. With only a
> ground plane, it is a fixed 6.155 m clutter return. A 2 m steel cube at 6–7 m
> never outranks that clutter return, so it is never the one detection emitted.

That reframes the open question from *"what do these fields mean"* — answered —
to a bounded, specific one: **why only one return per frame.** That is a
property of the radar profile (returns-per-beam, detection threshold, RCS or
Doppler gating), and it lives in the `OmniSensorGenericRadarWpmDmatAPI` schema,
which has not been read.

Two supporting details:

- **`scalar` = 11.5 for radar is RCS in dBsm** — a large, entirely plausible
  radar cross-section for a 12 m × 4 m concrete wall. It was recorded in this
  file as an uninterpreted "wall intensity". Lidar's 2 × 10⁻⁴ is a *normalized*
  intensity, so its smallness is likewise expected, not a symptom.
- The moving condition's 72 detections spread to azimuth 18° and elevation 10°
  at ranges 5.5–6.7 m. The target is at azimuth 0°, range 7.5–8.5 m, so those
  are still not the target. With Motion BVH on and the target **teleported**
  each frame by `set_world_poses`, the implied velocities are non-physical;
  motion-induced ghosts are the natural reading. Not established.

### Answering the three questions directly

**1. What frame are x/y/z in, and is a transform missing?**
Spherical (azimuth°, elevation°, range m) by default, in the **sensor's own
frame** by default. Two conversions are missing, not one: spherical→Cartesian,
*then* sensor→world. The spike does neither. Here the sensor sits at the origin
with identity orientation, so the second conversion is nearly free — but it will
not be for any real sensor station in `config/sensors.yaml`.

**2. What is the 30-detections-with-zero-variance signature?**
**None of the offered options.** Not a sentinel, not a header misread, not a
wrong stride or offset, not uninitialised memory. The buffer was parsed
correctly and the values are real measurements; they were interpreted in the
wrong coordinate system. Zero variance across 30 frames is the *expected*
result of a deterministic renderer pointed at a static scene: the same single
strongest return, every frame.

And the premise "identical between the target scene and the empty scene" — while
literally true of those two rows — concealed the decisive third row: introducing
the wall moves the value to 3.9002 m. **It does change when the scene changes.**
This file previously asserted it did not, and that assertion was what made
"uninitialised memory" look plausible. Withdrawn.

**3. One bug or two?**
**One bug, one root cause, both sensors.** `_sample()` is shared; both sensors
emit the same buffer format with the same defaults. Lidar's zero-in-box is the
`|range| < 2.5 m` clause; radar's is the same clause plus the azimuth clause.
The previous entry — "both sensors returning geometrically implausible
coordinates points at a shared cause — most likely the GMO frame/convention" —
had the right instinct and the wrong severity: it is not implausible data, it is
correctly-shaped data measured with the wrong ruler.

### Two further defects in the same function, found while reading

Neither caused the observed numbers; both would corrupt a corrected run.

1. **`flags` is never checked.** `ElementFlags` carries a `VALID` bit. The spike
   consumes all `numElements` entries unconditionally, so invalid elements are
   counted as detections.
2. **`rv_ms` is never read.** `aux_output_level="BASIC"` was set specifically to
   populate radial velocity in `auxiliaryData`, and radial velocity is the exact
   quantity that would confirm or kill the Doppler-gating hypothesis for the
   static target. It is being requested and discarded.

### Why this is a Layer-1 finding and not a spike leftover

`core/observation.py` maps `generic-model-output → "points"`, and the contract
says `points  (N, 3) float  metres`. **Nothing in Isaac hands you that.** Any
consumer of this annotator owes three steps before the contract is satisfied:

| # | Step | How it fails silently if skipped |
|---|---|---|
| 1 | Check `coordsType`; convert spherical→Cartesian | Degrees are read as metres. Plausible-looking numbers, wrong everywhere. |
| 2 | Check `frameOfReference`; apply sensor→world | Every station's cloud sits at the origin, mutually overlapping. |
| 3 | Mask on `flags & VALID` | Invalid elements counted as returns. |

Concretely: **S11's `sim/observation_adapter.py` must do all three, or emit
`points` that violate the Layer-3 contract while passing every type check** —
`(N, 3) float` is satisfied by degrees just as well as by metres. That is the
failure class this project keeps meeting, arriving one layer up.

S8's debug draw is *probably* safe — `attach_writer("draw-point-cloud")` and
`IsaacExtractRTXSensorPointCloud` do the conversion internally — but "probably"
is doing real work in that sentence and it has not been verified. S10 reads the
same buffer and is exposed.

**Prefer `IsaacExtractRTXSensorPointCloud` over raw `generic-model-output`**
wherever a point cloud in metres is what is wanted. It performs step 1 and hands
back the matrix for step 2. Raw GMO is for when the auxiliary channels are
needed.

### What this does to S4's status

S4 is **still not answered, and the reason has changed.** It is no longer "the
radar returns nothing interpretable" — that was the measurement instrument
misreading a working sensor. It is now:

- the spike's classification masks are wrong and must be rewritten in spherical
  coordinates (or against the extracted Cartesian cloud);
- the radar emits one return per frame, and until that is understood or
  configured away there is no target detection to compare against an occluded
  one — with or without correct masks.

The first is a fix. The second is a question with a named place to look. **A
re-run is required to close S4, and per the standing instruction it is not being
made here.** Reading has taken this as far as reading goes.

**What the fix costs:** rewriting `_sample()`'s two masks against
(azimuth, elevation, range) — the target subtends about ±7° of azimuth at 8 m,
so `|az| < 10°`, `|el| < 10°`, `6 < range < 10.5` is the direct translation —
plus the `flags & VALID` mask. The controls, the material sweep, `SPIKE_SENSORS`
and `extents_xyz` all stand unchanged.

> **Note for CLAUDE.md, not applied here per instruction.** The "Environment
> facts" bullet reading *"RTX radar returns nothing usable … only ~1–2
> detections per frame that do not localize a 2 m cube 8 m away and are
> unchanged by removing the target"* is now known to be wrong in its diagnosis.
> The radar localizes a wall to the millimetre. The API-era section is also
> worth extending with the spherical/sensor-frame defaults, which are exactly
> the kind of thing that section exists to catch.

---

## CLOSING VERDICT — 2026-08-12 (superseded above)

> **Superseded 2026-08-14.** The paragraph below is accurate about what was
> *observed* and wrong about what it *meant*. Specifically: "do not localize",
> "sit at implausible coordinates", "unchanged when the target is deleted" and
> "the field usage is not the bug either" are all artifacts of reading spherical
> coordinates as Cartesian. Kept unedited as the record of the wrong turn.

S4 asked one question: **does RTX radar return something from a target that an
occluder hides from lidar, and which `omni:simready:nonvisual:*` materials does
it see through?** That question is **unanswered, and the rig reports itself
INVALID rather than producing a number.** What was measured: offscreen sensor
capture does not work under `SimulationApp` on this host at all, and does work
under `runheadless.sh --exec`, so the spike was rebuilt in exec mode; there,
lidar with Motion BVH enabled returns **6,786,180 points over 30 frames with
zero errors**, while the radar under identical conditions on an uncontended GPU
returns **1–2 detections per frame** that do not localize a 2 m cube 8 m away,
sit at implausible coordinates (z ≈ 5.5–6.7 m for a sensor and target both at
z = 1 m), and are **unchanged when the target is deleted entirely** — an empty
scene yields the same 30 detections as a scene with a target in clear line of
sight. The rig's own baseline check therefore fires: *"radar saw nothing with a
clear line of sight → TEST INVALID"*, and per its design it prints that instead
of a table of zeros that would read like "penetrates nothing". What could not be
measured is penetration itself, because with no working clear-line-of-sight
baseline there is nothing to compare an occluded reading against — the
subtraction that the whole experiment rests on has no valid minuend. **Both the
`tasks/SERVER.md` headline ("radar sees through the shelf") and its stated
fallback ("radar sees a return where lidar sees only an occlusion shadow")
require radar to return something interpretable, so both are unavailable and the
demo needs a different framing** — the honest multi-modal contrast that *is*
available today is lidar geometry plus cameras plus segmentation, all of which
work.

**Not the reason, though it looked like it:** the earlier 122-error CUDA crash
that closed the previous session was very likely **GPU contention, not a
renderer defect**. This is a shared machine; on 2026-08-12 another user's job
held ~18.8 GB on GPUs 0, 1 and 3, and the radar run on GPU 3 died with 34,499
`ERROR_OUT_OF_DEVICE_MEMORY` errors. Re-run on the free GPU 2, the same radar
configuration produced **zero errors**. Motion BVH is *not* hazardous here, and
the board decision prepared for "if Motion BVH alone crashes" does not apply.

> **Corollary that is now a project invariant:** check
> `nvidia-smi --query-compute-apps=...` before attributing any GPU-side failure
> to the code. Two sessions of this investigation were spent on symptoms that a
> neighbour's memory footprint explains.

**Where it stands:** radar is not usable on this host today, but the reason is
narrower and more tractable than "the interop bug" — the sensor runs, the
renderer is healthy, and the remaining unknown is the meaning of the radar GMO
detections. That is a bounded question, not an open-ended one. Nothing else in
the project depends on it: S5–S9 and S11 need no Motion BVH and no radar.

---

## Bottom line, in order of confidence

1. **Capture works under `runheadless.sh`.** Measured: camera rgb 32,700 px,
   lidar GMO 460,800 points, orchestrator `Status.STARTED` on all 60 frames.
2. **Capture is dead under `SimulationApp`**, at every render config and GPU
   count tried, with the orchestrator `STOPPED`.
3. Therefore the launcher is the variable. Everything below is the record of
   what was eliminated on the way, so it does not get re-tested.

---

## Hypotheses tested and ELIMINATED

Kept in full deliberately. A one-line recap drops these and the same work gets
done twice.

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| 1 | `omni.hydratexture` extension disabled | **Wrong** | Not an extension at all — it is a carb plugin, so `is_extension_enabled` returning `False` carries no information. The real extension `omni.kit.hydra_texture` is enabled, and the plugin registers and initialises normally in the info log. |
| 2 | Multi-GPU + IOMMU P2P | **Not the cause of capture failure** | Collapsing to `device_ids: ["3"]`, `multi_gpu=False`, one CUDA device — capture still empty. It *is* a real 30× perf bug (§ below). |
| 3 | CUDA↔Vulkan device-index ambiguity | **Eliminated by construction** | One GPU, one CUDA device, one `/dev/nvidia0`. Nothing left to disagree. (Direct UUID comparison not completed — see caveat below.) |
| 4 | Vulkan external-memory export failing (`vkGetMemoryFdKHR`) | **Refuted** | Real Kit log at `info`, 10,281 lines, from a run reproducing every symptom: 0 occurrences of `vkGetMemoryFdKHR`, `shared handle`, `external memory`; 0 `[Error]`; `carb.cudainterop.plugin` registers *and initialises* cleanly. |
| 5 | Async rendering incompatible with annotator readback | **Refuted, backwards** | The **working** path has `/app/asyncRendering = True` and `asyncRenderingLowLatency = True`. The **broken** path has both `False`. Opposite of the prediction. |
| 6 | Kit experience file (`base.python` vs `full` vs `full.streaming`) | **Not the variable** | All three fail under `SimulationApp`. Loading the streaming *experience* through `SimulationApp` is not the same as using the streaming *launcher*. |
| 7 | Orchestrator being `STOPPED` is the cause | **Eliminated** | `rep.orchestrator.run()` in the standalone path reaches `Status.STARTED` on **all 60 frames** — and capture is *still* empty. Orchestrator status and capture are decoupled. |
| 10 | Motion BVH is fatal on this host | **Refuted 2026-08-12** | lidar + BVH, exec mode, uncontended GPU: 6,786,180 points, **0 `[Error]` lines**. The previous session's leading hypothesis. |
| 9 | The renderer collapse is a CUDA↔Vulkan interop defect | **Superseded — it was GPU contention** | Identical radar run: GPU 3 with a neighbour holding 18.8 GB → 34,499 `ERROR_OUT_OF_DEVICE_MEMORY`; GPU 2 free → **0 errors**. |
| 8 | `asyncRendering=False` is the one-line fix | **Eliminated** | Forced `--/app/asyncRendering=true` + `asyncRenderingLowLatency=true` at launch under `SimulationApp`, verified read back as `True`, with `orchestrator.run()` as well. Capture still 0 px / 0 GMO. Dropped as instructed; launcher-diff thread NOT opened. |

### Note on hypothesis 5

The premise is half right and worth recording. `omni.replicator.core`'s
`orchestrator.py` does treat async rendering as incompatible — it caches the
value and forces `/app/asyncRendering = False` while running:

```
orchestrator.py:1067   # disable async rendering unless asyncRendering is specified
orchestrator.py:1069   if _settings.get_as_bool("/app/asyncRendering") != async_rendering:
orchestrator.py:610    carb.settings.get_settings().set("/app/asyncRendering", False)
orchestrator.py:642    carb.settings.get_settings().set("/app/asyncRendering", cached_async_rendering)
```

So Replicator manages the conflict itself — *when it runs*. In the working path
it runs, flips async off for the duration, and capture succeeds despite the
setting starting `True`. In the broken path the orchestrator never starts, so
the setting is irrelevant. Forcing both `False` in the standalone path would
have been a no-op: they were already `False` in every failing run.

---

## Hypothesis 4 in detail — the vkGetMemoryFdKHR signature

Tested because README §9.3 records this signature occurring once on this host:

```
[Error] [omni.rtx] VkResult: ERROR_INITIALIZATION_FAILED
[Error] [omni.rtx] vkGetMemoryFdKHR failed.
[Error] [omni.rtx] Cannot create shared handle for resource!
[Error] [carb.scenerenderer-rtx.plugin] Failed to allocate 1280x720 LdrColor resource
[Error] [omni.usd.multitick.render] multiTickRateRender() returned null
```

It predicts every symptom, which is why it was worth testing. It is not what is
happening here.

**Broken path, real Kit log, `--/log/level=info`, single GPU, 10,281 lines:**

| Term (`grep -ciF`) | Count |
|---|---|
| `[Error]` / `[Fatal]` | **0** / **0** |
| `vkGetMemoryFdKHR` | **0** |
| `shared handle` | **0** |
| `external memory` | **0** |
| `interop` | 7 — all plugin lifecycle |
| `gpu.foundation` | 4 — registration/init, no failure |

An `[Error] [omni.rtx]` line of that kind would appear in a Kit log at `info`,
so its absence here is meaningful. **This refutation stands.**

### Correction to my earlier cross-check

The previous version of this document also claimed the signature "appears in
neither the broken nor the working path", from grepping the streaming **Kit
logs** in the log volume. That comparison was invalid: README §9.3 greps the
**stdout redirect** produced by the §9.4 recipe, and the one confirmed
occurrence on this host was captured that way. Kit-log-volume files are a
different file class, so a zero count in them says nothing about whether the
signature ever occurred in the working path. **The "neither path" claim is
withdrawn.** Only the broken-path refutation is supported.

### Correction to how I read the log at all

An earlier version claimed "nothing logs an error" from
`grep -c '[Error]' <stdout>` returning `0`. Invalid twice over:

- `[Error]` is a **bracket expression** matching any line containing `E`, `r`,
  or `o` — it matches nearly everything. A `0` means the searched stream was
  effectively empty.
- I was grepping **stdout, not the Kit log**. `SimulationApp` launches Kit with
  `--portable`, which writes the log somewhere the container discards; the
  persistent volume holds **no** entry for any standalone run. And redirecting
  with `--/log/file=/workspace/...` silently produced no file, because
  `/workspace` is owned by uid 1004 while the container runs as uid **1234**.

**How to get a real log from a standalone run:** write it into the logs volume,
which uid 1234 owns.

```bash
./python.sh -u <script>.py \
  --/log/enabled=true --/log/level=info \
  --/log/file=/isaac-sim/.nvidia-omniverse/logs/kit_diag.log
```

(via `SimulationApp`'s `extra_args`). Then grep with `grep -F`, never `grep`
with brackets.

---

## The launcher bisect — the test that landed

`runheadless.sh --exec /workspace/sim/spikes/_diag_exec.py`

`--exec SCRIPT ARGS...` is confirmed present in 6.0 (`kit --help`), and
`runheadless.sh` forwards `"$@"`. The script must **not** construct a
`SimulationApp` — Kit is already up — and must drive frames from the update
event stream rather than calling `app.update()` in a loop, which would re-enter
the main loop.

```
  /app/asyncRendering           = True
  /app/asyncRenderingLowLatency = True
  /omni/replicator/captureOnPlay= True
  /renderer/multiGpu/enabled    = False

  frames sampled       : 60
  camera rgb max px    : 32700
  lidar GMO max points : 460800
  orchestrator status  : {'Status.STARTED': 60}
  VERDICT              : CAPTURE WORKS under runheadless.sh
```

Two incidental warnings from that run, both expected and both actionable for
the spike:

- `Multi-tick is enabled but motion BVH is not active. This is not supported.`
  and `MotionBVH for lidar model not enabled.` — `runheadless.sh` does not set
  Motion BVH, so **radar will not work there** until it is passed explicitly:
  `--/renderer/raytracingMotion/enabled=true`.
- `DLSS increasing input dimensions: Render resolution of (64, 64) is below
  minimal input resolution of 300` — render-product resolution, harmless here.

---

## Exec mode: built, runs — and Motion BVH crashes the renderer

`radar_penetration_exec.py` is the Route 2 port: no `SimulationApp`, frames
driven off the update event stream, config via environment variables (Kit's
`--exec SCRIPT ARGS...` makes trailing-argv parsing ambiguous), results
appended + `fsync`'d per condition.

**It launches and initialises correctly.** All three Motion BVH settings were
asserted at runtime rather than inferred from the absence of an exception:

```
=== MOTION BVH ===
  /renderer/raytracingMotion/enabled                  = True
  /renderer/raytracingMotion/enableHydraEngineMasking = True
  /renderer/raytracingMotion/enabledForHydraEngines   = '0'

created radar /World/radar and lidar /World/lidar
subscribed to update stream; measurement starting
```

Then rendering collapses. 122 `[Error]` lines, distinct kinds:

| count | error |
|---|---|
| 28 | `[carb.cudainterop.plugin] CUDA error 700: cudaErrorIllegalAddress` |
| 13 | `[omni.usd.multitick.render] Sensor endFrame() failed after null multiTickRateRender()` |
| 12 | `[carb.cudainterop.plugin] Failed to allocate CUDA host memory (size: 72806752)` |
| 12 | `[carb.cudainterop.plugin] Failed to allocate CUDA host memory (size: 62824)` |
| 10 | `[omni.physx.plugin] Cuda context manager error, simulation will be stopped` |
| 10 | `[carb.scenerenderer-rtx.plugin] Render Graph on device 0 has failed during recording` |
| 13 | `[omni.usd.multitick.render] multiTickRateRender() returned null` |
| 5 | `[omni.rtx] Failed to start frame in pooled data manager. Semaphore wait timed out after 60 seconds.` |
| 3 | `[omni.physx.plugin] PhysX error: SynchronizeStreams cuStreamWaitEvent failed with error 700` |

plus, from the first failing frame:

```
CUDA error 1 ... invalid argument (cudaMemcpyAsync)   omni/sensors/cuda/CudaHelperMem.h:324
[Error] [carb.cudainterop.plugin] Failed to signal external semaphore in CUDA.
[Error] [omni.rtx] Signal external semaphore failed in CUDA submission in command list
                   "Render graph command list (Render queue 0, device 0, frame submission index 0)"
```

**No condition ever completed.** The JSONL holds only the `config` record —
which is exactly what the incremental-write guardrail was for; a
write-at-exit design would have left nothing at all.

### This substantially vindicates the interop hypothesis

Not `vkGetMemoryFdKHR` — that string still never appears. But this is the same
**CUDA↔Vulkan interop** family, failing on the *external semaphore* signal and
on host-memory allocation rather than on memory-FD export. And
`multiTickRateRender() returned null` is **literally one of the five lines in
README §9.3's signature**, alongside
`carb.scenerenderer-rtx.plugin ... Render Graph ... failed`.

So §9.3's "was streaming, then went black" and this crash are plausibly the
same underlying defect, reached by two different routes. §9.3 says it never
recovers and only a container restart clears it — consistent with the
`Semaphore wait timed out after 60 seconds` and the PhysX CUDA-context loss
here.

### The critical contrast

| Run | Motion BVH | Sensors | Result |
|---|---|---|---|
| `_diag_exec.py` launcher bisect | **off** | camera + lidar | **works**: 32,700 px / 460,800 pts, zero errors |
| `radar_penetration_exec.py` controls | **on** | camera + lidar + radar | renderer collapses, 122 errors |

Motion BVH (or radar, or their combination) is what tips it over. The
lidar-only path is healthy. **Which of the three is untested** — resolved
immediately below.

---

## The isolation experiment — run 2026-08-12, and it reverses the conclusion

`SPIKE_SENSORS=lidar|radar|both` was added so each sensor could be created
alone. A deselected sensor is not created at all, so its render product never
exists — merely skipping its readback would have measured nothing.

| # | Sensors | GPU | Motion BVH | `[Error]` lines | Outcome |
|---|---|---|---|---|---|
| 1 | lidar only | 3 (uncontended at the time) | **on** | **0** | **6,786,180 pts / 30 frames.** Healthy. |
| 2 | radar only | 3 (**~18.8 GB held by another user**) | on | **34,499** | `ERROR_OUT_OF_DEVICE_MEMORY`, `RtxSensor openTrace` fails. **Confounded — discarded.** |
| 2b | radar only | 2 (free) | on | **0** | No crash. Radar returns 30–72 detections per condition. |

Three conclusions, in order of confidence:

1. **Motion BVH alone is not the trigger.** Run 1 settles it: BVH on, zero
   errors, millions of points. The previous session's hypothesis that BVH is
   fatal here is **wrong**.
2. **The crash was GPU contention.** Run 2 vs 2b is the same binary, same
   scene, same settings, differing only in which card it landed on. 24 GB minus
   a neighbour's 18.8 GB is not enough for Isaac, and the failure it produces
   (`Out of GPU memory allocating resource 'MemoryManager-allocate'`) is
   indistinguishable from a renderer defect if you do not look at
   `nvidia-smi`. By extension the 122-error CUDA-700 crash from 2026-08-10 is
   most plausibly the same thing — not proven, since occupancy that day was
   never recorded, which is itself the lesson.
3. **Radar runs but does not measure.** This is the live blocker, and it is the
   *silent* failure class this project keeps meeting, not a crash.

### What run 2b actually returned

```
    radar extents  x[-0.28,17.98] y[0.00,10.24] z[5.50,6.69]
[CONTROL no_occluder (moving)    ] radar      0 @tgt /      72 tot
    radar extents  x[-0.28,-0.28] y[0.00,0.00] z[6.16,6.16]
[CONTROL no_occluder (static)    ] radar      0 @tgt /      30 tot
    radar extents  x[-0.15,-0.15] y[0.00,0.00] z[3.90,3.90]
[CONTROL no_target               ] radar      0 @tgt /      30 tot
    radar extents  x[-0.28,-0.28] y[0.00,0.00] z[6.16,6.16]
[CONTROL empty                   ] radar      0 @tgt /      30 tot
```

Read it carefully — every line is evidence:

- **`empty` returns exactly as much as a clear-line-of-sight target**: 30
  detections, i.e. **one per frame**, at a *constant* coordinate. A sensor
  reporting the same single point in an empty scene as with a 2 m cube in front
  of it is not detecting the cube.
- **The static conditions return one identical point per frame** with zero
  spread on all three axes. That is a fixed value, not a measurement.
- **Only the moving condition varies** (72 detections, wide extents) — the
  radar responds to *motion*, consistent with an FMCW/Doppler model, but the
  positions it reports (`z` between 5.5 and 6.7 m, when sensor and target are
  both at `z = 1 m` and the target is 2 m tall) do not correspond to the scene.
- The classification box is therefore not the bug: **no plausible box would
  contain these points**, and widening one until it did would be exactly the
  "find a framing that sounds like success" the task warned against.

**The field usage is not the bug either.** ~~The shipped example
`inspect_radar_gmo.py` reads detections as `gmo.x[i]`, `gmo.y[i]`, `gmo.z[i]` —
identical to this spike.~~ **WRONG — 2026-08-14. The field usage *is* the bug.**
Reading the same field *names* is not reading them with the same *meaning*:
`x`/`y`/`z` are azimuth°, elevation° and range-in-metres unless `coordsType` says
otherwise, and it defaults to `SPHERICAL`. See the top of this file. The second
half of the sentence — "these fields carry something other than Cartesian metres
for radar" — was the correct guess, and it is true of lidar too.

### Lidar has the same localisation problem — same cause, not a separate one

Run 1 returned 6.79 M points and **0 inside the target box**, with 76,3xx points
at `x ∈ (3,5)` in *every* condition including the one where the occluder is
1,000 m away.

**Resolved 2026-08-14.** Not "the cloud is in the wrong frame": the box is in
the wrong *coordinate system*. `|z| < 2.5` is a 2.5-metre **range** gate, and
`x ∈ (3,5)` is a 2-degree **azimuth** wedge — which is why its count is constant
to 0.04% while the total cloud size swings 35%. One bug, both sensors. Full
derivation at the top of this file.

`extents_xyz` did its job exactly as intended: the extents were what made the
spherical reading legible. Recording them was the right call.

---

## Orchestrator status — answering the question properly

**I had only ever sampled it once, and not even "after the fact".** The single
`Status.STOPPED` reading came from a state dump that ran *before* the frame
loop. So the original evidence was the weakest of the three possibilities:
"STOPPED before any stepping", not "STOPPED at the end", and certainly not
"never left STOPPED".

Both diagnostics now sample **per frame** and report a histogram.

| Path | Per-frame orchestrator status | Capture |
|---|---|---|
| `runheadless.sh` + `--exec` | `{'Status.STARTED': 60}` | **32,700 px / 460,800 pts** |
| `SimulationApp`, as-is | `{'Status.STOPPED': 60}` | empty |
| `SimulationApp` + `rep.orchestrator.run()` | `{'Status.STARTED': 60}` | **still empty** |

**This kills the orchestrator lead as a root cause.** Starting the orchestrator
in the standalone path is easy, it reports `STARTED` on every frame, and
nothing fills. So `STOPPED` was a correlate of the broken launcher, not the
mechanism — the two paths differ in something else that `run()` does not touch.

Worth noting `run()` had never actually been executed before this: in the first
drive diagnostic it was strategy E, and strategy C (`orchestrator.step()`) hung
before reaching it. `step()` and `run()` are different calls, and only `step()`
hangs.

---

## Findings that stand independently

### Multi-GPU + IOMMU is a ~30× performance trap (still true, still a config error)

`SimulationApp` defaults `multi_gpu=True`. This host has IOMMU on, and Isaac
Sim's own probe measures the cost:

```
Detected IOMMU is enabled. Running CUDA peer-to-peer bandwidth ...
P2P Writes:  GPU0->GPU0  830 GB/s      GPU0->GPU1  11.2 GB/s
```

Measured on the **shipped** example, unmodified:

| Config | Per frame |
|---|---|
| `multi_gpu=True`, 1280×720, RealTimePathTracing, 64 spp | **~150 s** |
| `multi_gpu=False`, `active_gpu=0`, same resolution | ~5 s |
| `multi_gpu=False`, 256×256, 1 spp, denoiser off | **0.05 s** |

NVIDIA documents IOMMU-enabled P2P as unsupported on bare metal, so this is a
config error, not merely a tuning note. `docker-compose.yml` is currently
`device_ids: ["3"]`; restore `["0", "3"]` only once capture is proven working
end-to-end.

RTX sensors render into their own render products, so main-viewport resolution
and sample count are pure overhead for a headless sensor run.

**Minor-0 mapping re-derived 2026-08-10 (not assumed):**

```
/proc/driver/nvidia/gpus/0000:c1:00.0/information -> Device Minor: 0
nvidia-smi index 3                                -> 00000000:C1:00.0
```

`device_ids: ["3"]` is therefore the minor-0 card and NVENC survives
(`/dev/nvidia0` confirmed present inside the container). Re-derive after any
driver or hardware change.

**Incidental:** this host has **four** RTX 3090s (indices 0–3), not the three
CLAUDE.md refers to. Worth fixing there.

### Which config the failing capture tests ran under

Reconciling "60 `update()` calls at 150 s/frame would be 2.5 hours" — no such
run happened. `multi_gpu` **was** varied for the capture test, not only for
perf; the numbers come from different runs.

| Run | `multi_gpu` | Render config | Frames | Per frame | Capture |
|---|---|---|---|---|---|
| Stock `inspect_radar_gmo.py --test` | True | 1280×720, RTPT, 64 spp | **5** (test mode) | ~150 s | no GMO |
| `radar_penetration.py` smoke/smoke2 | True | 1280×720, RTPT, 64 spp | 60/cond | ~12 s | 0 points |
| `_diag_*` capture tests | False, `active_gpu=0` | 256×256, 1 spp | 60–90 | 0.02–0.05 s | 0 px / 0 GMO |
| `_diag_render_path.py`, single GPU | False, one device | 256×256, 1 spp | 40–60 | ~0.05 s | 0 px / 0 GMO |

The 150 s/frame figure comes only from the 5-frame stock run.

### The task's file path is stale

`tasks/SERVER.md` S4 cites
`standalone_examples/api/isaacsim.sensors.rtx/create_radar_basic.py`. In 6.0
that directory is `isaacsim.sensors.**experimental**.rtx`; the 4.x/5.x path does
not exist.

### Motion BVH is a SimulationApp flag in 6.0, not a carb setting

CLAUDE.md says to enable it via carb settings. In 6.0 the supported route is
`SimulationApp({"enable_motion_bvh": True})`, which sets those settings during
renderer init; setting them afterwards is too late. `Radar._create_prim` raises
outright if `/renderer/raytracingMotion/enabled` is false, so this fails loudly.

Under `runheadless.sh` there is no such config dict — pass
`--/renderer/raytracingMotion/enabled=true` on the command line instead.

### Non-visual materials: the API, and what the CSVs are now

`omni:simready:nonvisual:*` is written by
`isaacsim.core.experimental.materials.NonVisualMaterial`:

```python
mat = NonVisualMaterial(path, bases="steel", coatings="paint", attributes="none")
cube.apply_visual_materials(mat)   # yes, "visual" — that is the binding call
mat.set_bases("cardboard")          # runtime swap, writes the USD attribute
```

The attribute prefix comes from the carb setting
`/rtx/materialDb/nonVisualMaterialSemantics/prefix`, not a hardcoded string.

**CLAUDE.md's phrasing is slightly off.** The 4.5-era CSV *material mapping* is
gone, but three CSVs still ship as the authoritative name→index *specification*
that `NonVisualMaterial` encodes into the USD attributes, at
`exts/isaacsim.core.experimental.materials/data/specifications/`.

- **48 bases** — `none`, `aluminum`, `steel`, `oxidized_steel`, `iron`,
  `oxidized_iron`, `silver`, `brass`, `bronze`, `oxidized_Bronze_Patina`, `tin`,
  `plastic`, `fiberglass`, `carbon_fiber`, `vinyl`, `plexiglass`, `pvc`,
  `nylon`, `polyester`, `clear_glass`, `frosted_glass`, `one_way_mirror`,
  `mirror`, `ceramic_glass`, `asphalt`, `concrete`, `leaf_grass`,
  `dead_leaf_grass`, `rubber`, `wood`, `bark`, `cardboard`, `paper`, `fabric`,
  `skin`, `fur_hair`, `leather`, `marble`, `brick`, `stone`, `gravel`, `dirt`,
  `mud`, `water`, `salt_water`, `snow`, `ice`, `calibration_lambertion`
- **coatings** — `none`, `paint`, `clearcoat`, `paint_clearcoat`
- **attributes** — `none`, `emissive`, `retroreflective`, `single_sided`,
  `visually_transparent`

`skin` exists, which is the honest base material for the avatar in S6.

Viewport check, once this is demoable:
*RTX – Real-Time 2.0 → Debug View → Non-Visual Material ID*.

---

## What remains untested

- **The actual config diff between the two launchers.** This is now the whole
  problem, and it is a bounded one: two known states, one working. The
  highest-value next step is a mechanical diff of what each launcher passes and
  loads — `SimulationApp`'s constructed arg list (it prints it at startup,
  including `--portable`) versus what `runheadless.sh` →
  `isaac-sim.streaming.sh` passes, plus the enabled-extension sets and the
  `/rtx/*`, `/app/*`, `/omni/replicator/*` settings dumped from both. The
  orchestrator result above says the difference is *not* orchestrator state, so
  it is somewhere in that diff.
- ~~Whether the spike can simply be run in exec mode~~ — **done.** It can, it
  does, and exec mode is now the project's execution model for sensor scripts
  (recorded in CLAUDE.md). It removed the launcher as a blocker without needing
  the root cause.
- **A `verbose` Kit log.** The hypothesis-4 refutation is verified at `info`.
  A failure emptying every annotator would not normally log below `info`, so
  this is "not present at `info`", not "provably absent".
- **CUDA vs Vulkan device UUID comparison.** `vulkaninfo` is not in the image
  and rule 7 forbids installing it. Single-GPU makes it moot in practice, but
  that is not the same as having checked.
- **The radar question itself.** Untouched. No penetration data exists yet.

---

## What is ready to run

`radar_penetration_exec.py` runs end to end under `runheadless.sh --exec`.
**It is not ready to run as written** — `_sample()` classifies spherical
coordinates as if they were Cartesian metres, so every `target_points` is zero
by construction (see the top of this file). Fix the two masks and the `VALID`
check first; that is a ten-line change. The measurement *design* below is
unaffected and is the part worth keeping.

It is written so a broken rig reports itself rather than quietly emitting a
number:

- **Controls bracket the measurement.** `no_occluder_moving` establishes what
  100% visibility looks like; `no_target` measures the artifact floor (ground
  bounce, multipath, sidelobe) and is subtracted from every reading; `empty` is
  the noise floor. A zero baseline prints `TEST INVALID` rather than a table of
  zeros that reads like "penetrates nothing" — which is exactly the trap this
  whole session fell into at the stdout level.
- **`no_occluder_static` is its own control**, because an FMCW/Doppler model may
  suppress zero-velocity returns entirely. If radar sees the moving target and
  not the static one, "no penetration" is meaningless — and that matters for
  the demo anyway, since static shelves would then be invisible to radar
  regardless of material.
- **The target oscillates radially** during measurement, giving radar its best
  shot at a "yes" and matching the demo's one-moving-entity premise.
- **A detection counts only inside a box around the target**, not merely "past
  the wall" — otherwise ground returns reaching *around* the wall's edge score
  as penetration.
- **Material swaps are validated, not assumed.** Mean lidar intensity off the
  wall face is recorded per material; if it does not move between `steel` and
  `fabric`, the swap never reached the renderer and the run prints `INVALID`.
- One launch, all 57 conditions, material swapped at runtime.

`--controls-only` validates the rig in seconds before committing to the sweep.

### Files

| File | Purpose |
|---|---|
| `radar_penetration_exec.py` | **The live spike.** Exec mode. Runs clean. `SPIKE_SENSORS=both\|lidar\|radar`, `extents_xyz` per condition. **`_sample()` has the spherical/Cartesian bug — fix before the next run.** |
| `radar_penetration.py` | Original `SimulationApp` version. Superseded — kept only for the measurement design and its comments. Cannot capture on this host. |
| `_diag_render_path.py` | Throwaway. Reproduces the `SimulationApp` failure; per-frame orchestrator histogram; `--start-orchestrator`, `--force-async`. |
| `_diag_exec.py` | Throwaway. The launcher bisect that landed. **Reference pattern for exec mode** (cited from CLAUDE.md). |
| `logs/exec_lidar_bvh_clean.log` | Run 1. Lidar + Motion BVH, 0 errors, 6.79 M points. The refutation of "BVH is fatal". |
| `logs/exec_radar_bvh_gpu2_no_detections.log` | Run 2b. Radar + BVH on a free GPU: 0 errors, no usable detections. **The S4 result.** |
| `logs/exec_radar_gpu_contention_oom.log` | Run 2. Same binary on a contended GPU: 34,499 OOM errors. Kept as the exhibit for checking `nvidia-smi` first. |
| `logs/isolation_results.jsonl` | Machine-readable results from runs 1 and 2b. |
| `logs/exec_controls_cuda_crash.log` | The 2026-08-10 122-error crash. Probably the same contention; occupancy was not recorded. |

---

## IF S4 IS EVER REOPENED

**Step 1 below is done — 2026-08-14, by reading.** The fields are spherical and
sensor-local, the masks were wrong, and both sensors are affected. See the top
of this file. What remains:

1. ~~What frame and units the buffer is in, and whether radar and lidar share a
   convention.~~ **Answered.** Azimuth°, elevation°, range m; `coordsType`
   defaults to `SPHERICAL`, `frameOfReference` to `SENSOR`; both sensors share
   it. Prefer `IsaacExtractRTXSensorPointCloud`, which converts and hands back
   the sensor→world matrix.
2. **Fix `_sample()`** — rewrite both masks in spherical coordinates (target
   subtends ~±7° azimuth at 8 m, so `|az| < 10°`, `|el| < 10°`,
   `6 < range < 10.5`), and mask on `flags & VALID`.
3. The WpmDmat radar model's configuration surface — **now the live question**,
   sharpened: the radar emits exactly **one return per frame**, and it is the
   strongest one, so the wall wins and the target never appears. Read
   `OmniSensorGenericRadarWpmDmatAPI` for returns-per-beam, detection
   threshold, RCS gating and minimum range. Also read `rv_ms`, which
   `aux_output_level="BASIC"` is already populating and the spike discards — it
   settles the Doppler-gating question directly.
4. Only then re-run. `SPIKE_SENSORS` and `extents_xyz` are already in place, and
   `--controls-only` validates the rig in one condition.

Do **not** widen the target box to make points fall inside it. The controls
exist precisely to stop that.

### Practical notes for any future run

- **Check `nvidia-smi` before a GUI or streaming session too, not only before a
  timed run.** The existing rule in CLAUDE.md is written around long headless
  runs, and that is too narrow. On the evening of **2026-08-13** all four GPUs
  were held by another user at **17–19 GB each with load average 11**, and an
  interactive session was simply unusable — not crashed, not erroring, just
  unresponsive enough to waste the session. An interactive session is the *most*
  contention-sensitive thing on this host, because the symptom is latency rather
  than a log line, and there is nothing to grep afterwards. One command, before
  connecting:
  ```
  nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv
  uptime
  ```
  If the cards are full, the session is not worth starting. Two GPUs are
  nominally free for inference (see CLAUDE.md), but "free" is a scheduling
  convention on a shared machine, not a guarantee — verify, do not assume.
- **Startup was ~1,000 s** with Motion BVH on (vs ~270 s without) — a one-time
  shader-permutation compile. The cache volume should now be warm, so expect
  faster; but budget the container `timeout` at **2,400 s minimum**. The
  controls run died at `EXIT=124`, my own 1,500 s timeout, not a hang.
- kit at ~1,000% CPU with a growing log is *compiling*, not hung. Check
  `nvidia-smi` + log mtime before killing anything.
- Results land in the logs **volume**
  (`/isaac-sim/.nvidia-omniverse/logs/`), not `/workspace` — the bind mount is
  uid 1004 and the container is uid 1234, so writes there fail silently.
- Crash log preserved at `sim/spikes/logs/exec_controls_cuda_crash.log`.

### Landed 2026-08-12

- **CLAUDE.md** now carries the exec-mode section (execution model, reference
  pattern, hard-rule-3 exemption and its replacement constraint), the corrected
  four-GPU picture with the minor-0 requirement, the corrected Motion BVH
  guidance, the corrected non-visual-material text with the `NonVisualMaterial`
  API and `skin` for S6, and the shared-machine GPU-occupancy rule.
- **`tasks/SERVER.md`** — S1 gate corrected to four GPUs, S4's stale example
  path and carb-settings instruction fixed, S5's dead fallback asset replaced,
  S10 pointed at this verdict.
- **`docker/docker-compose.yml`** — `device_ids: ["3"]` decided and reasoned,
  including the honest note that the IOMMU penalty is unverified under
  `runheadless.sh`.

### Landed 2026-08-14

- **The GMO coordinate bug found and documented** (top of this file). Reading
  only, no run. It is one bug, it explains both sensors, and it invalidates
  every `target_points` and `wall_points` number this spike has ever produced.
- The Layer-1 consequence recorded for S8/S10/S11: three conversion steps stand
  between `generic-model-output` and the `points  (N, 3) float  metres` that
  `core/observation.py` promises.

### Still not done

- **The radar question itself. No penetration data exists.** Not measurable from
  the runs made so far — but the reason is now a known, fixable defect in the
  measurement rig plus one bounded question about the radar profile, not an
  unexplained sensor.
- ~~The GMO frame/units question above — the one thing that would reopen it.~~
  **Answered 2026-08-14.** It reopens S4 as a fixable task.
- **Why the radar emits one return per frame.** The live question. Read
  `OmniSensorGenericRadarWpmDmatAPI` and `rv_ms` before running anything.
- **Fixing `_sample()`'s masks and re-running.** Required to close S4. Not done
  here: this pass was reading-only by instruction.
- Whether `attach_writer("draw-point-cloud")` (S8) really does the spherical
  conversion internally. Believed yes, not verified.
- The launcher-diff root cause (deliberately never opened).
- A `verbose` Kit log.
- Whether the 2026-08-10 CUDA-700 crash was contention. Most likely, not
  proven; occupancy that day was not recorded.

---

## On the S4 fallback

`tasks/SERVER.md` offers a fallback if radar penetrates nothing: *"radar sees a
return where lidar sees only an occlusion shadow."* That fallback still requires
radar to return something, so it is blocked on exactly the same thing as the
headline.

The physics caution in the task remains sound: real 77 GHz radar does not
penetrate steel, so warehouse shelving is close to the worst possible occluder.
`cardboard`, `plastic`, `fabric` and `plexiglass` are in the base list and are
the honest candidates to try first.
