# MacBook task queue

Hand these to Claude Code on the MacBook. **These run in parallel with the
server track** — nothing here needs a GPU or the simulator, which is the whole
point of the layer discipline.

`CLAUDE.md` at the repo root is loaded automatically.

---

## What the MacBook can and cannot do

**Cannot:** run Isaac Sim. Not under Docker, not under emulation. It needs an
NVIDIA RTX GPU with RT cores, Vulkan, and NVENC, and the image is Linux-only.
The 6.x container is multi-arch, but that means Linux ARM (DGX Spark, Jetson) —
not macOS. No Dockerfile changes this.

**Can:** everything in `core/` and `tests/`, the entire registry, the whole
observation contract, and all research code in `core/memory/`. Because `core/`
never imports simulator APIs, this is a real development machine and not just a
screen.

**If Claude Code here ever proposes adding `omni`, `pxr`, or `isaacsim` to
`requirements-dev.txt`, that is the signal that simulator code has leaked into
`core/`.** Fix the leak, not the requirements file.

---

## M0 — Scaffold validation **[CC]**

**Depends on:** nothing. Do this first, before the server finishes provisioning
— five minutes of CPU work catches repo problems before an hour-long GPU image
pull.

> Run `make dev-build`, then `make verify`. All 14 tests and the layer-boundary
> check should pass. **Everything runs inside the dev container — do not install
> any Python package on this host, and do not run pytest or pip directly.** If
> something is missing, add it to `docker/requirements-dev.txt` and rebuild.
>
> Then verify the boundary check actually works: create a file `core/_leak.py`
> containing `import omni`, confirm `make check` fails with a non-zero exit
> code, then delete it. Report anything that did not behave as described.

**Gate:** tests pass, and the guard demonstrably fails on a real violation. A
guard that never fails is not a guard.

---

## M1 — Client verification **[YOU]**, then **[CC]**

**Depends on:** server S3 streaming.

This resolves the plan's open question about whether the native client works on
Apple Silicon. Spend ten minutes on it on day one rather than assuming.

**[YOU]:** download the Isaac Sim WebRTC Streaming Client DMG from the Isaac Sim
Latest Release page, drag it to Applications, enter the server's Tailscale IP,
click Connect. Client platform support is documented as independent of host
platform support, so a macOS build does exist.

If it fails, fall back to the browser viewer on port 8210 in a Chromium-based
browser.

**[CC]:**

> Record in `README.md` §3.2 which client actually worked, its version, and
> whether it is arm64-native or running under Rosetta (`file` on the binary, or
> Activity Monitor's Kind column). This closes an open decision — write down the
> answer so nobody re-litigates it in three months.

**Gate:** you can see and navigate the stream from the MacBook. One client at a
time — close one session before opening another.

---

## M2 — Registry hardening **[CC]**

**Depends on:** M0. Runs while the server is still on S1–S5.

> The registry currently validates unique ids, unique prim paths, and
> parent-child consistency. Add validation for the failure modes we actually
> expect, each with a test:
>
> - every `fixed` sensor has a `parent` station Xform
> - a station claiming to be multi-modal has all three of camera, lidar, radar
> - no two sensors at the same prim path (already covered — verify it stays)
> - `resolution` is present for camera modalities and absent for lidar/radar
> - `config` (profile name) is present for lidar and radar
> - annotator names are drawn from a known set, so a typo fails loudly at load
>   rather than producing an empty annotator at runtime
>
> Keep every test runnable with no GPU.

**Gate:** `make test` passes with meaningfully more coverage; a deliberately
malformed `sensors.yaml` fails at load with a message naming the bad entry.

---

## M3 — Mock observation source **[CC]**

**Depends on:** M2. **This is the highest-value MacBook task.**

> Write `core/mock_source.py`: a fake sensor source that reads
> `config/sensors.yaml` and emits synthetic `Observation` objects — plausible
> shapes, a moving avatar pose over time, point counts that vary as the avatar
> approaches. No simulator, no GPU.
>
> This lets the entire Layer 3–4 stack be developed and tested before the
> simulator side exists, and it becomes the permanent test fixture for research
> code. Add tests that consume it exactly as real code will.
>
> Design constraint: the server will later write
> `sim/observation_adapter.py` producing the same `Observation` type. Both must
> satisfy the same test suite. Write those tests against the *contract*, not
> against the mock's internals, so they can be pointed at either source.

**Gate:** a test suite that passes against the mock and could be run unchanged
against the live simulator. This is the handoff contract for server task S11.

---

## M4 — Habitat adapter shape **[CC]**

**Depends on:** M3. Optional but cheap, and it is the thing that gets expensive
to retrofit.

> Sketch `core/adapters/habitat.py` — do not install Habitat, do not implement
> it. Write the interface and a docstring establishing exactly what a Habitat
> observation dict would have to be mapped from to produce our `Observation`
> type. The purpose is to prove the contract is genuinely simulator-agnostic
> while that is still free to fix.
>
> If you find something in `core/observation.py` that could not be satisfied by
> Habitat — an Isaac-specific assumption that leaked into the contract — say so
> clearly. That finding is more valuable than the file.

**Gate:** either a clean interface sketch, or a specific named leak in the
observation contract. Both outcomes are useful; the second is more so.

---

## M5 — Memory module skeleton **[CC]**

**Depends on:** M3.

This is where the actual research begins, and it can begin before the simulator
works at all.

> In `core/memory/`, sketch the closed loop the project exists to test:
>
> ```
> persistent memory → prediction → observation → residual → memory revision
> ```
>
> Every existing persistent-memory system in the literature is write-only from
> perception: perception writes, planning reads, nothing writes back. The
> contribution here is the residual path. Prediction residual equals model noise
> in a static scene and **change detection** in a dynamic one.
>
> Define the interfaces only — `predict(pose) -> expected_observation`,
> `residual(expected, actual)`, `revise(residual)`. Implement nothing clever
> yet. Write tests using `core/mock_source.py` that assert the loop closes:
> feed a consistent world and confirm the residual stays near zero; move an
> object and confirm the residual spikes.
>
> Also preserve the state/experience distinction: `MountType.FIXED` readings are
> allocentric state, `MountType.AVATAR` readings are embodied experience.
> Fixed cameras never move, so they never experience an action, a consequence,
> a collision, or a traversal cost. Anything fusing the two must know which it
> holds.

**Gate:** the residual test passes against a synthetic change. That is the
research claim, demonstrated in miniature, on a laptop, with no simulator.

---

## Ordering across the two machines

```
MacBook:  M0 ──── M2 ── M3 ──── M4
                          └───── M5
             │
             │ (M1 needs server S3)
             ▼
Server:   S0 ── S1 ── S2 ── S3 ── S4 ── S5 ── S6 ── S7 ── S8 ── S9 ── S10
                                                                       │
                                       M3's contract ─────────────────► S11 ── S12
```

M0 first, always — it is five GPU-free minutes that de-risk the long image pull.
M3 must land before S11, since it defines the contract S11 implements. Everything
else on the MacBook track runs genuinely in parallel.