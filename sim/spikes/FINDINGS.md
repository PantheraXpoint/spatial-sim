# Spike and findings log

What the spikes measured and what each one turned out to have got wrong:
sensor behaviour, environment facts, and performance sessions. **Newest
first** — a new dated section goes directly under this heading, not at the
end of the file.

## THE WALK CYCLE — 2026-08-26, and the shipped clips are on a different skeleton

Conditions: idle host, GPU index 3, exec mode under `runheadless.sh`,
`observatory_avatar.usd`. Three spikes: `sim/spikes/_diag_walk_clip.py` reads
assets and renders nothing; `sim/spikes/_diag_walk_render.py` renders the
avatar's own third-person camera and prices the result — **and got the price
wrong**; `sim/spikes/_diag_walk_timesample.py` re-prices it against its own
controls and retracts that number. Outputs: `logs/walk_clip.json`,
`logs/walk_render.json`, `logs/walk_timesample.json{,l}`, `logs/walk*.png`.

### First, two premises that were wrong, and the pictures that settle them

**"The avatar is an invisible capsule, so in third person there is no body."**
No. `body_mesh` is indeed invisible, but it is not the body: `/Root/Avatar/character/rig`
references the warehouse's own rigged Worker, renders, carries `person` on
every mesh, and follows the capsule every frame. `logs/walk_A_shipped_idle.png`
is the third-person camera before any change in this entry — a man in blue
overalls and a cap, seen from behind, standing.

**"The skeleton falls back to a T-pose."** That is what `sim/avatar.py`'s own
docstring says, and it is not what happens. The Worker's 582-sample idle clip
is bound and running, so he stands like a person with his arms down. The
T-pose is what you would get if the clip were unbound, and nothing unbinds it.

So S6's second gate was closer to passing than the file believed. What was
actually missing is narrower and real: **his legs did not move.** He slid.

### The finding: no shipped clip fits this skeleton

The 2026-08-17 scoping note concluded the walk was "a reference swap plus a
blend, NOT retargeting", from an inference it flagged as untested. The test
was possible after all — a `SkelAnimation` carries its own `joints` array,
which is how UsdSkel maps a clip to a skeleton, by NAME — and the inference
was false:

```
  the avatar's skeleton     101 joints   RL_BoneRoot/Hip/Pelvis/L_Thigh/...
  every Isaac/People clip    81 joints   Root/Pelvis/R_UpLeg/R_LoLeg/...
  joints in common                    0
```

Zero. Not a different order — no joint name in `stand_walk_loop_in_place` or
any of its siblings exists on this skeleton. The earlier note compared
People's CHARACTERS to the Worker, found 101 matching joints, and assumed the
clips beside them matched too. Re-measured here: `male_adult_construction_01`
is `RL_BoneRoot`, 101 joints — so People's clips do not fit People's own
characters either, without a retarget step that `omni.anim.people` would
normally provide and which is **absent from this image**.

Nor does a walk ship anywhere else within reach. Everything was listed:

| library | contents |
|---|---|
| `Reallusion/Worker/Motions` | Kitchen_CleanTable, Market_Sales_Assisting, Market_Sales_SortOut, StandingDiscussion, TrafficGuard, Worker_Idle_Pose |
| `Reallusion/Debra`, `Orc` | idles, a superpower fist, a sword shout |
| `Isaac/People/Animations` | the walks — on the 81-joint rig |
| `full_warehouse_worker_and_anim_cameras.usd` | the same 101-joint rig, carrying the same idle: its root travels **0.0042** over the whole clip, i.e. sway |

That last row answers the obvious suggestion directly: the sample stage's
"animated worker" is animated, and what it is animated *doing* is standing
still.

**`omni.anim.retarget.core` and `omni.anim.graph.core` ARE enabled here.**
Retargeting People's motion-captured walk onto this skeleton is therefore the
principled route to a better cycle, and it is the recommended next step. It
was not taken: it is an authoring workflow, its Python surface is not the part
that is documented, and its output cannot be checked without a human looking —
which makes it a poor thing to land blind at the end of a session.

### What was built instead

A gait, authored, on the joints the avatar actually has —
`avatar.WalkCycle` — layered **on top of** the shipped idle rather than
replacing it. The clip supplies the whole body's pose, and ten joints
(thighs, calves, feet, upper arms, forearms) get a swing added. Standing
still, the character is exactly the shipped asset; walking, its legs swing.

Three properties worth stating because each was a decision:

- **Driven by distance, not time.** The phase advances by
  `distance / stride`, so the same metre of travel turns the cycle by the same
  amount at 60 fps and at 6 fps. Feet cannot drift out of step with the ground
  because the frame rate moved, and a capture reproduces.
- **The swing axis is derived, not assumed.** Rotating a hip about the wrong
  local axis abducts instead of flexing — the leg swings out sideways — and
  that raises nothing and reads as a broken rig. So the axis is taken from the
  rest pose (`R_rest^-1 · lateral`, in USD's row-vector convention) and the
  SIGN is measured: rotate the joint a test angle, run the chain forward, see
  which way the foot actually went. Confirmed in the render: the stride is in
  the sagittal plane.
- **The facing is written to `character`, never to the capsule.** A PhysX
  character controller has a position and an up direction and no rotation, so
  a yaw written to the capsule never reaches the renderer. The character Xform
  is a prim physics does not own. Before this, the body never turned at all —
  it slid sideways when you walked sideways.

The capsule is untouched: not its transform, not its collision, not its
visibility. It is what the lidar and radar bounce off (failure mode 1) and the
gait only reads where it is. No new renderable geometry exists either — a
SkelAnimation does not render — so the `person` labels are exactly the ones
the asset shipped and segmentation is unchanged.

### Does it reach the renderer? Yes, and it is measured rather than looked at

The risk with posing a skeleton by writing `rotations` on a bound
SkelAnimation every frame is that USD accepts the writes whether or not
anything downstream re-skins. So: hold the world still, advance ONLY the
phase, and count pixels. The avatar does not move and the third-person camera
is mounted on it, so any pixel that changes is a limb.

```
  changed pixels, phase FROZEN      median 0      max 34
  changed pixels, phase ADVANCING   median 2,338  max 4,257
  UsdSkel joints resolving differently between phase 0.0 and 0.5:  28
```

Twenty-eight, not ten, and that is the right number: ten driven joints plus
the eighteen below them in the chain.

### The frame-rate cost — RETRACTED THE SAME DAY. It was the instrument.

**What was published first, kept verbatim:**

> Measured in one process, back to back, 640x480 render product on the
> avatar's third-person camera, at Play, nothing else attached:
>
> ```
>   shipped idle clip bound (the world as it was)   17.37 ms/frame   57.6 fps
>   our animation bound, and NEVER written          33.30 ms/frame   30.0 fps
>   our animation bound and written every frame     33.34 ms/frame   30.0 fps
> ```
>
> **The gait costs +0.04 ms. Binding the animation at all costs +15.9 ms.**
> […] it says where a fix would have to look: at how the animation is
> authored, not at what is written into it. The shipped clip is TIME-SAMPLED
> and ours holds values at the default time code, and that is the only
> structural difference between the two rows that differ by 16 ms. Whether a
> time-sampled animation would keep the fast path while still being driven by
> distance is the obvious next question and was not answered here.
>
> Read it against the rest of the budget before turning it on for a demo […]
> It is one keyword — `install_character_follow(..., animate=False)` — and
> that switch exists because of this number.

That next question was asked — `_diag_walk_timesample.py`, same day, same
host, same stage. **There was no 15.9 ms to recover.** The row that differs is
not the row that binds a different animation; it is the row that reads the
annotator back.

Row A never called `annotator.get_data()`. Rows B and C called it on every
frame, to count changed pixels. Two variables, one table.

### The measurement that replaces it

A 3×2 matrix — three animation variants × readback off/on — nine arms back to
back in one process, 40 frames each, 640x480 on the same camera at Play. The
shipped baseline is measured at both ends of the run in both readback modes,
because absolute frame times on this shared host are not comparable between
runs: this baseline alone has measured 17.61, 26.11, 17.37 and 17.63 ms on
four earlier occasions, and 17.80 then 16.70 twice inside this one. Only a
delta inside a single process means anything.

```
  arm                     ms/frame   get_data   pixel diff   changed px
  shipped_noread_1           17.80         --           --           --
  shipped_read_1             33.30       2.46         7.38          102
  default_static_read        33.33       2.47         7.34            0
  default_write_noread       16.70         --           --           --
  default_write_read         33.32       2.49         7.39        2,873
  sampled_write_noread       16.71         --           --           --
  sampled_write_read         33.57       2.52         7.35        2,464
  shipped_noread_2           16.70         --           --           --
  shipped_read_2             33.34       2.48         7.34          150

  shipped baseline, readback OFF   17.25 ms      (its two arms drift -1.10)
  shipped baseline, readback ON    33.32 ms      (its two arms drift +0.04)
  PRICE OF THE READBACK           +16.07 ms/frame
  cost of the gait, readback OFF   default -0.55   sampled -0.54
  cost of the gait, readback ON    default  0.00   sampled +0.25
  bound-but-never-written          +0.01 ms against its OWN baseline
  time-sampled minus default-time  +0.01 ms (no readback)  +0.25 ms (readback)
```

**Three answers, in order of how much they change.**

1. **The gait is free.** Not "+0.04 ms for the writes and +15.9 for the
   binding" — free, both halves. Our animation bound and written every frame
   costs **−0.55 ms** against the shipped clip doing nothing, which is to say
   nothing at all: the two shipped arms disagree with each other by 1.10 ms.
   The retracted paragraph's mechanism — *"the character is now re-skinned
   every frame where before it was not"* — is refuted by the number, and the
   reason it was never plausible is in `_diag_walk_render.py`'s own docstring:
   the Worker ships a 582-sample idle and Play advances the timeline, so the
   character was already being re-skinned every frame. Binding a different
   animation does not add skinning that was already happening.
2. **Time-sampling changes nothing: +0.01 ms.** It was a good hypothesis about
   a real structural difference — after the sampled arm runs, `rotations`
   carries 81 time samples and no default, exactly like the shipped clip's
   582, and reading two of those samples back gives quaternions that differ by
   0.248, so the poses really are distinct per time code — and the difference
   simply does not cost anything in either direction. `sim/avatar.py` keeps
   authoring at the default time code.

   The `changed px` column is what stops that being a hollow result. A variant
   that got cheaper by quietly not being applied would show its pixels
   collapse; the sampled arm renders **2,464** changed pixels per frame
   against **0** for the bound-but-frozen control and **102–150** for the
   shipped idle alone. It is running, it is reaching the renderer, and it
   costs the same.
3. **Reading the annotator back costs 16.07 ms/frame at 640x480, and timing
   the call does not show you that.** Of the 16.07, only **9.84 ms** is
   attributable inside the two calls that were timed directly — 2.48 ms in
   `get_data()` and 7.36 ms in the numpy pixel difference this spike does with
   the result. The other **6.23 ms** is a residual: it is a real, repeatable
   difference between whole frames that does not appear in either timed
   region. *Why* is not measured here. The obvious candidate is that the
   readback is a synchronisation point and the CPU stops overlapping the next
   frame with the GPU's current one — **that is an inference and it is
   untested**, which is exactly the shape of claim this entry exists to
   retract. What is measured, and is enough: **price a readback by
   differencing whole frames, never by timing the call.**

### What this costs elsewhere, and it is not nothing

The readback is not overhead in capture mode — reading the annotator *is* the
job, and `sim/observation_adapter.py` pays it once per sensor per frame by
design. It is overhead in a **demo**, and it is a trap in any **measurement**:

> **Never price a scene change while an annotator readback is in the loop
> unless every arm reads back.** 16 ms is larger than most things worth
> measuring in this project, and it attaches to whichever arm happens to
> sample pixels. Here it was mistaken for the cost of a skeletal animation,
> published, and used to justify a feature switch.

How it scales was not measured: one annotator, one render product, one
resolution. `sim/observation_adapter.py` calls `get_data()` once per annotator
per sensor, so a capture run with several sensors plausibly pays several of
these — **plausibly**, not measured, and worth an arm of its own before any
capture budget is quoted.

Nothing raises. Both arms are legitimate code doing legitimate work; the table
lines up; the numbers are stable to two decimal places across 40 frames. It is
wrong only in what the two rows have in common, and that is not visible from
the numbers.

### So what is `animate=False` for now

Not frame rate. `install_character_follow(..., animate=False)` stays — as the
control the comparison above is built from, and as the way to get the
character posed exactly as the asset ships it — but it buys **0.0 ms**, and any
demo that turned it off for performance can turn it back on. The real budget
line is unchanged and is elsewhere: Play costs this scene most of its frame
rate (14–20 fps in the GUI, measured as 100% physics).

---

## THE SETTLE CONSTANT IS GONE — 2026-08-26, and a step now waits for the buffer, not the clock

Conditions: idle host, GPU index 3, exec mode under `runheadless.sh`,
`observatory_avatar.usd`, the adapter's own rig (robot platforms referenced in
and pinned, seven sensors live). Driver:
`sim/observation_adapter.py` at `OA_MODE=freshness`. Evidence for the change
this entry is about, and it exists because **`tests/contract.py` cannot be
that evidence** — it checks shape, dtype, units, frame and reactivity, and a
payload six frames behind the pose beside it satisfies every one of them.
Output: `logs/observation_adapter_freshness.jsonl`.

`OA_SETTLE` — "frames between advancing the world and sampling", default 2 —
has been removed. `IsaacObservationSource.step()` now advances the world and
then waits until **every sensor whose buffer can be tracked has published two
new buffers**, using the sensor's own refresh counter.

### The measurement that justifies it

The avatar walks a circle. After each `step(dt)` the lidar's cloud is counted
in the box the avatar is in **now** against the box it was in **one step ago**.
Both behaviours ran in the same session, on the same rig, against the same
walk — the fixed wait that was replaced, then the refresh wait that replaced
it:

```
  replaced: fixed 2-frame settle            new: wait for 2 buffer refreshes
       t    here   there  frame_id              t    here   there  frame_id
     1.2     234    1033       120            1.2    1050       0       156
     2.4      24     234       120            2.4     354       0       168
     3.6       0     203       126            3.6     483       0       180
     4.8       0     488       132            4.8     479       0       192
     6.0       0       0       132            6.0     467       0       204
     7.2       0       0       138            7.2     459       0       216
     8.4       0     458       144            8.4     593       0       228
     9.6       0       0       144            9.6     790       0       240
  returns at the CURRENT position: 0/8     returns at the CURRENT position: 8/8
  frames waited 3..3                       frames waited 7..11
```

**Nought out of eight.** Not "sometimes stale" — under the constant that was
in the file, *no step in the run* returned a cloud of the world that step had
asked for. Every one described where the avatar had been a step earlier, at
full density, on a real body, with a `pose` and a `timestamp` beside it that
were current.

Read the `frame_id` column of the left-hand table, which is new and is half
the point: **120, 120, 126, 132, 132, 138, 144, 144.** Three of those eight
steps were handed a buffer that a previous step had already been handed. Two
readings a caller had every reason to believe were two observations were one
observation returned twice, and nothing in either of them said so.

The right-hand column advances by exactly **12** every step — two refreshes of
six frames — which is the mechanism doing precisely what it claims.

### Why a counter and not a comparison

The obvious implementation is to compare buffers and call it changed when the
contents differ. It cannot work, and the reason is worth stating because it is
not obvious: **in a static scene two consecutive refreshes are identical**, so
content comparison cannot distinguish "has not refreshed yet" from "refreshed
while nothing moved". A source built on it blocks forever the first time the
world happens to be still — which is the state a benchmark is in at every
episode reset.

`sim/spikes/_diag_buffer_clock.py` was written to find something better, and
held the scene deliberately still for 72 frames to do it:

| field | distinct values in 72 static frames | run lengths |
|---|---|---|
| `gmo.frameId` | 13 | exactly 6 |
| `gmo.timestampNs` | 13 | exactly 6, +100,000,000 ns each |
| `gmo.numElements` | 13 | exactly 6 |
| 48 other header fields | 1 | — |
| camera `rgb` / `depth` / `semantic` | 72 | 1 |

So `frameId` and `timestampNs` tick on publication and on nothing else, and
they tick while the world is frozen. That is the signal. It is also free: two
header integers against a digest of 290,000 points.

Three things that entry settles as a side effect:

- **The camera has no cadence.** Its annotator buffers changed on all 72
  frames with nothing moving. Nothing waits on the cameras, and the
  "the camera tracked immediately" half of the avatar-pose finding is now
  measured directly rather than inferred from a centroid.
- **10 Hz × 60 fps = 6.** `timestampNs` steps 0.1 s per refresh and `frameId`
  goes 28, 34, 40. The arithmetic offered as an unverified hypothesis in the
  object-motion entry above is now a measurement.
- **Two fields that look like counters and are not.** `frameStart` and
  `frameEnd` are `FrameAtTime` **objects**, so anything that stringifies them
  sees a fresh memory address every frame and reads as a per-frame tick — the
  probe's own first output made exactly that mistake. `scanComplete` and
  `scanIdx` were 0 on all 72 frames.

### Why two refreshes and not one

A sweep takes time. Advance the world at application frame T and the next
buffer covers an interval that *started* before T, so it can hold the old
world and the new one at once — which is the three-state transition the
object-motion entry measured directly (7 returns at the new position for one
refresh, then 849). The second refresh is the first whose whole interval lies
after T. One is enough only when the advance happens to land on a publication
boundary, and nothing controls the phase.

The cost is bounded and was measured: **7 frames on the first step, 11
thereafter**, against 3 for the constant. That is the honest price of the
guarantee, and it is a price per step rather than a risk per step.

### What the readings say now

Every reading carries `intrinsics["freshness"]` — refreshes required,
refreshes waited, frames spent, whether the wait completed — and every range
reading carries the `frame_id` and `timestamp_ns` of the buffer it came from.
A reading that could **not** be made fresh (`OA_MAX_WAIT` spent, or a sensor
with no counter) says so in the data rather than only in a log. The whole
class of bug this replaces was invisible precisely because the reading said
nothing about itself.

`step()` called on Kit's own main thread still cannot wait — blocking there
would block the loop that produces the refreshes — and now says so in the same
field rather than quietly looking like the off-thread path.

### Constraints and caveats

- **Setting `OA_SETTLE` now prints an obsolescence notice** instead of being
  silently ignored. A caller who set it was asking for a freshness guarantee
  and would otherwise have got a different one without being told.
- **The old behaviour is still reachable**, as `OA_SETTLE_FRAMES`, and exists
  for one reason: so the comparison above can be run rather than argued. It
  defaults to 0.
- **A sensor with no `frameId` is not waited on.** It is reported once,
  loudly, and its readings carry the same `freshness` block saying what did
  not happen. Nothing silently degrades.
- **The cadence is not assumed anywhere in the code.** Six is a fact about
  this rig; the adapter waits for a counter to change and never for a number
  of frames, so a different scan rate, frame rate or render-product count
  needs no change here.
- **The spikes keep their fixed settles on purpose.**
  `sim/verify_avatar_pose.py` and `sim/spikes/move_object_exec.py` profile a
  30-frame window because they are *measuring* the cadence. A spike that
  waited for the answer could not report it.

---

## MOVING A WAREHOUSE OBJECT — 2026-08-26, and the lidar's cloud only changes every sixth frame

Conditions: idle host, GPU index 3 only, no other GPU tenant, exec mode under
`runheadless.sh`, `observatory_avatar.usd`, capture-mode configuration (no
collider mask, no raised `minFrameRate`, render products at the registry's
declared resolution, debug draw attached — the same sensor rig the avatar pose
run used, so the frame counts below are comparable rather than merely similar).
Driver: `sim/spikes/move_object_exec.py`, which displaces one warehouse prop by
writing its group Xform's `xformOp:translate` while the timeline plays, and
reads INFRA_01's camera and lidar back through `sim/observation_adapter.py`.
Settle 30 frames, warm-up complete after 5, 319 frames end to end. Output:
`logs/move_object.jsonl`. **89 checks, 0 failed, 0 unanswerable.** Three
consecutive runs of the finished script agree on every crossover frame below,
to the frame.

Two objects, chosen by the script from the stage's own semantics and then
ranked by what the lidar actually returned off them, not named in any file:

| arm | prop | physics as it runs | at rest |
|---|---|---|---|
| as authored | `/Root/Warehouse/Box_21821` | `UsdPhysics.CollisionAPI`, no rigid body — a STATIC collider | 1,424 returns, 10.9 m, elev −10.2° |
| kinematic | `/Root/Warehouse/Box_33988` | plus `RigidBodyAPI` + `kinematicEnabled`, applied BEFORE Play | 406 returns, 9.3 m, elev −13.3° |

Each ran four waypoints — its own position (a no-op write, which is how the
noise floor is measured rather than assumed), two free positions found by
searching arcs about the station and screening them with a PhysX overlap, and
back to where the asset put it. Six transitions in total.

### The move works, and both modalities register it

Every waypoint landed at **0.000 mm** from what was asked for, read back off
the stage's world bounding box, and both objects returned to their authored
pose exactly after three intervening writes. Nothing overwrote the transform:
unlike the avatar capsule, whose `xformOp:orient` PhysX puts back to identity
every frame, a warehouse prop's translate survives — including on the arm
where PhysX owns the prim as a kinematic body.

Lidar returns in the box around each waypoint, **net of the background that
box holds with the object out of it**, as the object stands at each waypoint
(as-authored arm):

```
  as authored             P0      P1      P2      P3      raw at P0's box
  P0 origin              834       0       0     834      1427
  P1 away                 10     401       0      10       603
  P2 away                  0       0     799       0       593
  P3 back                845       0       0     845      1438
  background             593       0       0     593
```

P0 and P3 are the same position, so they are the same box; the raw column on
the right is what that box reads before the background is taken off, and is
the whole reason the correction below exists. The kinematic arm's matrix has
the identical shape at a quarter of the density: 117 / 209 / 226 / 120 net
against a background of 287.

### Headline: the "5 to 10 frame lidar lag" is a SIX-FRAME REFRESH

The 2026-08-26 avatar run measured the lidar catching up 5, 10 and 9 frames
after a pose write and called it a lag. It is not a lag, or not only one.
Every one of the twelve profiles measured here — six transitions, each 30
frames sampled one per application frame — is **piecewise constant in blocks
of exactly six frames**. Run-lengths of the net return count, per transition:

```
  as authored  P1     0x4   402x6  396x6  402x6  399x6  401x2
  as authored  P2     0x3   750x6  787x6  791x6  796x6  799x3
  as authored  P3     0x2     7x6  849x6  830x6  824x6  845x4
  kinematic    P1     0x6   211x6  214x12 212x6
  kinematic    P2     0x5   224x6  230x6  228x6  225x6  226x1
  kinematic    P3     6x4   114x6  119x6  116x6  115x6  120x2
```

Not one block is any other length. The ragged first and last blocks are the
phase the write landed on and the cut at frame 30; the `214x12` is two
consecutive refreshes that happened to agree. **The RTX lidar's
`generic-model-output` buffer changes once every six application frames**, and
`get_data()` hands back the same buffer in between — so what the earlier run
measured as a variable 5-to-10-frame lag is a fixed cadence sampled at
whatever phase the write happened to land on.

That reading also predicts the spread, and the spread is what was measured.
Crossover, per transition, against the camera at the same station:

```
  arm                       transition   lidar   camera   lag
  as-authored static        P1 away          5        1    +4
  as-authored static        P2 away          4        1    +3
  as-authored static        P3 back          9        1    +8
  kinematic rigid body      P1 away          7        1    +6
  kinematic rigid body      P2 away          6        1    +5
  kinematic rigid body      P3 back          5        1    +4
```

**So yes, the same lag applies to a moved object, and it is the same size**:
4 to 9 frames here against the avatar's 5, 10 and 9. And the camera does not
merely win — its pixel count at the object's new range is *identical on all
thirty frames* (kinematic P1: 3,569 px at frame 1 and at frame 30, unchanged),
so the camera had finished tracking before the first frame this script could
sample. The two modalities disagree about where that object is for between
three and eight frames, every time anything moves.

**The mechanism, and it is no longer a hypothesis.** This entry originally
offered "six is what a 10 Hz rotary scan sampled at 60 application frames per
second would give" as a candidate explanation, flagged as unmeasured. It was
measured the same day, by reading the buffer's own header instead of its
payload (`sim/spikes/_diag_buffer_clock.py`): over 72 frames with the scene
held still, `gmo.timestampNs` stepped **exactly 100,000,000 ns** per refresh —
10 Hz — and `gmo.frameId` went 28, 34, 40, i.e. **six application frames**
apart, which is 60 fps. The arithmetic was right and is now a finding.

That measurement also produced the thing this cadence made necessary: a way
to know which refresh you are holding. `frameId` and `timestampNs` tick on
publication and on nothing else — including while nothing in the scene moves,
which is the case no comparison of the payload can handle. 48 other header
fields never changed at all across those 72 frames. `sim/observation_adapter.py`
now waits on that counter instead of on a settle constant; see **THE SETTLE
CONSTANT IS GONE** above.

### The three-state transition is here too, and six frames is its length as well

`P3` of the as-authored arm: the box at the destination reads **0** for two
frames, then **7** net returns for exactly one refresh, then **849**. Seven
returns is a refresh that caught the sweep part-way across the object — the
same shape as the avatar run's 65-at-the-new-pose-and-907-at-the-old, which
also held for six frames, and the same as this spike's own earlier runs (11
new against 397 old, and 1,723 new against 36 old, both for frames 4 through 9
of the settle). Across the twelve transitions measured while building this,
the dual state appeared in two, and both times it lasted exactly one refresh.

So the transition has three states for a moved object exactly as it does for a
moved avatar, it is not present in every transition, and its duration is one
refresh rather than a number that needs measuring separately.

The detector itself sits close to its own threshold and should be read that
way: the two final runs, which agree on every crossover frame, disagree about
whether the tail of one transition counts — one flagged two frames holding 401
returns at the new position against 10 at the old, the other saw 401 against
nothing. Ten returns is where "a cluster on a body" stops being distinguishable
from a stray ray, and that boundary is a choice, not a measurement.

### Collision follows the move — and the object does NOT have to be kinematic

The question this spike existed to close, and the answer is clean. Probed with
a PhysX `overlap_box` over the object's own footprint at every waypoint, on
every sample:

```
  arm                     P0        P1              P2              P3
  as-authored static    present   present/left    present/left    present/left
  kinematic rigid body  present   present/left    present/left    present/left
```

Identical. **A plain `UsdPhysics.CollisionAPI` collider with no rigid body at
all follows a USD transform write made while the simulation is running, and
vacates the position it left.** Adding `RigidBodyAPI` with `kinematicEnabled`
changed nothing that was measured — not the collision behaviour, not the
sensor latency, not the return counts. On Isaac Sim 6.0.1, for a scripted
placement, kinematic is not required.

Two limits on that, both real:

- **A move here is a PLACEMENT, not a swept motion.** The object is teleported
  and nothing sweeps the space in between, so this says nothing about pushing
  an object through or into something else. It is the same caveat that applies
  to `set_avatar_pose`.
- **Nothing was dropped.** Neither arm is a dynamic body, so this says nothing
  about whether a moved object then falls, settles or topples — which is what
  a scenario system would eventually want to know.

### What it took to make the measurement honest

Four corrections, each of which produced a confident wrong answer first.

1. **A box count counts a VOLUME, not an object.** For an avatar standing in
   an aisle the volume empties when it walks away. A warehouse prop stands in
   a stack: when it leaves, the shelf and the cartons beside it are revealed
   and the box settles at a few hundred returns that were never the object.
   Measured: P0's box read **1,423 with the carton in it and 604 without**, so
   a raw here-versus-there comparison had the object's new position (396)
   losing to the furniture it had left behind (604) — and reported that the
   lidar never caught up on a transition whose trace plainly shows it catching
   up at frame 5. Every count above is net of a background measured with the
   object elsewhere.
2. **A downward raycast is the wrong collision probe in a warehouse.** It asks
   what the topmost collider above a spot is, and in a stack the answer is the
   carton on top: it hit a NEIGHBOUR for all seven visible candidates —
   `Box_21821`'s probe came back holding `Box_21803`, 0.195 m down — and every
   one of them was rejected as unprobeable while its collider sat exactly
   where it belonged. An overlap over the object's own footprint asks the
   question that was meant.
3. **"Which pixels got nearer" is blind to a move along the camera's own sight
   line.** A carton pushed from 11.04 m to 13.18 m on the same bearing gave
   **8,489 pixels farther and 149 nearer** — it re-occupied the pixels it had
   just left, only farther away, so nothing in the frame got nearer. The lidar
   had 400 returns on it at the new position at the same instant. The camera
   signal is now gated on RANGE — a pixel that changed and now reads the
   object's new distance — which covers the sideways and the radial case with
   one definition.
4. **A prop whose box holds 61 returns may hold 61 returns of its
   neighbours.** One did: moving it away left the count at 63. Below a couple
   of hundred there is no way to tell an object from its stack before the
   move, so the selection floor is 150.

### Constraints hit, and they cost most of the runs

- **`sensor_factory` aims every station camera at the AVATAR**, which is right
  for every other script in this repo and wrong for this one: **1,778 of 3,137
  props were outside that frame** and the first run rejected every candidate
  on the stage. The spike now re-aims the station camera at the object it is
  about to move, once per arm, before that arm's first reference frame — never
  while a transition is in flight, so the camera is still fixed in the sense
  the claim needs.
- **Occlusion is the binding constraint on which object can be used, and no
  amount of geometry predicts it.** Of 40 props that passed every geometric
  filter — size, floor-standing, inside the lidar's −15..+10° band, 6 to 25 m
  out — **11 were visible to the lidar.** A barrel fully in band at 10 m
  returned exactly zero points out of a 290,057-point cloud because a rack
  stands in front of it. The run now records the distance to the nearest
  return for every candidate it cannot see, because "occluded" (nearest return
  2.2 m away) and "the counting box is in the wrong place" (nearest return
  0.6 m away) are different problems and the count alone cannot tell them
  apart. Two of forty fell in that second category and are unexplained.
- **Reading semantics with `prim.GetProperties()` cost four minutes** over
  3,137 props, because on a mesh that returns its points, normals, uvs and
  every primvar. Reading the applied-schema list instead is effectively free.
  Safe here only because the 2026-08-25 audit measured **zero** prims on this
  stage carrying semantics attributes without the matching API applied — a
  coverage audit must still scan properties, since the schema-less case is
  what it exists to find.
- **A `--rm` container that has been `docker stop`ped can still hold its
  name.** One run never started: `Conflict. The container name
  "/spatial-sim-move" is already in use`. Nothing about it looked like a
  failed launch from the outside — the log file simply stayed empty. With
  exactly one sim container permitted host-wide this is a real trap; do not
  pass `--name` to `docker compose run`, and check `docker ps -a`.
- Computing world bounding boxes is **not** the expensive part, contrary to
  the fifteen-minute figure quoted for the collider-mask pass: 1,600 of them
  took 0.3 s here. Whatever cost that pass fifteen minutes, it was not the
  bounding boxes on their own.

### What this does not establish

The radar was not involved — `sensor_factory` skips it without the three
Motion BVH kit flags. Nothing here is a scenario system, an episode, or a
schedule: it moves one object at a time, on command, and reports what each
sensor said and when. And the six-frame cadence is measured on this rig — one
rotary lidar, four render products, debug draw attached — not shown to be
independent of any of them.

---

## AVATAR POSE WRITES — 2026-08-26, and the lidar was five to ten frames behind the pose

> **SUPERSEDED, 2026-08-26, on the reading of the numbers — not on the
> numbers.** The measurements below stand and were reproduced. Their
> INTERPRETATION does not: this entry calls the effect a *lag* that *varies
> between 5 and 10 frames*, and it is neither. The RTX lidar's
> `generic-model-output` buffer refreshes **once every six application
> frames** — 10 Hz scan, 60 fps application, both read off the buffer's own
> `frameId` and `timestampNs` while the scene was held still
> (`sim/spikes/_diag_buffer_clock.py`). `get_data()` returns the same buffer
> in between and says nothing about it. So the crossover frames recorded
> here, 5 / 10 / 9, are one fixed cadence sampled at three different phases,
> not a latency that changed three times.
>
> What that changes, concretely:
>
> - **"5 to 10 frames" is not a range to design against.** The quantity is
>   "up to one refresh, plus the refresh that is already in flight". A settle
>   of 11 would be as arbitrary as a settle of 2 and would still be a bet on
>   phase.
> - **The advice at the end of "The measured crossover" — raise the settle to
>   30 — was the wrong fix and has been withdrawn.** No constant is the right
>   constant. `sim/observation_adapter.py` now waits for the sensor's own
>   refresh counter to advance instead, which is exact and needs no tuning;
>   `OA_SETTLE` is gone. `sim/verify_avatar_pose.py` still profiles a fixed
>   30-frame window on purpose — it is measuring the cadence, not consuming
>   it, and a spike that waited for the answer could not report it.
> - **The three-state transition below is explained rather than merely
>   observed.** A refresh published while the sweep is part-way across the
>   change carries both poses; that is why it lasted six frames, and six is
>   the cadence.
>
> Everything the entry says about *consequences* — that a stale cloud is full
> and plausible, that the pose is current while the payload is not, that
> `tests/contract.py` cannot see it — is unchanged and is the reason the
> adapter was changed. See **MOVING A WAREHOUSE OBJECT** above for the
> cadence measurement and **THE SETTLE CONSTANT IS GONE** for what replaced
> the constant.
>
> The original text is left below exactly as written, including the two
> sentences this box contradicts, so that what was believed and what corrected
> it are both readable.

Conditions: idle host, all four 3090s free, no other GPU tenant, exec mode under
`runheadless.sh`, `observatory_avatar.usd`, capture-mode configuration (no
collider mask, no raised `minFrameRate`, render products at the registry's
declared resolution). Driver: `sim/verify_avatar_pose.py`, which writes poses
through the new `avatar.set_avatar_pose()` and reads INFRA_01's camera and
lidar back through `sim/observation_adapter.py`. Four waypoints on an arc
centred on the station — constant distance, so Example_Rotary's −15..+10°
elevation band holds by construction — the fourth returning to the first.
Output: `logs/verify_avatar_pose.jsonl`. Three runs; the two after the fixes
agree to the frame.

**Headline: after a pose write the RTX lidar keeps describing the previous
pose for 5 to 10 frames, while the camera at the same station tracks the write
immediately.** This is failure mode 10 one layer along and it is worse, because
what comes back is not empty.

> *Superseded reading:* the lidar keeps describing the previous pose until its
> next buffer refresh, and refreshes come every six frames. "5 to 10" is where
> in that cycle these three writes happened to land. The camera half is
> unchanged and was later measured directly: its annotator buffers change on
> every single frame, so it has no cadence to be behind.

### The table that started it, at a 4-frame settle

Rows are where the avatar was standing; columns are how many returns fell in
the box around each waypoint. W3 is W0's pose again, so their boxes are the
same box. The station camera's `person` centroid is on the right, taken from
the same tick.

```
                     lidar returns in the box around      camera
avatar standing at     W0     W1     W2     W3        person centroid px
W0 start             1133      0      0   1133             (640, 354)
W1 +arc              1037      0      0   1037             (409, 363)
W2 -arc                 0    840      0      0             (869, 359)
W3 back to start        0      0    725      0             (640, 354)
```

Every row after the first has its returns in the box the avatar **just left**.
The camera's centroid, same station, same tick, same settle, is correct at all
four. Nothing in the cloud says so: ~1,000 returns, right density, right
extent, genuinely on a body — just not the body's current position. **A stale
cloud is full and plausible where an empty one is at least obviously empty.**

### The measured crossover, and why it is measured every run

`sim/verify_avatar_pose.py` does not assume a settle constant. After each pose
write it counts, on every frame of the settle, the returns in the box the
avatar is standing in now against the box it just left, and reports the frame
they cross over. Two consecutive runs, identical numbers. Read
`frame:here/there` as "on this frame of the settle, this many returns in the
box the avatar is in now, this many in the box it left":

```
W0 -> W1   crossover at frame  5     1:0/1038  4:0/1038  5:852/0
W1 -> W2   crossover at frame 10     3:0/882   4:65/907  9:65/907  10:773/0
W2 -> W3   crossover at frame  9     8:0/781   9:1018/0
```

The W1→W2 row is the one to read twice. From frame 4 to frame 9 the cloud
holds **65 returns at the new pose and 907 at the old one simultaneously** —
the rotary sweep caught mid-rotation. So the transition has three states, not
two: old, both, new. A settle chosen anywhere in that window yields a cloud
containing an avatar in two places, which is not a state any consumer models.

The default settle is now **30 frames**, and the check is spelled "the lidar
caught up within 30 frames" with the crossover frame in its detail, so a
regression names the number instead of failing an opaque box count.

> *Superseded, for consumers.* Thirty frames is fine for a SPIKE that is
> measuring the cadence — it has to outlast the thing it is timing, and
> `sim/verify_avatar_pose.py` and `sim/spikes/move_object_exec.py` both keep
> it for that reason. It is the wrong answer for a SOURCE. Any constant is:
> it is a duration standing in for an event, it has to be re-derived whenever
> the scan rate or frame rate changes, and it is either too short or wasteful
> at every step of every trace. `sim/observation_adapter.py` waits for
> `frameId` to advance instead.

### Why this matters beyond the spike

`core/observation.py` hands a consumer one `Observation` per sensor per tick,
each carrying a `pose` and a `timestamp`. **The pose is current and the
`points` payload is up to one refresh — six frames — old**, and there is
nothing in the object that says so. *(Written here as "5–10 frames old"; the
quantity is a refresh, not a frame count. Both halves of the complaint stand:
the payload is older than the pose, and the object does not say so. The second
half has since been fixed — every reading now carries
`intrinsics["freshness"]` and every range reading its buffer's `frame_id`.)* Any benchmark that writes a pose at episode start — which is every
benchmark this project is aimed at, since that is what episode reset *is* —
gets stale geometry in its opening frames from the lidar and correct geometry
from the camera beside it. The two modalities then disagree with each other
about where the avatar is, at exactly the moment a fusion module is least able
to notice, because it has no earlier frame to compare against.

`tests/contract.py` does not catch this and would not: it checks shape, dtype,
units, frame and that readings react to the avatar, and temporal alignment
between a reading's pose and its payload is not among them. The S11 contract
run used `OA_SETTLE=2`, well inside the stale window, and passed. *(`OA_SETTLE`
no longer exists; setting it now prints an obsolescence notice rather than
being quietly ignored. The contract still cannot see freshness and still is
not evidence about it — which is why `OA_MODE=freshness` exists.)*

### `runheadless.sh` enables `omni.physx.cct` itself

Measured from the extension startup log of a run whose command line was
exactly `./runheadless.sh --exec /workspace/sim/verify_avatar_pose.py`, with no
`--enable` anywhere on it:

```
[35.597s] [ext: omni.physx.cct-110.1.13] startup
```

The belief that no shipped experience enables it comes from grepping
`/isaac-sim/apps/*.kit`, which is a statement about `.kit` files and not about
what a launcher script adds on top. It is untested against `python.sh` and it
is now **measured false for `runheadless.sh`** — which is the launcher
everything that reads sensor data runs under. Corrected in place in
`sim/observation_adapter.py`, whose `_CircuitWalk` docstring asserted the
opposite. Ask the extension manager; never infer this from argv.

Corroborated by effect rather than only by the log: with the extension up, the
capsule's **authored** z of 0.925 becomes 0.895 within the first frames of
Play. Something is simulating the prim. That is the resting height of a
controller with `contactOffset` 0.02 on a capsule of half-height 0.875, so it
is the character controller and not a coincidence.

**What it changes about the S11 contract result: nothing that the contract
asserts, and one sentence of the reasoning behind it.** No contract check
depends on which agency moved the capsule — they are about payload shape,
units, frame, and that readings react. What is retracted is the *reason the
scripted walk was called safe*: "nothing contends for the capsule's transform"
was false, and the walk was contended and won anyway rather than by design.
Two residues, both small and both worth knowing:

- `_CircuitWalk` reads the capsule's z once, in `Run.setup()`, **before**
  `play()` — so it captures the authored 0.925 and re-asserts it every tick
  while physics pulls the controller to 0.895. A ~3 cm disagreement, refreshed
  every frame. Which value the renderer sees was not measured.
- The existing caveat is unchanged and now sharper. "A scripted walk, not CCT
  collide-and-slide — it says nothing about the character controller" was
  written on the assumption that no controller was there. One was, and the
  walk's positions were applied anyway: `sim/verify_avatar_pose.py` writes the
  same transform under the same live controller and reads it back at **0.000
  mm error** at every waypoint. So a USD transform write is not overridden by
  the controller's own position — the caveat stands for the reason it always
  did, that a write is a placement and not a swept move, and not because
  nothing was listening.

### A PhysX character controller has no rotation, so the capsule cannot be yawed

Third finding, briefly, because it cost the first run of the verification
script. Commanding yaw 0 / 90 / −90 / 0° produced a first-person camera
heading of **0.00° at all four waypoints** while the visible character — which
physics does not touch — turned by exactly the commanded angle each time. The
capsule's `xformOp:orient` reads back as `(1, 0, 0, 0)` at every sample no
matter what was written, so this is not a Fabric-versus-USD problem that
reading somewhere else would fix: PhysX simulates the prim as a character
controller, a controller has a position and an up direction and no
orientation, and the writeback puts identity back every frame.

Nothing raises. The write succeeds, the attribute exists, the type is right,
and the value is gone before the renderer sees it.

Consequence, and it is general: **anything that must turn with the avatar has
to live on a prim physics does not own.** The cameras are those prims, and
they are also the half that matters — what the avatar sees is the camera's own
aim. `avatar.set_avatar_pose()` therefore writes each camera's `rotateXYZ`
directly, pitch preserved, and keeps the capsule write only for the cases
where physics is not holding the capsule.

---

## SEMANTICS AUDIT AND RENDER PROBE — 2026-08-25, and the recon was wrong by a factor of 313

Conditions: idle host, all four 3090s free, no other GPU tenant, exec mode under
`runheadless.sh`, `observatory_avatar.usd`. Two scripts, both report-only:
`sim/spikes/_diag_semantics_audit.py` reads USD and renders nothing;
`sim/spikes/_diag_semantics_render.py` renders and reads `idToLabels`. Neither
authors a semantic label, saves a layer, or creates a prim under `/Root` — the
render probe snapshots the prim set under the default prim before and after
setup and diffs it, so "the stage was not touched" is measured (0 created, 0
removed) rather than asserted. Outputs: `logs/s10_semantics_audit.{json,log}`,
`logs/s10_semantics_render.{json,log}`.

**Headline: the observatory stage is 98.5% labelled, not 0.3%.** `tasks/SERVER.md`
S5 recorded "Only **11** prims carry semantics, all of them parts of the worker
character; the shelving, floor, walls and forklift are unlabelled." That is
wrong and has been corrected in place. Measured: **3,441 of 3,493 renderable
prims carry a direct label — 98.5% by count, 99.0% by bounding-box volume,
across 37 classes.**

### The schema split, which is why the recon read 11

Two incompatible semantics schemas ship in 6.0.1 and this stage uses both. Read
out of the shipped `generatedSchema.usda` of each, not recalled:

| schema | applied as | attribute | entries here |
|---|---|---|---|
| `UsdSemantics.LabelsAPI` (current, `omni.usd.libs`) | `SemanticsLabelsAPI:<tax>` | `token[] semantics:labels:<tax>` | **13** |
| `Semantics.SemanticsAPI` (deprecated, `omni.usd.schema.semantics`) | `SemanticsAPI:<inst>` | `string semantic:<inst>:params:semanticType` + `…:semanticData` | **3,467** |

The 2026-08-12 recon looked for the current schema only. The 11 it found are
the Worker's; the 13 current-schema entries are the avatar's, authored later by
`sim/avatar.py`. Zero prims carry the attributes without the API schema applied
— the third case, and the nastiest, since it looks correct in a layer dump and
`LabelsAPI.Get()` cannot see it.

Class census, `class` taxonomy (the only one on this stage): `box` 1841,
`rack` 432, `sign` 253, `pallet` 158, `wall` 122, `bottle` 120, `ceiling` 72,
`floor_decal` 72, `lamp` 60, `bracket` 59, `pillar` 54, `floor` 54, `crate` 51,
`barel` 37 (the asset's spelling), `barcode` 20, `wire` 17, `person` 13,
`paper_note` 12, `fire_extinguisher` 7, `cone` 6, `emergency_board` 3,
`fuse_box` 2, then singletons. Only **41** renderable prims carry nothing: 40
ceiling beams `SM_BeamA_9M` (0.36 m³, 7.3 m² each) and one forklift fork.

### Do the 6.0.1 annotators read the deprecated schema? YES — measured

USD cannot answer this and neither can string-scanning the plugin
(`libomni.syntheticdata.plugin.so` carries strings for *both* schemas). The
audit's headline was therefore conditional: deprecated read → 98.5%; not read →
0.4%, the avatar alone. Settled by render, four probes, 480×480, targets chosen
off the stage rather than hardcoded.

**`idToLabels` alone does not settle it** — a class can appear in the map with
no pixel carrying it. The number that answers the question is the labelled
pixel fraction.

Probe `racking`, camera on the largest `rack`-labelled prim
(`/Root/Warehouse/SM_RackPile_91/…/Section0`), whose labels are *entirely*
deprecated-schema:

```
LABELLED 227,594 / 230,400 px = 98.78% of frame     18 map entries, 16 with pixels
rack    55,429  24.06%     pallet  14,505  6.30%     sign          1,361  0.59%
wall    45,717  19.84%     pillar  14,415  6.26%     floor_decal   1,052  0.46%
ceiling 30,162  13.09%     lamp    10,364  4.50%     bracket         283  0.12%
barel   22,212   9.64%     box      5,999  2.60%     wire            192  0.08%
floor   21,267   9.23%     crate    3,147  1.37%     UNLABELLED    2,806  1.22%
```

Probe `avatar` (positive control, current schema only): `person` 7,233 px
(3.14%), frame **99.89%** labelled. Probe `stage_camera`, the existing
`/Root/Avatar/body_mesh/cam_third_person` with nothing created: `person`
15,276 px (6.63%), frame **100.00%** labelled.

**Both schemas are read.** Since all 13 current-schema labels are `person` on
the avatar, every one of those 16 warehouse classes can only have come from
`Semantics.SemanticsAPI`. No migration is required for segmentation to work.

### The controls, because "empty" has more than one cause

Every probe carried `rgb` (is anything rendering) and
`instance_id_segmentation` (is the target in frame — it maps prim **paths** and
never consults semantics, so it separates "the annotator ignored these labels"
from "the camera was pointed at a wall"). All four: rgb ~921,500 non-zero px,
target confirmed in frame at 4.03% / 3.14% / 2.95% / 6.63% of pixels, 390 / 50 /
356 / 134 distinct instances visible. `ids_missing_from_map_pixels` was **0**
everywhere — no pixel carried an id absent from `idToLabels`, which is the thing
`tests/contract.py` forbids of the observation adapter and which the annotator
itself also honours.

### RETRACTED: "/Root/Worker reads as unlabelled"

The audit found `/Root/Worker` with **11 renderable meshes, 0 direct labels, 11
inherited** — its labels sit on the parent Xforms
(`/Root/Worker/ManRoot/Worker/Field_Jacket`), one level above the meshes
(`…/Field_Jacket/Field_Jacket/Field_Jacket`). From
`omni.syntheticdata.set_default_semantic_filter(predicate="*:*",
hierarchical_labels=False, matching_labels=True)` being the shipped default —
"option to propagate semantic labels within the hierarchy, from parent to
children" — and `omni.replicator.core` calling `set_semantic_filter` without
overriding it, **it was inferred that the Worker would read as unlabelled. That
inference is wrong.**

A third probe framed whichever subtree the scan found in exactly that state,
discovered rather than hardcoded, and ancestor labels reach the annotator:

```
fieldjacket 3,098 px 1.34%   basketballshoes 377 0.16%   basebody   295 0.13%
cargopant   2,859 px 1.24%   baseballcap     165 0.07%
```

The six that did not appear — `baseeye`, `baseeyeocclusion`, `basetearline`,
`baseteeth`, `basetongue`, `vneckscrubshirt` — are eyes, tearline, teeth,
tongue, eye-occlusion and the shirt under the jacket: not visible on a clothed
person at 3.7 m, not a semantics failure.

The lesson is the ordinary one and it cost a wrong claim in a report: a default
argument read out of shipped source is a hypothesis, not a measurement. It was
labelled as such at the time and was still wrong. `s7_camera_capture.jsonl`
(2026-08-15) had already contradicted it — its `idToLabels` contains
`fieldjacket`, `basebody` and `baseballcap`, which exist nowhere but on those
parent Xforms — and nobody had read the label names out of it.

That probe also caught both humans in one frame: the Worker as
`fieldjacket`/`cargopant`/`basebody`, the avatar as `person` (3,880 px). **Two
people in the scene, two vocabularies, and `person` on only one of them.** That
is a ground-truth defect for a navigation benchmark and it is a labelling
*quality* problem, not a schema one — as is `barel`.

### Replicator captures NOTHING while the timeline is stopped

Incidental to the probe and now failure mode 10 in CLAUDE.md. The render script
deliberately samples with the timeline stopped first, so that a run which fills
never perturbs the scene at all. It does not fill:

```
40 stopped frames   ->  no data on any of 4 render products
play()              ->  all 4 filled at frame 49, the very next frame
```

Filling one frame after `play()` is what rules out "it just needed a longer
warm-up". Nothing raises; `get_data()` returns an empty buffer, which is
indistinguishable from a working sensor that sees nothing. A capture script that
samples a fixed number of frames and exits therefore writes a complete,
well-formed, entirely empty dataset. Capture mode: press Play, warm up, *then*
sample.

### What is NOT decided

**Migrating the 3,467 deprecated-schema labels to `UsdSemantics.LabelsAPI` is
optional and undecided.** Segmentation works without it — that is the measured
result above, not an assumption. Against migrating: it edits a shipped asset's
semantics for no functional gain today. For migrating: the schema is deprecated,
`pxr.Semantics` already warns on import, and a future Isaac release that drops
it would silently take 98.5% of the ground truth with it. The path, if it is
ever taken, is
`isaacsim.core.experimental.utils.semantics.upgrade_prim_semantics_to_labels(prim,
include_descendants=True)` — old `semanticType` becomes the new taxonomy, old
`semanticData` becomes the label, and it removes the old API as it goes. **Do
not run it as a side-effect of anything else.**

---

## S11 OBSERVATION ADAPTER — 2026-08-25, and two conversions nobody had counted

Conditions: idle host, all four 3090s free, exec mode under `runheadless.sh`,
`observatory_avatar.usd`, no collider mask and no `minFrameRate` (capture mode,
CLAUDE.md rule 6). Seven sensors resolved: INFRA_01 camera + lidar, three robot
cameras, two avatar cameras. Radar skipped — it needs the three Motion BVH kit
flags.

**Result: `tests/contract.py` passes unchanged against the live simulator.**
`22 passed in 27.59s`, pytest exit 0, the same 22 tests the mock passes. That
is the S11 gate and the M3 handoff, closed.

### `IsaacExtractRTXSensorPointCloud` does step 1, not steps 1 and 2

CLAUDE.md said it did both. It does not, and the correction matters because
step 2 is the one that breaks fusion.

Read out of the shipped extension in this image
(`isaacsim.sensors.rtx.nodes`, Isaac Sim 6.0.1):

- it converts spherical→Cartesian, and **publishes** `outputs:transform` —
  *"The sensor-to-world transform matrix at the time of capture"* — **without
  applying it**. NVIDIA's own `test_point_cloud_annotator.py` asserts the
  output equals `r·cos(el)·cos(az)`, elementwise, i.e. **sensor-local**;
- its output list is azimuth, elevation, distance, intensity, normals,
  material/object/channel/echo/emitter/tick ids, `rv_ms`, timestamp,
  transform — **and no `flags`**. So the raw `generic-model-output` buffer has
  to be read every tick anyway, or there is no `VALID` mask and no sentinel
  drop.

The two buffers are element-aligned by construction — the node emits exactly
`numElements` points in input order, which is what that same NVIDIA test
asserts — so pairing them per tick is sound. `sim/observation_adapter.py`
reads both.

Enum values, read from `omni.sensors.generic_model_output` rather than
recalled: `CoordsType {CARTESIAN 0, SPHERICAL 1}`,
`FrameOfReference {SENSOR 0, WORLD 1, CUSTOM 2, PARENT 3}`,
`ElementFlags.VALID 64`.

Measured output, INFRA_01_LIDAR, sensor at world (4.910, 6.610, 2.60):

```
290,160 points   frame=world   decoded_by=IsaacExtractRTXSensorPointCloud
sensor_to_world=IsaacExtractRTXSensorPointCloud.transform
xyz_min [-26.368, -23.433, -0.004]   xyz_max [5.461, 30.625, 9.005]
```

Floor at z ≈ 0 and a 9 m ceiling, from a sensor at 2.6 m. Degrees would not
look like that, and neither would a cloud left on the origin.

### `distance_to_camera` writes 0, not `inf`, where the ray hit nothing

Replicator's own annotator documentation, in this image: *"0 in the 2d array
represents infinity (which means there is no object in that pixel)."*
`core/observation.py` specifies `inf` for that case and forbids NaN.

Measured share of no-hit pixels, one tick:

```
BOT_01_CAM   depth_finite_frac 0.7065    <- 29% of the frame is "no hit"
BOT_02_CAM   depth_finite_frac 1.0000       depth 0.179 .. 35.356 m
BOT_03_CAM   depth_finite_frac 1.0000       depth 0.152 .. 32.469 m
INFRA_01_CAM depth_finite_frac 1.0000       depth 5.329 .. 43.565 m
```

BOT_01 is the TurtleBot at 0.2 m, looking up past the racking into open space.
Left at 0 those 29% read as **a surface at zero range** — nearer than anything
real, so they win every min-depth, nearest-obstacle and collision-margin query,
and the more open the space the more confident the reading that something is
against the lens. Nothing raises: 0.0 is a legal float, non-negative, and
survives every shape, dtype and "is this metric, not normalised" check the
contract makes. The adapter maps `0 → inf` and `NaN → inf`.

The same annotator is euclidean range from the camera origin and already
applies `metersPerUnit`, so it needs no scaling — only USD-derived poses do.

### `points` is world frame, and the mock had been wrong since M3

`core/mock_source.py` emitted sensor-local clouds; the adapter emits world.
Both looked correct in isolation for the reason `core/observation.py` had
already guessed and then mis-concluded: it left the frame unpinned because
"pinning it now would be a claim no suite could check."

That was true only while the check was imagined as a property of one cloud. A
fixed translation breaks none of `(N, 3)`, float, finite, non-empty, or
reacts-to-the-avatar, and for a sensor at the origin the two conventions are
literally the same numbers. `POINTS_FRAME = "world"` in `core/observation.py`
is now the pinned convention, and `tests/contract.py` gained
`test_range_clouds_are_in_the_world_frame`.

**The first version of that test was unsound, and how it failed is the useful
part.** It compared the nearest return to the avatar under both readings —
world, and sensor-local-plus-the-mount — expecting them to be separated by the
mount offset. Against the mock they are, decisively:

```
INFRA_01_RADAR: nearest return is 7.24 m from the avatar as world coordinates,
                but 0.55 once the sensor's own position is added.
```

Against the live simulator it **accused a correct adapter**:

```
INFRA_01_LIDAR: 0.89 m as world  vs  0.12 m as sensor-local
                0.99             vs  0.95
                1.65             vs  1.24
```

Both readings fit, because the lidar returns **290,160 points spanning the
whole warehouse**: "some point is within 1.5 m of X" is true for almost any X,
under either hypothesis. The mock has ~2,000 points and the simulator 290,000,
and the test had silently assumed the sparse case. **Cloud density is not
something a contract may assume** — a discriminator that works on a fixture
with a thousand points can be meaningless on one with a third of a million.

The sound version uses **the floor** as the known target. A sensor mounted `h`
above it sees it below; in world coordinates those returns sit at the floor's
height, and read as sensor-local they sit at `-h` — underground, which is not
somewhere a range sensor can put a return. The floor's height is derived, not
assumed: the avatar's own lowest camera height less the `eye_height`
scene.yaml declares, so the cloud is measured against a pose the same tick
reported. Verified by regression against the mock, where the measured floor
lands on the predicted one:

```
INFRA_01_LIDAR mounted 6.50 m above the floor, lowest return -6.70 (predicted -6.50)
INFRA_02_LIDAR mounted 3.00 m above the floor, lowest return -3.00 (predicted -3.00)
```

It is honest about its blind spot rather than silent: a translation with no
vertical component, or a mount too low for `-h` to clear the tolerance, cannot
be seen — so those sensors are excluded by an explicit guard, and if none
qualify the test fails as vacuous instead of green.

Fixing the mock exposed a second defect in it: with clutter generated as
`distance x sin(elevation)` and no floor, a 3 m mount returned points **1.42 m
underground** (INFRA_02_LIDAR, measured). Downward beams now terminate at the floor. Nothing had ever
noticed, for the same reason nothing noticed the frame — a cloud is only ever
checked against itself.

Generalisable, and the reason this is written down: **"no test could check
this" is worth doubting once**, in case what it really means is that the
fixture was too symmetric to tell. And the correction to that: a test built on
one fixture is a hypothesis about the other.

### Two silent failures found by running it, not by reading it

1. **A lidar's GMO carries the radar-only `rv_ms` member as an unfilled
   pointer, and slicing it returns an EMPTY array rather than raising.** It
   surfaced ~100 lines from the cause as
   `boolean index did not match indexed array; size of axis is 0 but size of
   corresponding boolean axis is 289930`, and every lidar reading was dropped
   for 300 warm-up frames while the cameras looked fine. Per-element arrays are
   now length-checked against `numElements`, not assumed.
2. **pytest inside Kit loads Isaac's bundled ROS 2 `launch_testing` as a
   `pytest11` entry point, which imports `lark`, which this image does not
   have.** It died before collecting a single test.
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` before `pytest.main`.

### How the suite runs at all, given exec mode

`ObservationSource.step()` is synchronous and the contract calls it in a plain
loop; exec mode forbids `app.update()` and frames come from the update event
stream. A synchronous loop on Kit's main thread can never yield a frame, so
both cannot be true at once — unless `step()` is thread-aware:

- **on the main thread** it reads what the renderer last produced (the capture
  loop is already inside an update callback);
- **on any other thread** it posts a request, the main thread services it on
  its next update, and the caller blocks.

Sampling always happens on the main thread; the worker only ever touches numpy
that was copied there. pytest runs on the worker. Copies are not optional:
annotator buffers are recycled, and the contract's `trace` fixture holds a
whole walk at once — handing out views would make every tick identical and
turn "the sensor never reacted" into a finding about the adapter.

The avatar is keyboard-driven and has no trajectory, so the headless run
supplies one: a 2.5 m circle written straight to the capsule transform, with
`omni.physx.cct` deliberately **not** enabled so the character-controller node
type stays unregistered and nothing contends for it. That is a scripted walk,
not CCT collide-and-slide — it says nothing about the character controller.

---

## GUI PERFORMANCE — 2026-08-24, and it is the first uncontended measurement

**Every earlier fps number in this project was taken on a loaded host.** The
collider sweep that produced **2.48 → 19.69 fps** ran at load ~13.9. This
session ran at load **0.68**, no other GPU compute processes, all four 3090s
free. So 19.69 is a *floor*, not a result, and no number below is comparable to
anything measured before today.

Conditions unless a line says otherwise: `make gui`, two viewport panels,
**camera stationary**, avatar not driven. Both timeline states are reported —
stopped and playing differ by more than any knob tested here. Numbers are the
viewport HUD's and the `[gui_viewports] fps` line's, which agree.

### Panel render resolution — 38 → 50 fps stopped, and the arguments lied

`create_viewport_window(width=, height=)` sizes the **window**, not the render
target. Both panels were rendering **1280×720** and downscaling into a 640×360
box: ~75% of every panel's pixels computed and discarded. The render resolution
is a separate property, `ViewportAPI.resolution`, set after the window exists.

```
1 panel                              59-60 fps stopped   (app loop caps at 60)
2 panels, 1280x720 render               38 fps stopped
2 panels,  640x360 render               50 fps stopped   +32%
```

Per-frame, that is 16.7 ms → 26.3 ms → 20.0 ms, so the **cost of one extra
panel fell from ~9.6 ms to ~3.3 ms**. That is what makes the five-viewport S9
demo (`GUI_ROBOT_CAMS=1`) viable; at 9.6 ms each it was not.

### The lidar is free at Play — both halves of it

Measured with the `GUI_LIDAR` / `GUI_LIDAR_DRAW` gates, at Play:

```
GUI_LIDAR=0        no lidar created            14-20 fps
GUI_LIDAR_DRAW=0   lidar casts, nothing drawn  14-20 fps
both on            lidar + ~419,000 points     ~17 fps
```

Ray casting and drawing ~419,000 points are **both below measurement noise**.
The three configurations are one spread, not three. **S8's debug draw costs
nothing** — it can stay on for the demo.

### Physics is 100% of the Play-state cost

```
stopped   ~30-40 ms/frame
playing   ~50-70 ms/frame
```

Nothing else moved that number in this session. The remaining cost is the
**~2,000 exact triangle-mesh colliders** still enabled after
`disable_unreachable_colliders` switches off 1,486.

### Thread count is not a lever

`--/persistent/physics/numThreads` at **16 and 32 are indistinguishable** on
this 64-thread EPYC 7543. `0` — synchronous, physics on the main thread — is
**untested**.

### Camera motion costs ~10-20 fps, stopped

Moving the viewport camera while measuring is worth more than most of the knobs
above. **Fix the camera position before comparing any two runs.** This is the
explanation for the 50 vs 30-40 fps discrepancy between readings taken in this
same session — not lidar state, not thread count.

### Renderer

**RTX - Real-Time 2.0**, confirmed in the viewport HUD. Everything above was
measured under it. `--/rtx/pathtracing/cached/retrace=0.1` remains **untested**;
this session ran no path tracing at all.

### Convex-hull conversion — REJECTED, and not on performance grounds

Converting the remaining triangle-mesh colliders to convex hulls would be the
obvious next win, and it is refused. The benchmark measures **dynamic
environment change during embodied navigation**, which makes collision geometry
*the measured quantity*, and a hull fills concavities — a shelf you can reach
into becomes a shelf you cannot. Walls and floors are convex already and could
convert safely; **anything that can move during an episode must stay exact.**
Buying frame rate here would mean buying it out of the result.

### Two comments this session made stale

Recorded, not edited — they belong to the code that owns them:

- **`sim/gui_viewports.py`**, on the collider mask: *"+0.8% from 1280x720 ->
  960x540"*. That was true when measured, at Play, where physics dominates and
  swamps any render cost. It reads today as "resolution does not matter", and
  panel **render** resolution measured **+32% stopped**. Different state,
  different conclusion.
- **`sim/sensor_factory.py`**, the `lidar_draw` docstring: it exists to
  *"separate the cost of drawing ~419,000 points from the cost of casting
  them"*. The measurement it was written for has now been made, and **there is
  no drawing cost to separate.** The parameter is still useful as a control;
  its stated premise is spent.

---

## PEOPLE AND THE WALK CLIPS — 2026-08-17 (SUPERSEDED 2026-08-26)

> **Superseded by the walk-cycle entry at the top of this file.** The note
> below is kept verbatim because it is the one that cost a session: it was
> read as settled, it was acted on, and the thing it settled was not true.
> Its conclusion is wrong and its own log said so on the line above the
> conclusion.

Entered into this log on 2026-08-26, late. It was written up in a commit
message (`b7f789a`) and in `_diag_people_and_pose.py`, and never here — which
is part of why nothing checked it. **A finding that is not in this file does
not get corrected by this file.**

### What it claimed

> *Isaac/People ships characters on the SAME skeleton the avatar already
> rides. All ten in `Isaac/People/Characters` report 101 joints with joint
> names identical to the Worker's `RL_BoneRoot/Hip/...` list, compared element
> by element rather than eyeballed. `omni.anim.people` itself is absent from
> this image, but it is not needed: `Isaac/People/Animations` carries the clips
> directly, including*
>
> ```
> stand_walk_loop_in_place.skelanim.usd     <- the one that matters
> ```
>
> *So this is a reference swap plus a blend, NOT retargeting.*
>
> *Caveat kept: the clips are SkelAnimation-only assets with no Skeleton prim,
> so the joint-list comparison could not run against them directly —
> `joints=0` in the log means "no Skeleton here", not "different skeleton".
> They ship in People alongside characters that DO match. Strong inference,
> not proof; the first ten minutes of that task should confirm it.*

The first half is true. The characters really are on the Worker's skeleton —
re-measured 2026-08-26, `male_adult_construction_01` is `RL_BoneRoot`, 101
joints. **The inference from it is false.**

### What was actually true

```
  the avatar's skeleton     101 joints   RL_BoneRoot/Hip/Pelvis/L_Thigh/...
  every Isaac/People clip    81 joints   Root/Pelvis/R_UpLeg/R_LoLeg/...
  joints in common                    0
```

Zero. Not a different order — no joint name in `stand_walk_loop_in_place` or
any of its siblings exists on this skeleton. And because People's characters
ARE the Worker's rig, **People's clips do not fit People's own characters
either**, without the retarget step `omni.anim.people` would normally supply
and which is absent from this image. Neither half of that library is what it
looks like beside the other half.

A `SkelAnimation` carries its own `joints` array — that is how UsdSkel binds a
clip to a skeleton, by NAME — so the comparison the note called impossible was
available the whole time, on the clip itself, without a Skeleton prim
anywhere. `_diag_walk_clip.py` runs it in about a second.

### Why it read as settled, and this is the part worth keeping

Its own log printed the answer directly above the verdict:

```
  male_adult_construction_01_new: joints=101 anims=0 same_skeleton=True
  ...
  stand_walk_loop_in_place.skelanim.usd: joints=0 anims=1 same_skeleton=False
  ...
  WALK CLIP: MATCH -> .../Isaac/People/Animations/LookAround.skelanim.usd
```

**`same_skeleton=False` on every clip, and then `MATCH`.** And the asset it
matched is `LookAround`, which is not a walk and was never a candidate — it is
simply first alphabetically. One line of `_diag_people_and_pose.py` does it:

```python
if anims and (same or joints is None) and out["match"] is None:
    out["match"] = rec
```

`joints is None` means *this asset has no Skeleton prim, so its joint list is
unknown*. It is scored as a match. Unknown is not agreement, and the two must
never share a branch — the fallback turns "I could not check" into "checked,
fine", and prints it in the summary line in capitals with an asset path beside
it, which is exactly what a real result looks like.

Nothing raised. The caveat in the note is a fair description of the gap and it
was still not enough, because the machine-printed verdict said MATCH and the
prose caveat said *strong inference*, and between a confident line of output
and a hedge in a paragraph, the next session believes the output.

**Two things follow, and both are general:**

- **A diagnostic must not have an "unknown" that falls through to a pass.**
  Where a check cannot run, it reports that it did not run — `UNKNOWN`, its own
  column, never folded into the affirmative case.
- **A scoping conclusion goes in this file, marked with what was measured and
  what was inferred.** This one lived in a commit message for nine days.

The recommendation the note made — retarget People's motion-captured walk onto
this skeleton with `omni.anim.retarget.core` — survives the correction and is
still the principled route to a better cycle. What does not survive is
"reference swap plus a blend": there is no swap that works, and
`avatar.WalkCycle` exists because of it.

---

## S4 MEASURED — 2026-08-14, and the answer is NO with one large caveat

The coordinate bug diagnosed below (by reading only) was fixed and the sweep
was run. Both sensors work. Both localise geometry to the millimetre. The
occlusion question is answered; the *material* question is not, and the reason
is a third bug that only became visible once the first two were gone.

### The answer

**Does RTX radar see through what stops lidar? NO — for all six materials
tested. But the material comparison is vacuous, so treat this as "the wall is
opaque to both", not as a result about materials.**

| condition | radar valid | radar @target | radar @wall | lidar valid | lidar @target |
|---|---:|---:|---:|---:|---:|
| clear LOS, target moving | 72 | **72** | 0 | 6,786,324 | **980,196** |
| clear LOS, target static | 60 | **60** | 0 | 6,787,338 | **974,100** |
| wall @4 m, no target | 30 | 0 | **30** | 9,119,304 | 0 |
| empty (ground only) | 30 | 0 | 0 | 5,984,826 | 0 |
| steel \| none \| none | 30 | **0** | 30 | 9,119,676 | **0** |
| concrete \| none \| none | 30 | **0** | 30 | 9,120,129 | **0** |
| cardboard \| none \| none | 30 | **0** | 30 | 9,120,144 | **0** |
| plastic \| none \| none | 30 | **0** | 30 | 9,120,387 | **0** |
| clear_glass \| none \| none | 30 | **0** | 30 | 9,120,996 | **0** |
| fabric \| none \| none | 30 | **0** | 30 | 9,119,562 | **0** |

30 frames per condition, 15 discarded as warm-up. Radar and lidar ran as
separate processes (both sensors at once still dies on `cudaErrorIllegalAddress`
and that was not retested). Exhibits: `logs/s4fix_radar_smoke.jsonl`,
`logs/s4fix_lidar_smoke.jsonl`, `logs/s4fix_lidar_ctl2.jsonl`.

### The caveat that matters more than the answer

**The non-visual material swap moved neither sensor's return.** Radar reports
`scalar` as RCS in dBsm, and across all six materials it is:

```
steel        10.160172      plastic       10.160171
concrete     10.160171      clear_glass   10.160171
cardboard    10.160172      fabric        10.160172
```

Identical to seven significant figures. Lidar's wall-face intensity is likewise
flat at 0.0000403 across all six. **The sweep measured the same wall six times.**
Whether `NonVisualMaterial.set_bases()` never reached the renderer, or the
WpmDmat radar model ignores `omni:simready:nonvisual:*` altogether, is not
established and is the next thing to find out.

The spike already had a guard for exactly this — and it did not fire, because it
read *lidar* intensity only and the material run was radar-only, so it printed
`INCONCLUSIVE` and let `ANSWER: NO` through underneath it. That guard now checks
every sensor present, and radar's RCS is the right quantity for it.

### What the sensors actually did — both were working the whole time

Radar, clear line of sight, target oscillating ±0.5 m:

```
az[-17.978, 17.978]deg  el[0.000, 10.239]deg  range[5.501, 6.686]m
world x[5.500, 6.501] y[-1.898, 1.940] z[1.000, 2.110]   rv_ms[-9.270, 9.270]
```

World x spans **5.500 to 6.501** — the 4 m cube's near face (centre 8 ± 0.5,
half-extent 2) tracked to the millimetre, with 70 of 72 returns carrying
non-zero radial velocity. Static, it collapses to x = 6.001. With the wall up,
every return is at x = **3.900**, the wall's near face. The radar was never
broken and has been localising geometry correctly in every run this file
records.

Lidar sees **974,100** points on the target with a clear line of sight and
**0** behind any wall — where the old code reported 0 in both cases.

### The empty scene returns a sentinel, and it is flagged VALID

With nothing in the scene but the ground plane, the radar emits 30 detections
in 30 frames at **exactly** `az 0.000°, el 0.000°, range 100.000 m` — a round
number, zero variance, no geometry there. That is a no-detection placeholder,
not a measurement.

**It carries `flags = 64`, the VALID bit.** So `flags & VALID` does *not* mean
"this is a real return", and a consumer that trusts it — S11's observation
adapter above all — will publish a phantom point at 100 m every frame the scene
is empty. Range-gating against `maxRangeM` (200 m by default) will not catch it
either, since 100 is comfortably inside.

This is the one place the earlier "sentinel" guess was right. It was not right
about the 6.155 m constant, which was the target.

### Why "one return per frame" is a sensitivity ceiling, not just an oddity

Measured, it is 1–2.4 detections per frame and it does vary with the scene. The
structural point stands: the radar reports only its strongest few returns, so a
weak return from *behind* a wall can never outrank the wall's own echo. **With
the default configuration the sweep can only detect penetration if the wall's
return disappears entirely.** Any future material result has to clear that bar
before it means anything.

### The radar's config surface — read from the shipped schema

`OmniSensorGenericRadarWpmDmatAPI` (`isaacsim.sensors.experimental.rtx/.../sensor_checker/generatedSchema.usda`),
and its `OmniSensorGenericRadarWpmDmatScanCfgAPI:s001` sub-schema. The schema
confirms the two defaults this file's diagnosis rests on, for the radar
specifically:

```
omni:sensor:WpmDmat:elementsCoordsType   = "SPHERICAL"   (allowed: CARTESIAN, SPHERICAL)
omni:sensor:WpmDmat:outputFrameOfReference = "SENSOR"    (allowed: SENSOR, WORLD, CUSTOM)
```

Levers for the detection count, none of which were needed to answer the
occlusion question and all of which are now reachable via `SPIKE_RADAR_ATTRS`:

| attribute | default | why it matters |
|---|---|---|
| `WpmDmat:cfarMode` | `"2D"` | `"4D"` adds angular CFAR — the most likely single lever |
| `scan:s001:cfarOffset` | `1` | threshold multiplier; lower = more detections |
| `scan:s001:cfarRnT/VnT/AznT/ElnT` | `1` each | CFAR test cells per dimension |
| `scan:s001:cfarMinVal` | `7e-17` | minimum bin energy to consider |
| `scan:s001:azBins` / `elBins` | `12` / `2` | **2 elevation bins over ±20°** — elevation is effectively unresolved, which is why every return reports el ≈ 0 |
| `scan:s001:rcsTuningCoefficients` | `[-12, 150, 0]` | scales returned RCS |
| `scan:s001:debugForceDetection` | `0` | forces a synthetic detection at `debugForceRange`/`Az`/`El`/`V` — an end-to-end readback validator that needs no scene physics |

`maxRangeM = 200`, `maxAzAngDeg = 66`, `maxElAngDeg = 20`, `rangeResM = 0.4`:
the target at 8 m, azimuth 0°, elevation 0° is comfortably inside the field of
view, so FOV was never the limiter.

Attribute names are the full USD paths, and NVIDIA's own `test_radar_sensor.py`
passes them the same way:
`Radar(path, attributes={"omni:sensor:WpmDmat:outputFrameOfReference": "WORLD"})`.

### The second bug: `no_target` and `empty` still contained the target

Only visible once the decode was correct. `_condition(moving=True)` oscillated
the target about the literal `TARGET_X`, so `no_target` — whose entire job is to
remove the target — pushed it to 1 km and then teleported it straight back to
x ≈ 8 on **every collected frame**. `empty` then inherited it at the last sine
phase.

The signature was unmistakable once the units were right: `empty` reported the
target's near face at **6.155 m**, which is exactly `8.154 − 2` for phase
`i = 29` of the oscillation. Both control conditions were invalid.

Fixed by oscillating about wherever the target is currently parked
(`SCENE["target_base"]`, set through `_park_target`). After the fix `no_target`
and `empty` both report 0 target points for both sensors, and `empty` drops to
the 100 m sentinel.

### The third bug: the wall box swallowed the floor

The replacement wall mask is a box, but the ground plane runs straight through
it, so it counted a ~559k-point strip of *floor* whether the wall stood at 4 m
or a kilometre away — making the discrimination gate unpassable by
construction. Excluding `z < 0.20 m` fixes it. Demonstrated on real data, which
is the check the gate now enforces before any answer is printed:

```
Wall-mask discrimination (box x (3.5, 4.5), |y|<12.5, z (0.2, 5.6)):
  lidar  wall @4m ->  4113504 pts | wall @1km ->        0 pts   DISCRIMINATES
```

Against the mask it replaces, which counted 76,317–76,350 points in *every*
condition — a 0.04% spread while the cloud itself swung 35%.

### Two environment facts, both cheap and both non-obvious

- **Two sim containers cannot run at once.** `network_mode: host` is mandatory
  for streaming, and it makes the second container die on
  `[Errno 98] address already in use` binding Kit's `0.0.0.0:8011`. Separate
  GPUs do not help; the collision is the port, not the card. Runs are
  sequential, full stop.
- **The exec-mode container does not exit when the script does.** It printed
  `DONE`, called `post_quit()`, and was still up 12 minutes later holding
  ~1.6 GB of GPU memory. `docker stop` it after each run or the next run starts
  on a card that is not as free as `nvidia-smi` suggested a moment earlier.

### Verification before the run, not after

`_to_world()` was checked on CPU against known geometry before any GPU time was
spent — the test execs the real source text out of the script rather than
reimplementing it. All checks pass: az 0°/el 0°/r 8 m → world (8, 0, 1); the
ground sweep at el −5°…−89° lands on z = 0 to 1e-6 at every angle; VALID
decodes 64/0/65 → true/false/true; CARTESIAN+WORLD passes through untouched;
and a densely sampled wall face gives 1,560 in-box points at 4 m against 0 at
1 km.

---

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

> **Resolved 2026-08-14 by the run recorded at the top of this file.** The two
> items below were both correct as diagnoses. The first was a fix and is done.
> The second turned out to be a *sensitivity ceiling* rather than a blocker: the
> radar does detect the target (72 and 60 returns in the two clear-line-of-sight
> controls), so there was a baseline to compare against after all — but because
> it reports only its strongest returns, it cannot see a weak return past a
> wall's echo. The occlusion question is answered; the material question is
> blocked by a *third* bug, the inert material swap.
>
> The CLAUDE.md note below has now been applied.

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

> **Note for CLAUDE.md — APPLIED 2026-08-14.** The "Environment facts" bullet
> reading *"RTX radar returns nothing usable … only ~1–2 detections per frame
> that do not localize a 2 m cube 8 m away and are unchanged by removing the
> target"* was wrong in its diagnosis. The radar localizes a wall to the
> millimetre and tracks the cube's near face across ±0.5 m of motion. The bullet
> has been replaced and the spherical/sensor-frame defaults added to the API-era
> section, which is exactly the kind of trap that section exists to catch.

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
| `radar_penetration_exec.py` | **The live spike.** Exec mode. Runs clean and correct as of 2026-08-14: `_to_world()` decodes spherical→Cartesian and sensor→world from the buffer header, masks are world-metre boxes, `flags & VALID` applied, `rv_ms` kept. `SPIKE_SENSORS=both\|lidar\|radar`, `SPIKE_RADAR_ATTRS` for radar prim attributes. |
| `logs/s4fix_radar_smoke.jsonl` | **The S4 radar result.** Controls + six materials, correct decode. Shows the flat RCS that makes the material sweep vacuous. |
| `logs/s4fix_lidar_smoke.jsonl` | Same sweep, lidar. 974k target points with LOS, 0 behind every wall. |
| `logs/s4fix_lidar_ctl2.jsonl` | Controls after the wall-box ground fix. The 4,113,504-vs-0 discrimination exhibit. |
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

> **Steps 1–4 are all done — 2026-08-14.** Read the top of this file for the
> measured result. What is left is one question, and it is not about the radar:
>
> **Why does swapping the occluder's non-visual material change nothing?** Radar
> RCS off the wall face is 10.16017 dBsm for steel, concrete, cardboard,
> plastic, clear_glass and fabric alike. Either `NonVisualMaterial.set_bases()`
> is not reaching the renderer, or the WpmDmat model ignores
> `omni:simready:nonvisual:*`. Until that is settled, no material result from
> this rig means anything. Two ways in: check the USD attributes actually
> written on `/World/occluder/material` after a `set_bases()` call, and check
> whether the radar's own docs tie RCS to those attributes at all.
>
> Second, smaller: with the default `cfarMode="2D"` the radar reports only its
> strongest returns, so a penetrating return can never outrank the wall's echo.
> Try `SPIKE_RADAR_ATTRS='{"omni:sensor:WpmDmat:cfarMode": "4D"}'` before
> trusting a negative.

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
