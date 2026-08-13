# spatial-sim — environment setup

Multi-sensor spatial observatory in Isaac Sim. Server renders, MacBook views and
develops CPU-side code.

**Read this first:** every step below has a *verification gate*. Do not advance
past a gate that hasn't passed. Isaac Sim's characteristic failure mode is
silence — wrong configuration produces no error, just no data.

> **Already set up and just want to run it?** Skip to **§8 Day-to-day
> operation**. If the screen is black, go to **§9**, which is a decision tree,
> not prose. Sections 1–2 are one-time host setup and you should not need them
> again.

---

## 0. Which Isaac Sim version

`ISAACSIM_VERSION` in `docker/.env` is the only place this is set.

**Default: 6.0.1.** The 6.x container is rootless, multi-arch, and has official
Docker Compose support with a browser-based viewer — which is exactly the remote
setup here. The docs site now defaults to 6.x, so building on 5.1.0 means
reading mismatched documentation every day.

The cost: 6.0 deprecated `isaacsim.sensors.rtx` in favour of
`isaacsim.sensors.experimental.rtx`, which reshapes the RTX lidar/radar API
(authoring classes `Lidar`/`Radar` split from runtime `LidarSensor`/`RadarSensor`,
array-form transforms, no command-based creation). The deprecated extension
still ships, so older examples continue to work — but new code should target the
experimental namespace, and this project is entirely RTX-sensor-shaped.

There are now **three** API eras in circulation:

| Era | Namespace | Where you'll meet it |
|---|---|---|
| 4.x | `omni.isaac.*` | Most web tutorials, most LLM training data |
| 5.x | `isaacsim.sensors.rtx` | 5.x docs, recent forum posts |
| 6.x | `isaacsim.sensors.experimental.rtx` | Current docs |

Check the 6.0 migration guide before trusting any recalled snippet.

Switch versions with one line:
```bash
ISAACSIM_VERSION=5.1.0 docker compose -f docker/docker-compose.yml build sim
```

---

## 1. Server — host-level setup

Docker cannot package the driver or the container runtime. Everything else lives
in this repo.

### 1.1 Prerequisites

```bash
ldd --version          # need >= 2.35. Ubuntu 22.04 OK, 20.04 fails.
free -g                # RAM: 32 GB minimum, 64 GB recommended.
                       # This is a more common wall than VRAM. Check it now.
nproc                  # physics is largely CPU-bound; multi-GPU won't help it
df -h                  # the Isaac image is large; assets larger
```

> **Gate:** GLIBC ≥ 2.35 and RAM ≥ 32 GB. If RAM is short, resolve it before
> anything else — you'll hit it at M6 when three robot viewports come up, and
> the symptom will look like a rendering bug.

### 1.2 NVIDIA driver

Install a production-branch driver via the `.run` installer from the Unix Driver
Archive. Note that the *newest* driver isn't always best — livestreaming has
historically lagged on brand-new releases.

```bash
nvidia-smi             # must list all 4 RTX 3090s
```

**Note the GPU ordering trap now, not later.** `nvidia-smi` index is *not* the
kernel device minor, and NVENC cares about the minor, not the index. Derive the
map before choosing `device_ids`:

```bash
grep -H 'Device Minor' /proc/driver/nvidia/gpus/*/information
```

On this host, `nvidia-smi` index 3 (`0000:c1:00.0`) is minor **0**. See §6 and
the comment in `docker/docker-compose.yml`.

### 1.3 Docker + NVIDIA Container Toolkit

```bash
curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh
sudo groupadd -f docker && sudo usermod -aG docker $USER && newgrp docker

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

> **Gate:** `make verify-gpu` lists all four GPUs.

### 1.4 NGC login — do this before the first build

```bash
docker login nvcr.io
# Username: $oauthtoken      <- literally that string, with the $
# Password: <NGC API key from ngc.nvidia.com>
```

Skipping this fails the pull with an auth error that reads like a network fault.

### 1.5 Firewall

```bash
sudo ufw allow 49100/tcp   # WebRTC signaling
sudo ufw allow 47998/udp   # WebRTC media  <-- the one people forget
sudo ufw allow 8210/tcp    # web viewer, if you use it
```

**Opening only the TCP port gives you a connection and a black screen.** WebRTC
media is UDP.

> **Security:** streaming has no authentication and no encryption. Never expose
> these ports publicly.

### 1.6 `ISAACSIM_HOST` is the LAN IP, never the tailnet IP

```
ISAACSIM_HOST = 143.248.57.94     # LAN       — correct
                100.96.11.37      # tailnet   — goes nowhere
```

Tailscale here runs in **userspace mode**, so there is no `tailscale0`
interface and the kernel has no route to the tailnet. Even this host cannot
reach its own tailnet address:

```
ip route get 100.96.11.37   ->  via 143.248.57.1 dev enp66s0f0   (default gw)
connect 100.96.11.37:49100  ->  FAILED
connect 143.248.57.94:49100 ->  CONNECTED
```

`tailscale serve` is **not** a workaround — it forwards TCP only, so signaling
would connect and UDP media would never arrive: a black screen.

**Consequence: streaming works only from a client that can reach the KAIST
LAN.** Fixing that needs a kernel TUN device, which needs root this server does
not have. That is an admin request, not a task.

---

## 2. Server — project setup

```bash
git clone <repo> && cd spatial-sim
cp docker/.env.example docker/.env
$EDITOR docker/.env               # ISAACSIM_HOST = the server's LAN IP.
                                  # NOT the tailnet IP — see the warning below.

make dev-build                    # 5 min, no GPU — catches repo problems early
make check test                   # boundary + tests must pass
make sim-build                    # long: large image pull
make compat                       # expect "System checking result: PASSED"
make encoder-check                # NVENC must OPEN A SESSION, not merely be
                                  # visible. "Visible" is the trap — see §9.2.
make stream                       # wait for "Isaac Sim Full Streaming App is loaded"
```

The first launch spends several minutes warming the shader cache. That is
normal, and `-v` shows progress so you can tell warm-up from a hang.

> **Gate:** `make ports` shows both 49100/tcp and 47998/udp listening.

**Second launch must be visibly faster.** If it isn't, the cache volumes aren't
mapped correctly — see the long comment in `docker-compose.yml`. Silent
cache failure costs ~10 minutes per restart, several times a day, forever.

---

## 3. MacBook

Isaac Sim itself will never run here: it needs an NVIDIA RTX GPU with RT cores,
Vulkan, and NVENC, and the image is Linux-only. (The 6.x container is now
multi-arch, but "aarch64" there means Linux ARM — DGX Spark, Jetson — not
macOS. No Dockerfile changes this.) The Mac is a thin client plus a real
CPU-side development machine.

### 3.1 Networking

Tailscale is installed on both machines and is genuinely useful for SSH and
file access — but **it cannot carry the stream.** See §1.6: userspace mode
means there is no tailnet route, and `tailscale serve` forwards TCP only, so
signaling would connect and UDP media would never arrive.

**So: the Mac must be on the KAIST LAN to view the stream.** Off-LAN, expect a
black screen no matter what else is correct.

Restarting Tailscale on the server after a reboot: see `RESTART.md` in
`/home/quang/EmbodiedAgent/tailscale`. There is deliberately no `tailscale0`
interface, no systemd unit, and no binary on `PATH` — that is normal, not a
broken install, and it will look absent to any check that greps for those.

```bash
cd /home/quang/EmbodiedAgent/tailscale
./tailscale --socket=$HOME/.tailscaled.sock status
```

### 3.2 Viewing — two options

**Native client (confirmed working):** download the DMG from the Isaac Sim
Latest Release page and drag it to Applications. Client platform support is
independent of host platform support. Enter **the server's LAN IP**
(`143.248.57.94`), click Connect.

Verified on 2026-08-10 against Isaac Sim 6.0.1:

| | |
|---|---|
| Client | `IsaacSimWebRTCStreamingClient/2.0.0` |
| Shell | Electron 41.7.0, Chrome 146.0.7680.216 |
| Renderer | ANGLE Metal Renderer, Apple M4 Pro |
| Chroma | RGB |

The Chromium/ANGLE fingerprint is the client's embedded Electron shell, not a
browser tab — don't chase it as a browser problem. Whether the binary is
arm64-native or running under Rosetta is **still unverified**; the server side
cannot see it. Check `Activity Monitor → Kind` on the Mac if it matters.

**Browser viewer (fallback):** NVIDIA's Docker Compose web viewer, served on
port 8210, works in any Chromium-based browser with no install. Note it's
supported on Ubuntu hosts only — that's fine, the *host* is your Linux server.

Either way: **one client at a time.** Close one session before opening another.

### 3.3 Development on the Mac

This is the payoff of the layer discipline:

```bash
make dev-build
make test
make check
```

`core/` never imports `omni`, `pxr`, or `isaacsim`, so registry and memory code
is fully unit-testable here with no GPU and no simulator.

> **Gate:** if `make test` cannot run on the MacBook, the boundary has leaked.
> `make check` will tell you exactly where.

---

## 4. Navigating the stream

Hold the **right mouse button**, then `WASD` to move, `Q`/`E` down/up, drag to
look. **Plain WASD does nothing** — this confuses everyone exactly once.

**Use a real mouse. A trackpad cannot do this.** A two-finger tap is a
right-*click*; it cannot be *held*, so the WASD binding never activates. The
symptom is precise and misleading: right-drag looks around perfectly, and
`WASD`/`QE` do nothing at all, which reads like broken keyboard forwarding.
It isn't — the keys never reach the binding. Confirmed on this project
2026-08-10; a USB mouse fixed it immediately.

If a real mouse still won't move you, check in this order:

1. **Is anything in the stage?** `runheadless.sh` opens the *default empty
   stage*. Flying through an infinite uniform grid looks identical frame to
   frame, so you appear stationary while actually moving. Select
   `/OmniverseKit_Persp` in the Stage panel and watch `xformOp:translate` in
   the Property panel — if the numbers change, navigation is fine and the scene
   is empty.
2. **Fly speed bottomed out.** Scrolling while RMB is held adjusts navigation
   speed. Hold RMB and scroll up a few notches.
3. **Viewport focus.** Left-click once inside the viewport. Mouse events carry
   coordinates and hit the viewport directly; keyboard events go to whatever
   widget holds focus.

---

## 5. Portability

Goal: new machine → `git clone` → `docker compose up` → working.

| Rule | Why |
|---|---|
| Everything in git | The machine holds nothing unique |
| **Pin by digest** once stable | Tags get re-pushed upstream; digests don't |
| All state in declared volumes | Nothing important inside a container |
| Machine-specific values in `.env` | The only file that differs per machine |
| Cache volumes persist | Otherwise every restart costs ~10 min |
| Install the Local Assets Pack | Otherwise scenes won't load without internet |

---

## 6. Failure lookup

| Symptom | Cause |
|---|---|
| Connects, black screen | `ISAACSIM_HOST` wrong, or UDP 47998 closed |
| Black screen, `NV_ENC_ERR_UNSUPPORTED_DEVICE` in log | `/dev/nvidia0` missing from the container. `make encoder-check` — see §9.2 |
| Streamed fine, then went black mid-session | Renderer collapse, `vkGetMemoryFdKHR failed`. Cause unknown; restart is the only known fix — §9.3 |
| Right-drag looks around, WASD dead | Trackpad — a two-finger right-click cannot be held. Use a real mouse (§4) |
| Navigation works but view never changes | Stage is empty; an infinite grid looks the same as you slide along it (§4) |
| `ldconfig` says NVENC is fine but streaming is black | `ldconfig` only proves the library is mounted. Only `make encoder-check` opens a session |
| Signaling OK, no video | Bridge networking — `network_mode: host` is mandatory |
| Every restart takes 10 min | Cache volumes not mapped (see compose comments) |
| Host files owned by 1234 | Expected; the 6.x container is rootless |
| `ImportError` on omni.isaac.* | 4.x-era snippet; 6.x uses `isaacsim.*` |
| Package installed but not found | Installed into system python, not `./python.sh` |
| Sensor returns nothing, no error | RTX sensor without its own viewport |
| Crash when rearranging panels | Docking during RTX Lidar sim — build layout, *then* Play |
| Segmentation annotator empty | Missing semantic label |
| **No sensor reacts to you at all** | **Avatar has no collision mesh — the most likely failure of this design** |
| Everything slows after radar | Motion BVH raises cost for *all* sensors |
| Empty/corrupted frames | Multi-GPU — cap at 2, validate against 1 |
| `ERROR_OUT_OF_DEVICE_MEMORY` | Viewport resolution vs. VRAM; restart clean |

---

## 7. Layout

```
config/sensors.yaml   THE registry — every sensor declared once
config/scene.yaml     environments, offsets, avatar, robot poses
core/                 Layers 3–4. NO simulator imports. Runs anywhere.
core/observation.py   the Observation type and the ObservationSource protocol
core/mock_source.py   synthetic source — the whole stack, no GPU, no simulator
sim/                  Layers 1–2. Isaac-specific. Server only.
ext/                  the custom omni.ui inspector panel (build last)
scenes/              saved USD (build artifacts)
tests/               runnable on the MacBook
tests/contract.py     source-agnostic suite; mock and simulator both run it
scripts/             boundary enforcement
```

### 7.1 The observation contract

`tests/contract.py` is written against `ObservationSource` and imports neither
the mock nor the simulator. Subclass it, say how to build a source, and the
whole suite runs:

```python
class TestMockSource(ObservationSourceContract):        # MacBook, no GPU
    def make_source(self): return MockObservationSource.from_config()

class TestIsaacSource(ObservationSourceContract):       # server, needs a stage
    def make_source(self): return IsaacObservationSource(world, registry)
```

The second one is server task S11 and does not exist yet. When it does, a
failure there means the simulator does not satisfy the contract the research
code was written against — which is worth finding out by running a file rather
than by arguing about it.

One test in that suite earns its keep on its own:
`test_a_fixed_sensor_reacts_to_the_moving_avatar`. Against the live simulator it
is the collision-mesh check — the failure that produces no error message
anywhere and that this whole design is most likely to hit.

---

## 8. Day-to-day operation

Everything here runs from the repo root on the **server**. Nothing in this
section touches the host environment.

### 8.1 Start the server

```bash
cd /home/quang/EmbodiedAgent/spatial-sim

make encoder-check     # ~30 s. MUST print:
                       #   OK: every visible GPU can open an NVENC session.
make stream            # holds the terminal; -v shows warm-up progress
```

**Do not skip `encoder-check`.** It is the one command that catches the failure
in §9.2, which is otherwise invisible until a client connects and sees black.

Warm-up before the stream accepts a connection:

| Situation | Time to ready |
|---|---|
| Warm shader cache (normal) | **~270 s** |
| After changing `device_ids`, driver, or image | ~390 s |
| Cold / cache volumes broken | ~10 min, *every single time* |

That third row is a bug, not a cost of doing business — see §6.

### 8.2 Ready gate

From a second terminal:

```bash
make ports
```

**TCP 49100 `LISTEN` means ready.** UDP 47998 stays unbound until a client
actually connects — its absence before you connect is normal and is not a
fault. Only worry if it's still missing *after* connecting.

Scriptable wait:

```bash
until ss -ltn | grep -q ':49100'; do sleep 5; done; echo READY
```

### 8.3 Connect the client

Open the Isaac Sim WebRTC Streaming Client on the Mac, host
**`143.248.57.94`** (LAN, never the tailnet address — §1.6), Connect.

**One client at a time.** Close a session before opening another.

### 8.4 Confirm it is genuinely working

Signaling connecting proves almost nothing — the whole point of §9 is that a
connected client and a black screen look identical. Thirty seconds on the
server settles it:

```bash
nvidia-smi --query-gpu=index,encoder.stats.sessionCount,encoder.stats.averageFps,encoder.stats.averageLatency --format=csv
ss -lun | grep 47998
```

Healthy, measured 2026-08-10: **1 encoder session, 60 fps, ~2600 µs latency on
GPU 0**, and 47998 bound. Zero sessions while a client is connected means the
encoder never started — go to §9.2.

### 8.5 Stop cleanly

```bash
docker stop -t 25 $(docker ps --format '{{.Names}}' | grep '^docker-sim-run-')
```

The container name is hash-suffixed and **changes on every launch**, which is
why the command greps for it rather than hardcoding a name. `Ctrl-C` in the
`make stream` terminal also works.

---

## 9. When the screen is black

Signaling succeeds in *every* case below. "It connected" is not evidence.
Start by answering one question:

> **Did it never show video, or did it show video and then go black?**

### 9.1 Never showed video

```bash
grep ISAACSIM_HOST docker/.env      # must be the LAN IP, not 100.x (§1.6)
make ports                          # 49100 must be LISTEN
ss -lun | grep 47998                # after connecting: must be bound
```

47998 missing *after* a client connects means the media pipeline never
started — which is almost always §9.2.

### 9.2 NVENC cannot open a session

```bash
make encoder-check
```

Any device reporting `UNSUPPORTED_DEVICE` means the container is missing
`/dev/nvidia0` — the node whose **device minor** is 0:

```bash
docker compose -f docker/docker-compose.yml run --rm sim ls /dev/ | grep nvidia
grep -H 'Device Minor' /proc/driver/nvidia/gpus/*/information
```

Fix `device_ids` in `docker/docker-compose.yml` so the card with minor 0 is
among those requested, then rebuild nothing — just relaunch.

Two traps worth knowing, because both cost a full day once:

- **`nvidia-smi` index ≠ device minor.** On this host, index 3 is minor 0.
  Re-derive after any driver or hardware change; do not assume it holds.
- **`ldconfig -p | grep libnvidia-encode` is worthless here.** It passes
  whenever the library is mounted, which is always, and it reported healthy
  through a complete streaming outage. Only opening a session tells the truth.

Upstream: driver bug in 570.x/580.x,
[nvidia-container-toolkit#1249](https://github.com/NVIDIA/nvidia-container-toolkit/issues/1249).
The only published workaround is downgrading to 550.x, which needs root this
server does not have — hence the `device_ids` approach instead.

### 9.3 Was streaming, then went black

```bash
grep -c 'vkGetMemoryFdKHR failed' <your-stream-log>
```

Non-zero means the **renderer** collapsed, not the encoder. Full signature:

```
[Error] [omni.rtx] VkResult: ERROR_INITIALIZATION_FAILED
[Error] [omni.rtx] vkGetMemoryFdKHR failed.
[Error] [omni.rtx] Cannot create shared handle for resource!
[Error] [carb.scenerenderer-rtx.plugin] Failed to allocate 1280x720 LdrColor resource
[Error] [omni.usd.multitick.render] multiTickRateRender() returned null
```

Once it starts it **never recovers** — every subsequent frame fails. The
encoder stays perfectly healthy and keeps shipping packets the whole time,
which is exactly why this is indistinguishable from §9.2 at the client.

**Only known fix: restart the container.** Turning the client off and on
achieves nothing.

Observed once, 2026-08-10: 49 clean minutes, then total failure on every frame.
The client was connected and healthy throughout (so it is not a teardown bug),
no other user's job started nearby, and no leak was visible. **Root cause
unknown.** If it recurs, capture the 30 s of log preceding the first
`vkGetMemoryFdKHR` line at all severities, plus `nvidia-smi` and `ps` at that
instant — a 49-minute clean run followed by instant total failure suggests a
discrete event rather than a leak.

Note: `dmesg` is unreadable on this host (`kernel.dmesg_restrict`, no sudo), so
there is **no Xid visibility**. `nvidia-smi -q -d PAGE_RETIREMENT,ECC,PERFORMANCE`
is the closest available substitute.

### 9.4 Getting a log you can actually read

The default log is a firehose — 336 MB in 90 minutes, and a single decisive
`[Fatal]` line hidden in it. Launch like this instead:

```bash
docker compose -f docker/docker-compose.yml run --rm --service-ports sim \
  ./runheadless.sh -v \
  --/log/channels/omni.kit.livestream.streamsdk=verbose \
  --/log/channels/omni.kit.livestream.webrtc=verbose \
  --/log/channels/omni.kit.livestream.app=verbose \
  --/log/channels/omni.kit.livestream.core=verbose \
  --/log/channels/omni.usd.multitick.render=error \
  > /tmp/stream.log 2>&1
```

That is ~10 MB/hour instead of ~220, and it unmutes the streaming stack that
actually explains black screens. Caveat: it silences `multitick.render` below
error level — the channel adjacent to §9.3 — so if you are chasing *that*, drop
the last line and accept the size.

Greps that pay for themselves:

| Question | Command |
|---|---|
| Did the encoder open? | `grep 'Open encoding session result' log` |
| Did streaming start? | `grep 'Server is now in state: Streaming' log` |
| Did the renderer die? | `grep -c 'vkGetMemoryFdKHR failed' log` |
| Anything fatal? | `grep '\[Fatal\]' log` |
| Is the client sending input? | `grep 'waitForClientCommand' log` |

### 9.5 What is *not* the problem

Time already spent ruling these out, so you don't spend it again:

- **Codec negotiation / AV1.** No codec is ever selected before the failure in
  §9.2, and no carb setting or kit flag to force H.264 exists in 6.x. The
  streaming kit file exposes only `signalPort`, `streamPort`, `publicIp`,
  `allowDynamicResize`, `enableEventTracing`, and `streamType`.
- **The client being Chromium/Electron.** That is the native DMG client's own
  shell (§3.2), not a browser.
- **`Avg Game FPS: 0.00` in the QoS log.** It reads 0.00 in every report
  including verified-healthy 60 fps streaming. Isaac never populates it.
