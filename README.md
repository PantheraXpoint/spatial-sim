# spatial-sim — environment setup

Multi-sensor spatial observatory in Isaac Sim. Server renders, MacBook views and
develops CPU-side code.

**Read this first:** every step below has a *verification gate*. Do not advance
past a gate that hasn't passed. Isaac Sim's characteristic failure mode is
silence — wrong configuration produces no error, just no data.

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
nvidia-smi             # must list all 3 RTX 3090s
```

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

> **Gate:** `make verify-gpu` lists all three GPUs.

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

> **Security:** streaming has no authentication and no encryption. Restrict to
> your tailnet. Never a public port.

---

## 2. Server — project setup

```bash
git clone <repo> && cd spatial-sim
cp docker/.env.example docker/.env
$EDITOR docker/.env               # set ISAACSIM_HOST to the server's tailnet IP

make dev-build                    # 5 min, no GPU — catches repo problems early
make check test                   # boundary + tests must pass
make sim-build                    # long: large image pull
make compat                       # expect "System checking result: PASSED"
make encoder-check                # NVENC must be visible, or no streaming
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

Install Tailscale on both machines. Far less painful than SSH tunnels for
streaming ports, and it keeps the stream off the public internet.

### 3.2 Viewing — two options

**Native client (try first):** there *is* a macOS build — download the DMG from
the Isaac Sim Latest Release page and drag it to Applications. Client platform
support is independent of host platform support. Enter the server's tailnet IP,
click Connect. Worth ten minutes on day one; this resolves the brief's open
question about Apple Silicon.

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
sim/                  Layers 1–2. Isaac-specific. Server only.
ext/                  the custom omni.ui inspector panel (build last)
scenes/              saved USD (build artifacts)
tests/               runnable on the MacBook
scripts/             boundary enforcement
```
