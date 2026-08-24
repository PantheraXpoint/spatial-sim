# =============================================================================
# Targets that run on the MACBOOK: dev-build, test, lint, check
# Targets that run on the SERVER:  sim-build, sim, stream, verify-gpu
# =============================================================================
COMPOSE := docker compose -f docker/docker-compose.yml
.DEFAULT_GOAL := help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

# --- Both machines -----------------------------------------------------------
dev-build: ## Build the CPU dev image (5 min, no GPU). Do this FIRST.
	$(COMPOSE) build dev

dev: ## Shell into the dev container
	$(COMPOSE) run --rm dev bash

test: ## Run tests in the dev container
	$(COMPOSE) run --rm dev python -m pytest tests/ -v

# ESCAPE HATCHES -- debugging only, when the container itself is suspect.
# These run on the host and therefore need host packages, which hard rule 7
# forbids installing. Do not use them to "get around" a broken image: fix the
# image. Present so that a container problem is diagnosable, nothing more.
test-local: ## [debug only] Tests on the host. Requires host deps -- see rule 7.
	PYTHONPATH=. python3 -m pytest tests/ -v

check: ## Enforce the core/ layer boundary (in container)
	$(COMPOSE) run --rm dev bash scripts/check_layer_boundary.sh

check-local: ## [debug only] Boundary check on the host. See note above.
	bash scripts/check_layer_boundary.sh

lint:
	$(COMPOSE) run --rm dev ruff check core/ tests/

verify: check test ## Run before every commit. Fully containerized.

ci: check test lint ## Everything CI runs

# --- Server only -------------------------------------------------------------
verify-gpu: ## Confirm Docker sees all three GPUs before anything else
	docker run --rm --runtime=nvidia --gpus all \
		nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi

compat: ## Isaac Sim compatibility check -- catches most failures early
	$(COMPOSE) run --rm sim ./isaac-sim.compatibility_check.sh \
		--/app/quitAfter=10 --no-window

sim-build: ## Build the Isaac Sim image (long -- large pull)
	$(COMPOSE) build sim

sim: ## Interactive shell in the sim container
	$(COMPOSE) run --rm --service-ports sim bash

stream: ## Launch headless streaming. -v shows shader warm-up progress.
	@# --enable omni.physx.cct: the avatar's controls are an OmniGraph node
	@# from that extension, and NO shipped .kit experience enables it
	@# (checked across /isaac-sim/apps/*.kit). Without this flag the node type
	@# is unregistered when the stage opens, the graph loads "fine", and
	@# pressing Play moves nothing -- with no error that names the cause.
	$(COMPOSE) run --rm --service-ports sim ./runheadless.sh -v \
		--enable omni.physx.cct

# CPU worker threads for the gui session. Sweep them without editing this file:
#   PHYS_THREADS=8 KIT_THREADS=16 make gui
#
# Isaac takes the whole machine by default -- carb.tasking documents
# threadCount 0 as carb::thread::hardware_concurrency(), and PhysX's "Num
# Simulation Threads" preference is 0 (auto) out of the box. On a 64-thread
# EPYC 7543 that is 64 workers, and NVIDIA's performance handbook warns that
# spawning too many worker threads may lead to CPU bottlenecking, calling 32
# threads optimal for most use cases.
#
# Worth having because physics is where the frame rate goes. Measured
# 2026-08-24 on an idle host (load 0.68): no lidar, lidar-without-draw and
# lidar-with-draw all land at 14-20 fps at Play against 30-50 stopped, so the
# lidar is not the cost -- physics is 100% of it.
#
# This is a SCHEDULING change and does not alter physics results: the same
# scene simulates the same way on 16 threads as on 64, only slower or faster.
# That is the property that matters here, because this project is building
# benchmarks and a knob that changed the physics would invalidate every number
# measured on either side of it.
PHYS_THREADS ?= 16
KIT_THREADS  ?= 32

gui: ## Stream the observatory with every sensor built and every panel bound.
	@# Same launcher as `stream`, plus sim/gui_viewports.py, which opens
	@# observatory_avatar.usd, authors the stations from config/scene.yaml,
	@# creates the registry's sensors and binds one viewport per camera. It
	@# does NOT press Play. Dock the panels, save the layout, THEN Play --
	@# never re-dock while an RTX lidar sim is running.
	@#
	@# Thread flags go BEFORE --exec: Kit reads everything after the script
	@# path as arguments to the script, so a kit flag placed after it is
	@# swallowed silently. See PHYS_THREADS/KIT_THREADS above.
	$(COMPOSE) run --rm --service-ports sim ./runheadless.sh -v \
		--enable omni.physx.cct \
		--/persistent/physics/numThreads=$(PHYS_THREADS) \
		--/plugins/carb.tasking.plugin/threadCount=$(KIT_THREADS) \
		--exec /workspace/sim/gui_viewports.py

encoder-check: ## Confirm NVENC can actually OPEN A SESSION. Run before make stream.
	@# Not `ldconfig -p | grep libnvidia-encode` -- that passes whenever the
	@# library is mounted, which is always, and it reported healthy through a
	@# complete livestream outage. Only opening a session tells the truth.
	$(COMPOSE) run --rm sim /isaac-sim/kit/python/bin/python3 \
		/workspace/scripts/nvenc_probe.py

ports: ## Show whether the streaming ports are listening
	@echo "TCP 49100 (signaling):"; ss -ltnp 2>/dev/null | grep 49100 || echo "  NOT LISTENING"
	@echo "UDP 47998 (media):";     ss -lunp 2>/dev/null | grep 47998 || echo "  NOT LISTENING"

down:
	$(COMPOSE) down

clean-cache: ## Nuke Isaac cache volumes. Next start costs ~10 min.
	$(COMPOSE) down -v

.PHONY: help dev-build dev test test-local check check-local lint verify ci verify-gpu compat gui \
        sim-build sim stream encoder-check ports down clean-cache