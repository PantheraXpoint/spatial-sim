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
	$(COMPOSE) run --rm --service-ports sim ./runheadless.sh -v

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

.PHONY: help dev-build dev test test-local check check-local lint verify ci verify-gpu compat \
        sim-build sim stream encoder-check ports down clean-cache