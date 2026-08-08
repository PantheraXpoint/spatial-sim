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

test-local: ## Run tests on the host (needs local pyyaml + pytest)
	PYTHONPATH=. python3 -m pytest tests/ -v

check: ## Enforce the core/ layer boundary
	./scripts/check_layer_boundary.sh

lint:
	$(COMPOSE) run --rm dev ruff check core/ tests/

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

encoder-check: ## Confirm NVENC is visible inside the container
	$(COMPOSE) run --rm sim bash -c "ldconfig -p | grep libnvidia-encode"

ports: ## Show whether the streaming ports are listening
	@echo "TCP 49100 (signaling):"; ss -ltnp 2>/dev/null | grep 49100 || echo "  NOT LISTENING"
	@echo "UDP 47998 (media):";     ss -lunp 2>/dev/null | grep 47998 || echo "  NOT LISTENING"

down:
	$(COMPOSE) down

clean-cache: ## Nuke Isaac cache volumes. Next start costs ~10 min.
	$(COMPOSE) down -v

.PHONY: help dev-build dev test test-local check lint ci verify-gpu compat \
        sim-build sim stream encoder-check ports down clean-cache
