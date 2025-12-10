# ===========================
# ROS1 Docker Environment
# ===========================

# Dynamically detect user/group IDs
UID := $(shell id -u)
GID := $(shell id -g)

# Path to docker compose (multi-file)
COMPOSE := docker compose -f docker/compose.yml -f docker/compose.personal.yml

# ===========================
# Helper commands
# ===========================

# Default target (prints help)
help:
	@echo ""
	@echo "Available commands:"
	@echo "  make up        - Build + start + open shell"
	@echo "  make upfast    - Start without rebuilding + open shell"
	@echo "  make shell     - Open interactive shell in container"
	@echo "  make down      - Stop and remove containers"
	@echo ""

# ===========================
# Start + build + shell
# ===========================

up:
	xhost +local:docker
	UID=$(UID) GID=$(GID) $(COMPOSE) up --build -d
	$(COMPOSE) exec ros1 zsh

# Start without rebuilding
upfast:
	xhost +local:docker
	UID=$(UID) GID=$(GID) $(COMPOSE) up -d
	$(COMPOSE) exec ros1 zsh

# ===========================
# Shell only
# ===========================

shell:
	UID=$(UID) GID=$(GID) $(COMPOSE) exec ros1 zsh

# ===========================
# Root Shell
# ===========================


root-shell:
	$(COMPOSE) exec --user root ros1 bash


# ===========================
# Stop containers
# ===========================

down:
	$(COMPOSE) down

