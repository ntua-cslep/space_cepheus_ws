# ===========================
# ROS1 Docker Environment
# ===========================

# Dynamically detect user/group IDs
UID := $(shell id -u)
GID := $(shell id -g)

# Path to docker compose (multi-file)
COMPOSE := docker compose -f docker/compose.yml -f docker/compose.personal.yml

# Default target (prints help)
.DEFAULT_GOAL := help

# ===========================
# Helper commands
# ===========================

help:
	@echo ""
	@echo "Cepheus Workspace – Makefile Commands"
	@echo "-------------------------------------"
	@echo "make up         : Start ROS docker env (build image if needed, then open shell)"
	@echo "make upfast     : Start ROS docker env without build step (then open shell)"
	@echo "make shell      : Exec into the running ROS container as current user"
	@echo "make root-shell : Exec into the running ROS container as root"
	@echo "make logs       : Follow docker-compose logs for ROS containers"
	@echo "make down       : Stop and remove ROS docker containers"
	@echo "make code       : Open VS Code in the workspace root"
	@echo ""
	@echo "Development Workflow:"
	@echo "  - From terminal: use 'make up' to build (if needed) and start the environment"
	@echo "  - For faster restarts: use 'make upfast' when you know the image is up to date"
	@echo "  - From VS Code: open the repo and use 'Reopen in Container' (same docker-compose setup)"
	@echo ""
	@echo "Notes:"
	@echo "  - The image is rebuilt when 'docker compose up --build' detects changes in Dockerfile/context."
	@echo "  - Your workspace stays on the host; development happens inside the ROS container."
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
# Logs
# ===========================

logs:
	$(COMPOSE) logs -f

# ===========================
# Stop containers
# ===========================

down:
	$(COMPOSE) down

# ===========================
# Start Dev Environment (VS Code)
# ===========================

code:
	@echo "Opening VS Code in devcontainer..."
	code .
