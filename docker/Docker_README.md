# 🚀 ROS1 Docker Environment (Pilot Edition)

This folder contains everything related to the **ROS1 Noetic Docker environment** for the *Cepheus* planar robot project.  
The container provides a fully configured workspace with GUI support (RViz, rqt), Starship prompt, Zsh shell, and clean user mapping to your host.

---

## 🧩 Overview

| Component | Purpose |
|------------|----------|
| **Dockerfile** | Builds the main image (`ros1:universal`). |
| **compose.yml** | Launches and manages the container with one command. |
| **.dockerignore** | Excludes large or unnecessary files from build context. |
| **~/.docker_aliases.zsh** | Host file providing container-safe `ls` aliases (no eza). |

---

## 🧱 Directory Structure

```
space_cepheus_ws/
├── src/                  # ROS packages
├── docker/
│   ├── Dockerfile         # Build recipe for ros1:universal
│   ├── compose.yml        # One-command start (with UID/GID mapping)
│   ├── README.md          # (this file)
│   └── .dockerignore      # Exclude build/devel, logs, etc.
└── .dockerignore          # Root-level ignore file (for context)
```

---

## ⚙️ Building the Image

From the repo root:

```bash
cd /data/thesis/space_cepheus_ws
export UID=$(id -u) GID=$(id -g)
docker compose -f docker/compose.yml build
```

This builds the image `ros1:universal` with:
- ROS Noetic (base + rviz + rqt)
- Zsh + Starship prompt
- micro editor
- Host UID/GID mapping → no root-owned files
- Optional Oh My Zsh plugins (if mounted)

---

## 🧠 Running the Container

### 🔹 Quick aliases (add to your host `~/.zshrc`)

```bash
alias rosup='xhost +local:docker && \
  cd /data/thesis/space_cepheus_ws && \
  export UID=$(id -u) GID=$(id -g) && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml up --build -d && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml exec ros1 zsh'

alias rosupfast='xhost +local:docker && \
  cd /data/thesis/space_cepheus_ws && \
  export UID=$(id -u) GID=$(id -g) && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml up -d && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml exec ros1 zsh'

alias rosshell='cd /data/thesis/space_cepheus_ws && \
  export UID=$(id -u) GID=$(id -g) && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml exec ros1 zsh'

alias rosdown='cd /data/thesis/space_cepheus_ws && \
  docker compose -f docker/compose.yml -f docker/compose.personal.yml down'
```

Then:
```bash
rosup     # build & start container
rosshell  # new shell
rosdown   # stop container
```

---

## 🖥️ GUI Support (RViz, rqt)

The container connects to your host’s X11 server automatically.

```bash
xhost +local:docker
```

Then launch any GUI tool inside the container:
```bash
rviz
rqt
roscore
```

---

## 🧩 Workspace Mounts

| Host Path | Container Path | Purpose |
|------------|----------------|----------|
| `~/space_cepheus_ws` | `/home/pilot/space_cepheus_ws` | Main catkin workspace |
| `~/.config/starship.toml` | `/home/pilot/.config/starship.toml` | Starship prompt config |
| `~/.docker_aliases.zsh` | `/home/pilot/.zshrc.d/10-aliases.zsh` | Container-only aliases |
| *(optional)* `~/.oh-my-zsh` | `/home/pilot/.oh-my-zsh` | Shared plugin framework |

> ✅ `.zshrc` is **not** mounted — host and container have separate shell configurations.

---

## 🧭 Shell Features

- **Zsh + Starship** prompt  
- **micro** editor (fast, terminal-based)  
- **colorful `ls` aliases** (no `eza` dependency)  
- **autoloads from `/home/pilot/.zshrc.d/*.zsh`**

### Default Aliases (inside container)



## 🧹 Maintenance Commands

| Action | Command |
|---------|----------|
| Rebuild image | `docker compose -f docker/compose.yml build --no-cache` |
| Stop all containers | `docker compose down` |
| List active containers | `docker ps` |
| Shell into running container | `docker compose exec ros1 zsh` |
| Clean old images | `docker image prune -f` |

---

## 🧩 Notes

- Built for **Ubuntu 20.04 base** with ROS Noetic.  
- Uses **user mapping (UID/GID)** so files inside `/home/pilot/space_cepheus_ws` belong to your host user.  
- Container and host have **different aliases** (`eza` on host, plain `ls` inside Docker).  
- `.dockerignore` prevents unnecessary rebuilds from large files (`build/`, `devel/`, `rosbags/`, etc.).

---

## 🧭 Example Lifecycle

```bash
rosup        # build + start + open shell
roscore      # start ROS core
rosrun pkg node
rosdown      # stop container
```

---

**Maintainer:** `cmdr`  
**Image tag:** `ros1:universal`  
**Compose service:** `ros1`

Enjoy your streamlined, reproducible ROS1 dev environment 🛰️

---

## 🧩 Personal shell customization (optional, untracked)

If you want to load your **own** dotfiles (Starship config, Oh My Zsh, etc.) inside the container
without forcing them on everyone else, create a local override file:

```bash
cp docker/compose.yml docker/compose.personal.yml
```

Edit `docker/compose.personal.yml` and add your mounts, for example:

```yaml
services:
  ros1:
    volumes:
      - ${HOME}/.config/starship.toml:/home/pilot/.config/starship.toml:ro
      - ${HOME}/.oh-my-zsh:/home/pilot/.oh-my-zsh:ro
```

This file is:

- **Ignored by Git** via `.gitignore`
- **Ignored by Docker builds** via `.dockerignore`
- Only used when you explicitly include it:

```bash
export UID=$(id -u) GID=$(id -g)
docker compose -f docker/compose.yml -f docker/compose.personal.yml up -d
docker compose -f docker/compose.yml -f docker/compose.personal.yml exec ros1 zsh
```

This keeps the shared image and repo clean, while still letting you use your own shell setup.

