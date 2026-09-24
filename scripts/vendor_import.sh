#!/usr/bin/env bash
# Fetch the third-party packages Phase 1 builds on (ur7e.repos, D6) into
# src/vendor/. Idempotent — safe to re-run after a pin bump.
#
# Run this before `rosdep install` / `colcon build`, same place it's wired
# into .devcontainer/devcontainer.json's postCreateCommand and
# .github/workflows/ci.yml.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v vcs >/dev/null 2>&1; then
    echo "vcs (python3-vcstool) not found — install it first: apt-get install python3-vcstool" >&2
    exit 1
fi

mkdir -p src
vcs import src < ur7e.repos

# OnRobot_ROS2_Driver's Modbus RTU/TCP library is a git submodule, not a vcs
# repo of its own (tonydle/OnRobot_ROS2_Driver's .gitmodules) — vcs import
# doesn't follow submodules, so pull it explicitly.
driver_dir="src/vendor/OnRobot_ROS2_Driver"
if [ -d "$driver_dir" ]; then
    git -C "$driver_dir" submodule update --init --recursive
fi

echo "Vendor import complete: $(vcs list src/vendor 2>/dev/null | wc -l) package(s) in src/vendor/"
