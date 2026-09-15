#!/usr/bin/env bash
# Fetch the External Control URCap into ./urcaps/, where docker-compose.yml
# mounts it for URSim to auto-install on boot.
#
# Why needed: External Control is the pendant-side half of ur_robot_driver's
# control channel — the URCap program node connects back to the driver
# (host 192.168.56.1, port 50002) and streams setpoints. The real robot has it
# installed (RUNBOOK session 1); URSim starts bare, so we add it.
set -euo pipefail

VERSION="1.0.5"
URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${VERSION}/externalcontrol-${VERSION}.urcap"
DEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/urcaps"

mkdir -p "$DEST_DIR"
if [ -f "$DEST_DIR/externalcontrol-${VERSION}.urcap" ]; then
    echo "Already present: $DEST_DIR/externalcontrol-${VERSION}.urcap"
    exit 0
fi
echo "Downloading External Control URCap v${VERSION}..."
curl -fL --retry 3 -o "$DEST_DIR/externalcontrol-${VERSION}.urcap" "$URL"
echo "Done: $DEST_DIR/externalcontrol-${VERSION}.urcap"
