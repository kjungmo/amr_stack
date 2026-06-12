#!/usr/bin/env bash
# Install the AMR stack to a target machine directory and register the
# systemd unit. Usage: sudo deploy/install.sh [DEST=/opt/amr]
set -euo pipefail
DEST="${1:-/opt/amr}"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$DEST"
rsync -a --exclude .venv --exclude .git --exclude logs "$SRC/" "$DEST/"
python3 -m venv --system-site-packages "$DEST/.venv"
"$DEST/.venv/bin/pip" install --upgrade pip
"$DEST/.venv/bin/pip" install "$DEST"
install -m 644 "$DEST/deploy/amr.service" /etc/systemd/system/amr.service
systemctl daemon-reload
echo "Installed. Enable with: systemctl enable --now amr.service"
