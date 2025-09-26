#!/usr/bin/env bash
set -Eeuo pipefail

APP_DIR="/home/cobilsys/Cobilsys_CanGui"
ASSET_URL="${ASSET_URL:-https://github.com/gyus-world/Cobilsys_CanGui/releases/latest/download/linux-arm.tar.gz}"
ASSET_SUM_URL="${ASSET_SUM_URL:-${ASSET_URL}.sha256}"

TMP_DIR="$(mktemp -d)"

echo "[UPDATE] === $(date) : Update start ==="


exec 9>"$APP_DIR/.update.lock"
flock -n 9 || { echo "[UPDATE] Another update is in progress. Exit."; exit 0; }


if command -v systemctl >/dev/null; then
  if systemctl is-active --quiet cobilsys; then
    echo "[UPDATE] Stopping system service cobilsys"
    sudo systemctl stop cobilsys || true
  elif systemctl --user is-active --quiet cobilsys; then
    echo "[UPDATE] Stopping user service cobilsys"
    systemctl --user stop cobilsys || true
  fi
fi

echo "[UPDATE] Downloading: $ASSET_URL"
curl -fL --retry 3 -o "$TMP_DIR/app.tar.gz" "$ASSET_URL"


if curl -fsL "$ASSET_SUM_URL" -o "$TMP_DIR/app.tar.gz.sha256"; then
  echo "[UPDATE] Verifying checksum"
  (cd "$TMP_DIR" && sha256sum -c app.tar.gz.sha256)
else
  echo "[UPDATE] No checksum file; skipping verification"
fi


echo "[UPDATE] Extracting package..."
mkdir -p "$TMP_DIR/stage"
tar -xzf "$TMP_DIR/app.tar.gz" -C "$TMP_DIR/stage" --strip-components=1


NEW_DIR="$APP_DIR/releases/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$NEW_DIR"
rsync -a --delete "$TMP_DIR/stage/" "$NEW_DIR/"

ln -sfn "$NEW_DIR" "$APP_DIR/current"

chmod +x "$APP_DIR/current/Cobilsys_CanGui" "$APP_DIR/current/run.sh" || true


if command -v systemctl >/dev/null; then
  if sudo systemctl status cobilsys >/dev/null 2>&1; then
    echo "[UPDATE] Restart via system service"
    sudo systemctl restart cobilsys || true
  elif systemctl --user status cobilsys >/dev/null 2>&1; then
    echo "[UPDATE] Restart via user service"
    systemctl --user restart cobilsys || true
  else
    echo "[UPDATE] Starting run.sh directly"
    setsid "$APP_DIR/current/run.sh" </dev/null >/dev/null 2>&1 &
  fi
else
  echo "[UPDATE] Starting run.sh directly (no systemd)"
  setsid "$APP_DIR/current/run.sh" </dev/null >/dev/null 2>&1 &
fi
flock -u 9 
echo "Update complete"
exit 0