#!/usr/bin/env bash
# AUV Gorev Kontrol Paneli'ni baslatir.
#   bash ui/run_ui.sh            -> http://localhost:8080  (ve ag uzerinden <jetson-ip>:8080)
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8080 --log-level warning
