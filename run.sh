#!/usr/bin/env bash
# One-command launcher: creates a venv, installs deps, starts the server.
set -e
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
. .venv/bin/activate
pip install -q -r requirements.txt
echo "DRISHTI running at  http://localhost:8000"
exec uvicorn backend.main:app --host 0.0.0.0 --port 8000
