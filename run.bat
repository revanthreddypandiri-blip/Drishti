@echo off
cd /d %~dp0
if not exist .venv python -m venv .venv
call .venv\Scripts\activate
pip install -q -r requirements.txt
echo DRISHTI running at http://localhost:8000
uvicorn backend.main:app --host 0.0.0.0 --port 8000
