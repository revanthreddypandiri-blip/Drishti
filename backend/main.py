"""FastAPI + WebSocket server. Serves the 4-page frontend (Home, Explain, Math,
Simulation) and streams live simulation state over a WebSocket.
    uvicorn backend.main:app --port 8000     ->  http://localhost:8000
"""
from __future__ import annotations
import asyncio
import json
import os
import random
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from sim.scenarios import build, list_scenarios
from sim.simulation import Simulation

app = FastAPI(title="Drishti Simulation Server")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")


def _page(name):
    return FileResponse(os.path.join(FRONTEND_DIR, name))


@app.get("/")
def home(): return _page("home.html")

@app.get("/explain")
def explain(): return _page("explain.html")

@app.get("/math")
def math_page(): return _page("math.html")

@app.get("/simulation")
def simulation_page(): return _page("simulation.html")

@app.get("/api/scenarios")
def get_scenarios():
    return list_scenarios()


class Session:
    def __init__(self):
        self.sim: Simulation = None
        self.speed_multiplier = 1.0
        self.current_key = "village_road"
        self.seed = 42
        self.load_scenario(self.current_key, self.seed)

    def load_scenario(self, key: str, seed: int):
        self.current_key = key
        self.seed = int(seed)
        scene, agents = build(key, seed=self.seed)
        self.sim = Simulation(scene, agents, dt=0.1, seed=self.seed)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    session = Session()

    async def receiver():
        while True:
            try:
                raw = await ws.receive_text()
            except (WebSocketDisconnect, RuntimeError):
                return
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            a = msg.get("action")
            if a == "load_scenario":
                seed = msg.get("seed")
                session.load_scenario(msg.get("scenario", session.current_key),
                                      seed if seed is not None else session.seed)
            elif a == "randomize_seed":
                session.load_scenario(session.current_key, random.randrange(1, 999999))
            elif a == "pause":
                session.sim.paused = True
            elif a == "resume":
                session.sim.paused = False
            elif a == "reset":
                session.load_scenario(session.current_key, session.seed)
            elif a == "toggle_background_ai":
                session.sim.background_ai_enabled = not session.sim.background_ai_enabled
            elif a == "set_speed":
                session.speed_multiplier = max(0.1, min(5.0, float(msg.get("value", 1.0))))
            elif a == "whatif":
                session.sim.trigger_whatif()

    recv = asyncio.create_task(receiver())
    try:
        while True:
            session.sim.step()
            try:
                await ws.send_text(json.dumps(session.sim.snapshot()))
            except Exception:
                break
            await asyncio.sleep(0.1 / max(session.speed_multiplier, 0.05))
    except WebSocketDisconnect:
        pass
    finally:
        recv.cancel()


app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")
