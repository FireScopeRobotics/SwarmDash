from fastapi import FastAPI, HTTPException, Response
from datetime import datetime
from starlette.middleware.cors import CORSMiddleware
import sqlite3
import uuid

DB_PATH = "/home/ayushg/SwarmDash/dbs/robot.db"
docks = {}
current_session = None

class DockStatuses():
    CLOSED = 0
    GATE_OPEN = 1
    OFFLINE = 2
        

class Dock():
    # request commands
    OPEN_DOOR = "open_door"
    CLOSE_DOOR = "close_door"
    LIGHT_CHANGE = "light_change"
    def __init__(self):
        self.statuses = DockStatuses()
        self.id = str(uuid.uuid4())
        self.requests = []
        self.status =  self.statuses.CLOSED
    def _check_requests(self):
        if (not len(self.requests) == 0):
            return self.requests.pop(0)
        else:
            return None
    def queue_request(self, request):
        self.requests.append(request)
    def set_status(self, status):
        if status == 0:
            self.status = self.statuses.CLOSED
        elif status == 1:
            self.status = self.statuses.GATE_OPEN
    def process_request(self, params):
        command = params[0]
        if command == "heartbeat":
            #reset timer
            request = self._check_requests()
            if request != None:
                return request
            else:
                return None
    def process_heartbeat(self):
        request = self._check_requests()
        if request != None:
            return request
        else:
            return None
            
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["DELETE", "GET", "POST", "PUT"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return "SwarmDash API"

@app.get("/db/initdb")
async def get_initdb() -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute("CREATE TABLE IF NOT EXISTS Mapping(SessionID, Robot NOT NULL, Timestamp DATETIME, Pressure REAL, Temperature REAL, X REAL, Y REAL)")
    cur.execute("CREATE TABLE IF NOT EXISTS Fires(SessionID, Robot NOT NULL, Timestamp DATETIME, X REAL, Y REAL)")
    con.close()
    return {"Result": "Table Created"}

@app.get("/dock/request-init")
async def get_initdock() -> dict:
    dock = Dock()
    docks[dock.id] = dock 
    return Response(dock.id, media_type='text/plain')

@app.get("/dock/{dock_id}/heartbeat")
async def get_heartbeat(dock_id: str) -> dict:
    if dock_id in docks:
        dock = docks[dock_id] 
    else:
        raise HTTPException(status_code=404, detail="Dock ID doesn't exist")
    response = dock.process_heartbeat()
    return Response(response, media_type='text/plain')

@app.get("/dock/{dock_id}/status/{status}")
async def put_status(dock_id: str,status: int) -> dict:
    if dock_id in docks:
        dock = docks[dock_id] 
    else:
        raise HTTPException(status_code=404, detail="Dock ID doesn't exist")
    dock.set_status(status)
    return Response(f"Status {status} registered", media_type='text/plain')

@app.get("/dock/statuses")
async def get_statuses() -> dict:
    statuses = {}
    for k,dock in docks.items():
        statuses[k] = dock.status
    return {"statuses": statuses}


@app.get("/dock/all/{mode}")
async def set_all_mode(mode: str) -> dict:
    for k,dock in docks.items():
        dock.queue_request(mode)
    response = "<body>success</body>"
    return Response(response, media_type='text/html')


@app.get("/dock/{dock_id}/{mode}")
async def set_mode(dock_id: str, mode: str) -> dict:
    if dock_id in docks:
        dock = docks[dock_id] 
    else:
        raise HTTPException(status_code=404, detail="Dock ID doesn't exist")
    dock.queue_request(mode)
    response = "<body>success</body>"
    return Response(response, media_type='text/html')

@app.get("/db/session/get")
async def get_session_now() -> dict:
    return {"Session": current_session}

@app.put("/db/session/set/{session_id}")
async def set_session_now(session_id: int) -> dict:
    global current_session
    current_session = session_id
    return {"Message": "Session set"}

@app.get("/db/session/clear")
async def clear_session_now() -> dict:
    global current_session
    current_session = None
    return {"Message": "Session cleared"}

@app.get("/db/readings/sessions")
async def get_sessions() -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    con.row_factory = lambda cursor, row: row[0]
    cur = con.cursor()
    cur.execute("SELECT DISTINCT SessionID FROM Mapping")
    result = cur.fetchall()
    con.close()
    return {"Sessions": result}

@app.get("/db/readings/{session_id}/{robot_num}")
async def get_readings(session_id: int, robot_num: int) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"SELECT Timestamp, Pressure, Temperature, X, Y FROM Mapping WHERE SessionID={session_id} AND Robot={robot_num} ")
    result = cur.fetchall()
    timestamps = [r[0] for r in result]
    pressures = [r[1] for r in result]
    temperatures = [r[2] for r in result]
    X = [r[3] for r in result]
    Y = [r[4] for r in result]
    con.close()
    return {"Timestamps": timestamps, "Pressures": pressures, "Temperatures": temperatures, "X": X, "Y": Y}

@app.get("/db/pressure/{session_id}/{robot_num}")
async def get_pressure(session_id: int, robot_num: int) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"SELECT Timestamp, Pressure FROM Mapping WHERE SessionID={session_id} AND Robot={robot_num} ")
    result = cur.fetchall()
    timestamps = [r[0] for r in result]
    pressures = [r[1] for r in result]  
    con.close()
    return {"Pressures": pressures,"Timestamps": timestamps}

@app.get("/db/temperature/{session_id}/{robot_num}")
async def get_temperature(session_id: int, robot_num: int) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"SELECT Timestamp, Temperature FROM Mapping WHERE SessionID={session_id} AND Robot={robot_num} ")
    result = cur.fetchall()
    timestamps = [r[0] for r in result]
    temperatures = [r[1] for r in result]
    con.close()
    return {"Temperatures": temperatures,"Timestamps": timestamps}

@app.put("/db/add/{session_id}/{robot_num}")
async def add_robot_entry(session_id: int, robot_num: int, pressure: float, temperature: float, x: float, y: float) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"INSERT INTO Mapping (SessionID, Robot, Timestamp, Pressure, Temperature, X, Y) VALUES ({session_id}, {robot_num}, datetime('now'), {pressure}, {temperature},{x},{y})") 
    con.close()
    return {"message":f"Entry for Robot {robot_num} created for session {session_id}"}

@app.put("/db/add/fire/{session_id}/{robot_num}")
async def add_fire_entry(session_id: int, robot_num: int, x: float, y: float) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"INSERT INTO Fires (SessionID, Robot, Timestamp, X, Y) VALUES ({session_id}, {robot_num}, datetime('now'),{x},{y})") 
    con.close()
    return {"message":f"Fire registered for Robot {robot_num} created for session {session_id}"}

@app.get("/db/fires/{session_id}/{robot_num}")
async def get_fires(session_id: int, robot_num: int) -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute(f"SELECT Timestamp,X, Y FROM Fires WHERE SessionID={session_id} AND Robot={robot_num} ")
    result = cur.fetchall()
    timestamps = [r[0] for r in result]
    X = [r[1] for r in result]
    Y = [r[2] for r in result]
    con.close()
    return {"Timestamps": timestamps, "X": X, "Y": Y}