from fastapi import FastAPI, HTTPException
from datetime import datetime
from starlette.middleware.cors import CORSMiddleware
import sqlite3

DB_PATH = "/swarm-api/dbs/robot.db"
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
async def get_init() -> dict:
    con = sqlite3.connect(DB_PATH, isolation_level=None)
    con.execute('pragma journal_mode=wal')
    cur = con.cursor()
    cur.execute("CREATE TABLE IF NOT EXISTS Mapping(SessionID, Robot NOT NULL, Timestamp DATETIME, Pressure REAL, Temperature REAL, X REAL, Y REAL)")
    con.close()
    return {"Result": "Table Created"}

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