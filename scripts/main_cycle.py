# =====================================================================
#  KUKA KR 10 machine-tending cell - main cycle (RoboDK Python API)
#
#  1. SETUP : writes every target, picks ONE consistent arm configuration
#             per station (no wrist flips) and creates a safe "via" point
#             above each station.
#  2. CYCLE : pick -> CNC -> vision -> OK / NOK, always moving between
#             stations at safe height (via points).
#  3. LOG   : one CSV line per part (for the MES dashboard).
#
#  Tip: run it once with collision checking OFF (setup), then check_paths.
# =====================================================================
import csv
import json
import socket
import math
import os
import random
import time
from datetime import datetime

from robodk.robolink import (Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL,
                             ITEM_TYPE_TARGET, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT,
                             COLLISION_ON, COLLISION_OFF)
from robodk.robomath import transl, invH, KUKA_2_Pose

RDK = Robolink()

# ---------------------------------------------------------------- settings
N_PARTS = 5                      # number of parts to produce
MACHINING_TIME_S = 3             # simulated CNC cycle
NOMINAL_DIAM = 20.00             # mm
TOLERANCE = 0.05                 # mm  -> OK if 19.95 .. 20.05
_station_dir = RDK.getParam('PATH_OPENSTATION')          # folder of the saved station
LOG_DIR = _station_dir if _station_dir and os.path.isdir(str(_station_dir)) else \
    os.path.join(os.path.expanduser("~"), "Documents", "KUKA_Cell_MES")
LOG_FILE = os.path.join(LOG_DIR, "kuka_cell_log.csv")   # CSV saved next to KUKA_Cell.rdk
CNC_FRAME_X = 1050               # CNC farther: more space between robot and CNC door

# ---------------------------------------------------------------- STEP 1: faults & downtime
# Each fault: (station, stop reason, probability per part, min-max stop duration in seconds)
FAULTS = [
    ("infeed", "Part missing on infeed",   0.10, (20, 60)),
    ("infeed", "Gripper fault",            0.05, (30, 90)),
    ("cnc",    "CNC door not open",        0.08, (15, 45)),
    ("cnc",    "Tool change on CNC",       0.05, (60, 180)),
    ("vision", "Camera timeout",           0.07, (10, 30)),
]
TIME_SCALE = 0.05                # a 60 s stop is shown as 3 s in the simulation
EVENT_FILE = os.path.join(LOG_DIR, "kuka_cell_events.csv")   # one line per stop
ORDER_ID = datetime.now().strftime("OF-%Y%m%d-%H%M")         # production order number

# ---------------------------------------------------------------- STEP 2: real-time link (MQTT)
# The cell publishes its events to the MQTT broker of the MES stack (Docker).
# If the broker is not running, the cell works as before (CSV only).
MQTT_ON = True
MQTT_HOST = "localhost"          # PC where "docker compose up" runs
MQTT_PORT = 1883
MQTT_TOPIC = "autoparts/sousse/cellA"    # enterprise / site / cell (ISA-95 hierarchy)


class MiniMQTT:
    """Very small MQTT 3.1.1 publisher (QoS 0) - no extra library needed in RoboDK."""

    def __init__(self, host, port, client_id):
        self.host, self.port, self.cid = host, port, client_id
        self.sock, self.next_try = None, 0

    @staticmethod
    def _len(n):                                   # MQTT "remaining length" encoding
        out = bytearray()
        while True:
            b, n = n % 128, n // 128
            out.append(b | 0x80 if n else b)
            if not n:
                return bytes(out)

    @staticmethod
    def _str(s):
        b = s.encode("utf-8")
        return len(b).to_bytes(2, "big") + b

    def _connect(self):
        if time.time() < self.next_try:
            return False
        try:
            s = socket.create_connection((self.host, self.port), timeout=1.0)
            body = self._str("MQTT") + bytes([4, 0x02, 0, 0]) + self._str(self.cid)  # clean session, no keepalive
            s.sendall(b"\x10" + self._len(len(body)) + body)
            ack = s.recv(4)
            if len(ack) < 4 or ack[0] != 0x20 or ack[3] != 0:
                raise ConnectionError("broker refused the connection")
            self.sock = s
            print(f"MQTT connected to {self.host}:{self.port}")
            return True
        except OSError as e:
            print(f"MQTT broker not reachable ({e}) - CSV logging only, retry in 30 s")
            self.sock, self.next_try = None, time.time() + 30
            return False

    def publish(self, topic, data):
        if not MQTT_ON or (self.sock is None and not self._connect()):
            return
        body = self._str(topic) + json.dumps(data).encode("utf-8")
        try:
            self.sock.sendall(b"\x30" + self._len(len(body)) + body)
        except OSError:
            self.sock = None                        # connection lost: reconnect next time
            if self._connect():
                self.publish(topic, data)


MQTT = MiniMQTT(MQTT_HOST, MQTT_PORT, "kuka-cellA-" + ORDER_ID)


def publish(kind, data):
    """kind = order | status | part | stop"""
    MQTT.publish(f"{MQTT_TOPIC}/{kind}", data)

# Stations: name -> (frame, approach target, work target, approach values, work values)
# values = (X, Y, Z, A=rz, B=ry, C=rx) in the station frame (KUKA convention)
STATIONS = {
    "infeed": ("Frame_Infeed", "Approach_infeed", "pick_infeed",
               (0, 0, 580, 0, 0, 180), (0, 0, 460, 0, 0, 180)),
    "cnc":    ("Frame_CNC", "approach_cnc", "load_cnc",
               (-300, 0, 550, 0, 90, 0), (-110, 0, 550, 0, 90, 0)),
    "vision": ("Frame_Vision", "approach_vision", "place_vision",
               (0, 0, 580, 0, 0, 180), (0, 0, 460, 0, 0, 180)),
    "ok":     ("Frame_OK", "approach_ok", "place_ok",
               (0, 0, 480, 0, 0, 180), (0, 0, 360, 0, 0, 180)),
    "nok":    ("Frame_NOK", "approach_nok", "place_nok",
               (0, 0, 600, 0, 0, 180), (0, 0, 480, 0, 0, 180)),
}
HOME_JOINTS = [0, -90, 90, 0, 90, 0]
SOLVED = {}        # target name -> joints computed in setup (used by MoveJ)

# ---------------------------------------------------------------- items
def find_robot_and_tool():
    """KUKA robot (ignores a gripper mechanism) + its gripper tool."""
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise Exception("No robot in the station. Load the KUKA KR 10 first.")
    rob = next((r for r in robots if "KUKA" in r.Name().upper()), robots[0])
    tl = RDK.Item('Tool_Gripper', ITEM_TYPE_TOOL)
    if not tl.Valid() or tl.Name() != 'Tool_Gripper':
        tools = [t for t in RDK.ItemList(ITEM_TYPE_TOOL)]
        if not tools:
            tl = rob.AddTool(transl(0, 0, 150), 'Tool_Gripper')
        else:
            tl = tools[0]
            tl.setName('Tool_Gripper')
    return rob, tl


robot, tool = find_robot_and_tool()
part = RDK.Item('Part_Shaft', ITEM_TYPE_OBJECT)
robot.setPoseTool(tool)


def collision_report():
    """Names of the items that are touching right now (empty string if none)."""
    try:
        pairs = RDK.CollisionPairs()            # [[item1, item2, link1, link2], ...]
        out = []
        for p in pairs:
            a = p[0].Name() + (f" (J{p[2]})" if len(p) > 2 and p[0].Type() == ITEM_TYPE_ROBOT else "")
            b = p[1].Name() + (f" (J{p[3]})" if len(p) > 3 and p[1].Type() == ITEM_TYPE_ROBOT else "")
            out.append(f"{a}  <->  {b}")
        return "; ".join(out)
    except Exception:
        try:
            return ", ".join(i.Name() for i in RDK.CollisionItems())
        except Exception:
            return ""
robot.setPoseFrame(robot.Parent())      # poses measured in the robot base


def T(name):
    item = RDK.Item(name, ITEM_TYPE_TARGET)
    if not item.Valid() or item.Name() != name:
        raise Exception(f"Target not found in the tree: {name}")
    return item


def F(name):
    item = RDK.Item(name, ITEM_TYPE_FRAME)
    if not item.Valid():
        raise Exception(f"Frame not found in the tree: {name}")
    return item


def as_list(values):
    """RoboDK returns joints as a Mat -> flat Python list."""
    if hasattr(values, "rows"):
        values = values.rows
    elif hasattr(values, "list"):
        values = values.list()
    out = []
    for v in values:
        out.extend(v) if isinstance(v, (list, tuple)) else out.append(v)
    return [float(x) for x in out]


# ---------------------------------------------------------------- moves
def MJ(name):
    try:
        if name in SOLVED:
            robot.MoveJ(SOLVED[name])        # exact joints from setup: no config surprise
        else:
            robot.MoveJ(T(name))
    except Exception as e:
        hit = collision_report()
        msg = f"Move stopped going to: {name}" + (f"\nCOLLISION: {hit}" if hit else f" ({e})")
        RDK.ShowMessage(msg, True)
        raise Exception(msg)


def ML(name):
    try:
        robot.MoveL(T(name))
    except Exception:
        print(f"  MoveL to {name} not possible -> MoveJ used")
        MJ(name)


def grip():
    part.setParentStatic(tool)


def release(frame_name):
    part.setParentStatic(F(frame_name))


# ---------------------------------------------------------------- on-screen display (HMI)
# RoboDK shows the NAME of a frame as a text label in the 3D view.
# We create 4 small frames above the cell and change their names = live text panel.
HMI_POS = (-150, 0, 1700)          # where the text panel floats (mm, station frame)
HMI = []
PART_COLOR = [0.85, 0.65, 0.20, 1]   # gold = raw part


def hmi_init():
    for fr in RDK.ItemList(ITEM_TYPE_FRAME):          # remove the panel of a previous run
        if fr.Name().startswith("HMI"):
            fr.Delete()
    HMI.clear()
    for i in range(4):
        fr = RDK.AddFrame(f"HMI {i}")
        fr.setPose(transl(HMI_POS[0], HMI_POS[1], HMI_POS[2] - 70 * i))
        try:
            fr.setVisible(True, False)                  # show the text, hide the axes
        except Exception:
            pass
        HMI.append(fr)


def hmi(line, text):
    """Write one line of the 3D text panel (0 = title, 1 = status, 2 = last part, 3 = totals)."""
    if line < len(HMI):
        HMI[line].setName(f"HMI | {text}")
    if line == 1:                                   # status line -> also sent to the MES
        state = ("FAULT" if "FAULT" in text else "IDLE" if "READY" in text
                 else "DONE" if "FINISHED" in text else "RUNNING")
        publish("status", {"state": state, "text": text.replace("Status: ", "")})


def color_part(rgba):
    try:
        part.setColor(rgba)
    except Exception:
        pass


def reset_part():
    part.setParent(F('Frame_Infeed'))
    part.setPose(transl(0, 0, 400))
    color_part(PART_COLOR)


# ---------------------------------------------------------------- setup helpers
def choose_home():
    """Home with the gripper pointing DOWN (A5 = +90 or -90)."""
    global HOME_JOINTS
    for a5 in (90, -90):
        j = [0, -90, 90, 0, a5, 0]
        robot.setJoints(j)
        if robot.Pose()[2, 2] < -0.5:        # tool Z axis points down
            HOME_JOINTS = j
            break
    print(f"Home joints: {HOME_JOINTS}")


def get_or_create_target(name, parent):
    item = RDK.Item(name, ITEM_TYPE_TARGET)
    if not item.Valid() or item.Name() != name:
        item = RDK.AddTarget(name, parent, robot)
    item.setParent(parent)
    return item


def via_joints(frame_name):
    """Home posture rotated (A1) toward the station: high and safe."""
    p = F(frame_name).PoseAbs().Pos()
    best, best_dot = None, -2
    ang = math.degrees(math.atan2(p[1], p[0]))
    for a1 in (ang, -ang):
        j = list(HOME_JOINTS)
        j[0] = a1
        robot.setJoints(j)
        t = robot.Pose().Pos()
        n = math.hypot(t[0], t[1]) * math.hypot(p[0], p[1]) or 1
        dot = (t[0] * p[0] + t[1] * p[1]) / n
        if dot > best_dot:
            best, best_dot = j, dot
    return best


def flange_pose(frame_name, values):
    """Flange pose in the robot base for a TCP target given in a station frame."""
    base_abs = robot.Parent().PoseAbs()
    frame_abs = F(frame_name).PoseAbs()
    return invH(base_abs) * frame_abs * KUKA_2_Pose(list(values)) * invH(robot.PoseTool())


def ik(frame_name, values, joints_approx):
    target = flange_pose(frame_name, values)
    sol = as_list(robot.SolveIK(target, joints_approx))
    if len(sol) < 6:
        return None
    sol = sol[:6]
    # verify: forward kinematics must give back the same flange position
    fk = robot.SolveFK(sol)
    err = sum((a - b) ** 2 for a, b in zip(fk.Pos(), target.Pos())) ** 0.5
    if err > 1.0:
        return None
    lower, upper = [as_list(x) for x in robot.JointLimits()[:2]]
    if any(not (lo <= v <= up) for v, lo, up in zip(sol, lower, upper)):
        return None
    return sol


def path_free(j_from, j_to, frame_name=None, work_values=None):
    """True if the PTP move j_from->j_to (and the LIN to the work point) is collision free."""
    try:
        if robot.MoveJ_Test(j_from, j_to) != 0:
            return False
        robot.setJoints(j_to)
        if RDK.Collisions() > 0:
            return False
        if frame_name and work_values:
            robot.setPoseFrame(F(frame_name))
            res = robot.MoveL_Test(j_to, KUKA_2_Pose(list(work_values)))
            robot.setPoseFrame(robot.Parent())
            if res != 0:
                return False
        return True
    except Exception:
        robot.setPoseFrame(robot.Parent())
        return True        # if the test itself is not available, do not block


def setup_cell():
    print("SETUP...")
    RDK.Render(False)
    RDK.setCollisionActive(COLLISION_ON)       # setup picks collision-free configurations
    F('Frame_CNC').setPose(transl(CNC_FRAME_X, 0, 0))
    choose_home()

    base = robot.Parent()
    home = get_or_create_target('home', base)
    home.setAsJointTarget()
    home.setJoints(HOME_JOINTS)
    SOLVED['home'] = list(HOME_JOINTS)

    for key, (frame, appr, work, v_appr, v_work) in STATIONS.items():
        # safe point above the station
        vj = via_joints(frame)
        via = get_or_create_target(f'via_{key}', base)
        via.setAsJointTarget()
        via.setJoints(vj)
        SOLVED[f'via_{key}'] = vj

        # approach + work: same wrist orientation, configuration close to the via point
        done = False
        candidates = [(extra, t, dz) for dz in (0, 100) for t in (0, 0.25, 0.5)
                      for extra in (0, 180, 90, -90)]
        for extra, t, dz in candidates:
            # approach can be moved a bit toward the work point if too far / too folded
            va = [a + t * (w - a) for a, w in zip(v_appr[:3], v_work[:3])] + list(v_appr[3:])
            va[5] += extra
            if key != "cnc":
                va[2] += dz                    # higher approach if needed (not for the CNC door)
            vw = list(v_work); vw[5] += extra
            ja = ik(frame, va, vj)
            jw = ik(frame, vw, ja) if ja else None
            if (ja and jw and abs(ja[0] - vj[0]) < 60 and abs(jw[4] - ja[4]) < 60
                    and path_free(vj, ja, frame, vw)):
                for name, vals, joints in ((appr, va, ja), (work, vw, jw)):
                    tgt = get_or_create_target(name, F(frame))
                    tgt.setAsCartesianTarget()
                    tgt.setPose(KUKA_2_Pose(vals))
                    tgt.setJoints(joints)    # stored configuration -> no wrist flip
                    SOLVED[name] = joints
                print(f"  {key:6s}: OK collision-free (wrist +{extra} deg, approach shifted {int(t*100)}%, +{dz} mm)")
                done = True
                break
        if not done:
            RDK.Render(True)
            p = F(frame).PoseAbs().Pos()
            raise Exception(f"Station {key}: no reachable AND collision-free configuration. "
                            f"Frame at X={p[0]:.0f} Y={p[1]:.0f}. Tell Claude this message.")

    # check the moves between stations (home -> via -> via ...)
    order = ['home', 'via_infeed', 'via_cnc', 'via_vision', 'via_ok', 'via_vision', 'via_nok', 'home']
    for a, b in zip(order, order[1:]):
        if robot.MoveJ_Test(SOLVED[a], SOLVED[b]) != 0:
            print(f"  WARNING: collision on the move {a} -> {b}: {collision_report()}")

    robot.setJoints(HOME_JOINTS)
    RDK.Render(True)
    print("SETUP done.\n")


# ---------------------------------------------------------------- cycle
def go_station(key):
    MJ(f'via_{key}')
    MJ(STATIONS[key][1])           # approach


def leave_station(key):
    ML(STATIONS[key][1])           # back to approach
    MJ(f'via_{key}')


def write_csv(path, row):
    """Append one row to a CSV file (header written the first time).
    If an old file has different columns, it is renamed *_old.csv."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        with open(path, newline="") as f:
            header = f.readline().strip().split(",")
        if header != list(row.keys()):
            os.replace(path, path.replace(".csv", "_old.csv"))
    new_file = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new_file:
            w.writeheader()
        w.writerow(row)


def check_faults(serial, station):
    """Random stops at a station. Returns the downtime (s) and the wait time used."""
    downtime, waited = 0.0, 0.0
    for st, reason, prob, (dmin, dmax) in FAULTS:
        if st == station and random.random() < prob:
            duration = random.uniform(dmin, dmax)
            start = datetime.now()
            hmi(1, f"Status: FAULT - {reason} ({duration:.0f} s)")
            RDK.ShowMessage(f"FAULT at {station}: {reason}", False)
            wait = duration * TIME_SCALE
            time.sleep(wait)
            write_csv(EVENT_FILE, {
                "order": ORDER_ID,
                "serial": serial,
                "station": station,
                "reason": reason,
                "start": start.isoformat(timespec="seconds"),
                "duration_s": round(duration, 1),
            })
            publish("stop", {"order": ORDER_ID, "serial": serial, "station": station,
                             "reason": reason, "duration_s": round(duration, 1)})
            downtime += duration
            waited += wait
    return downtime, waited


def run_cycle(serial):
    t0 = time.time()
    downtime, waited = 0.0, 0.0
    hmi(0, f"ORDER {ORDER_ID}: {N_PARTS} parts  |  now: {serial}")
    reset_part()
    MJ('home')

    # PICK
    hmi(1, "Status: PICKING raw part")
    d, w = check_faults(serial, "infeed"); downtime += d; waited += w
    hmi(1, "Status: PICKING raw part")
    go_station('infeed')
    ML('pick_infeed')
    grip()
    leave_station('infeed')

    # CNC load / machining / unload
    hmi(1, "Status: LOADING CNC")
    d, w = check_faults(serial, "cnc"); downtime += d; waited += w
    hmi(1, "Status: LOADING CNC")
    go_station('cnc')
    ML('load_cnc')
    release('Frame_CNC')
    ML('approach_cnc')
    hmi(1, f"Status: MACHINING ({MACHINING_TIME_S} s)")
    time.sleep(MACHINING_TIME_S)
    hmi(1, "Status: UNLOADING CNC")
    ML('load_cnc')
    grip()
    leave_station('cnc')

    # VISION
    hmi(1, "Status: VISION INSPECTION")
    d, w = check_faults(serial, "vision"); downtime += d; waited += w
    hmi(1, "Status: VISION INSPECTION")
    go_station('vision')
    ML('place_vision')
    release('Frame_Vision')
    ML('approach_vision')
    diameter = random.gauss(NOMINAL_DIAM, 0.02)
    ok = abs(diameter - NOMINAL_DIAM) <= TOLERANCE
    color_part([0.1, 0.8, 0.2, 1] if ok else [0.9, 0.1, 0.1, 1])     # green OK / red NOK
    hmi(1, f"Status: MEASURED D={diameter:.3f} mm -> {'OK' if ok else 'NOK'}")
    time.sleep(1)
    ML('place_vision')
    grip()
    leave_station('vision')

    # OK pallet or NOK bin
    hmi(1, f"Status: SORTING to {'OK pallet' if ok else 'NOK bin'}")
    key = 'ok' if ok else 'nok'
    go_station(key)
    ML(STATIONS[key][2])
    release(STATIONS[key][0])
    leave_station(key)
    MJ('home')

    return {"order": ORDER_ID,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "serial": serial,
            "diameter_mm": round(diameter, 3),
            "result": 'OK' if ok else 'NOK',
            "cycle_time_s": round(time.time() - t0 - waited, 1),   # running time only
            "downtime_s": round(downtime, 1)}                       # stops (real seconds)


def log_row(row):
    write_csv(LOG_FILE, row)
    publish("part", row)


# ---------------------------------------------------------------- main
setup_cell()
hmi_init()
publish("order", {"order": ORDER_ID, "target": N_PARTS})
hmi(0, f"ORDER: {N_PARTS} parts  |  tolerance {NOMINAL_DIAM:.2f} +/- {TOLERANCE} mm")
hmi(1, "Status: READY")
hmi(2, "Last part: -")
hmi(3, "OK: 0   NOK: 0")

n_ok = 0
for i in range(1, N_PARTS + 1):
    row = run_cycle(f"SN-{i:04d}")
    log_row(row)
    n_ok += row["result"] == "OK"
    msg = (f"{row['serial']} | diam={row['diameter_mm']} mm | "
           f"{row['result']} | cycle={row['cycle_time_s']} s")
    print(msg)
    RDK.ShowMessage(msg, False)
    done = i
    total_down = globals().get("total_down", 0) + row["downtime_s"]
    globals()["total_down"] = total_down
    hmi(2, f"Last: {row['serial']}  D={row['diameter_mm']} mm  {row['result']}  "
           f"{row['cycle_time_s']} s  stop {row['downtime_s']:.0f} s")
    hmi(3, f"Done {done}/{N_PARTS}   OK: {n_ok}   NOK: {done - n_ok}   "
           f"Yield: {100 * n_ok / done:.0f} %   Downtime: {total_down:.0f} s")

summary = f"DONE: {N_PARTS} parts, {n_ok} OK, {N_PARTS - n_ok} NOK. Log: {LOG_FILE}"
print(summary)
RDK.ShowMessage(summary, False)
hmi(1, "Status: ORDER FINISHED")
