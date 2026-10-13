# =====================================================================
#  check_paths.py  -  checks every robot move of the cycle
#
#  For each move it checks:
#    1. REACH      : the robot can reach the target
#    2. COLLISION  : no collision along the path (robot / gripper vs cell)
#    3. LINEAR     : the straight-line (LIN) move is possible
#    4. SINGULARITY: wrist A5 not close to 0 deg (KUKA wrist singularity)
#    5. JOINT LIMIT: no joint closer than 5 deg to its limit
#
#  Run main_cycle.py at least once before (it writes the target values).
#  Result: a report in RoboDK + path_check_report.csv next to the station.
# =====================================================================
import csv
import os

from robodk.robolink import (Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET,
                             ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, COLLISION_ON, COLLISION_OFF)
from robodk.robomath import transl

RDK = Robolink()

SIM_SPEED = 1          # 1 = real speed (slow, easy to watch). 5 = faster
SINGULARITY_DEG = 10   # warn if |A5| below this
LIMIT_MARGIN_DEG = 5   # warn if a joint is this close to its limit
HOME_JOINTS = [0, -90, 90, 0, 90, 0]

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


def choose_home():
    """Pick the home where the gripper points DOWN (A5 = +90 or -90).
    A home with the gripper pointing up makes the wrist flip during the moves
    and the gripper fingers hit the robot arm (self-collision)."""
    global HOME_JOINTS
    for a5 in (90, -90):
        j = [0, -90, 90, 0, a5, 0]
        robot.setJoints(j)
        tcp = robot.Pose()                  # TCP pose (active tool) in robot base
        if tcp[2, 2] < -0.5:                # tool Z axis points down
            HOME_JOINTS = j
            print(f"Home: A5 = {a5} (gripper pointing down)")
            return j
    return HOME_JOINTS

# Same sequence as main_cycle (OK branch + NOK branch)
SEQUENCE = [
    ("J", "home"),
    ("J", "via_infeed"), ("J", "Approach_infeed"), ("L", "pick_infeed"), ("GRIP", None),
    ("L", "Approach_infeed"), ("J", "via_infeed"),
    ("J", "via_cnc"), ("J", "approach_cnc"), ("L", "load_cnc"), ("RELEASE", "Frame_CNC"),
    ("L", "approach_cnc"), ("L", "load_cnc"), ("GRIP", None), ("L", "approach_cnc"), ("J", "via_cnc"),
    ("J", "via_vision"), ("J", "approach_vision"), ("L", "place_vision"), ("RELEASE", "Frame_Vision"),
    ("L", "approach_vision"), ("L", "place_vision"), ("GRIP", None), ("L", "approach_vision"),
    ("J", "via_vision"),
    ("J", "via_ok"), ("J", "approach_ok"), ("L", "place_ok"), ("RELEASE", "Frame_OK"),
    ("L", "approach_ok"), ("J", "via_ok"),
    ("J", "via_nok"), ("J", "approach_nok"), ("L", "place_nok"), ("L", "approach_nok"),  # NOK (empty gripper)
    ("J", "via_nok"),
    ("J", "home"),
]


def item(name, itype):
    it = RDK.Item(name, itype)
    if not it.Valid():
        raise Exception(f"Not found in the tree: {name}")
    return it


def disable_part_collisions():
    """The part touches the gripper and tables on purpose -> ignore it."""
    for other in RDK.ItemList():
        for link in range(8):
            try:
                RDK.setCollisionActivePair(COLLISION_OFF, part, other, 0, link)
            except Exception:
                pass


def as_list(values):
    """RoboDK returns joints as a Mat object -> convert to a flat Python list."""
    if hasattr(values, "rows"):          # robodk Mat (column or row vector)
        values = values.rows
    elif hasattr(values, "list"):
        values = values.list()
    out = []
    for v in values:
        if isinstance(v, (list, tuple)):
            out.extend(v)
        else:
            out.append(v)
    return [float(x) for x in out]


def check_joints(joints):
    """Return a list of warnings for singularity and joint limits."""
    warnings = []
    j = as_list(joints)
    if len(j) < 6:
        return warnings
    if abs(j[4]) < SINGULARITY_DEG:
        warnings.append(f"A5={j[4]:.1f} deg (near singularity)")
    limits = robot.JointLimits()
    lower, upper = as_list(limits[0]), as_list(limits[1])
    for i in range(6):
        if j[i] - lower[i] < LIMIT_MARGIN_DEG or upper[i] - j[i] < LIMIT_MARGIN_DEG:
            warnings.append(f"A{i+1}={j[i]:.1f} deg (near limit)")
    return warnings


# ---------------------------------------------------------------- run
RDK.setSimulationSpeed(SIM_SPEED)
RDK.setCollisionActive(COLLISION_ON)
disable_part_collisions()

part.setParent(item('Frame_Infeed', ITEM_TYPE_FRAME))
part.setPose(transl(0, 0, 400))
via = RDK.Item('via_infeed', ITEM_TYPE_TARGET)
if not via.Valid() or via.Name() != 'via_infeed':
    raise Exception('Run main_cycle once first (it creates the via_ points).')
robot.setJoints(item('home', ITEM_TYPE_TARGET).Joints())

report = []
n_problems = 0

for step, (kind, name) in enumerate(SEQUENCE, start=1):
    if kind == "GRIP":
        part.setParentStatic(tool)
        continue
    if kind == "RELEASE":
        part.setParentStatic(item(name, ITEM_TYPE_FRAME))
        continue

    target = item(name, ITEM_TYPE_TARGET)
    j_start = robot.Joints()
    row = {"step": step, "move": "PTP" if kind == "J" else "LIN", "target": name,
           "reach": "OK", "linear": "-", "collisions": 0, "hit": "", "warnings": "", "status": "OK"}

    # Test LIN feasibility before moving
    if kind == "L":
        robot.setPoseFrame(target.Parent())
        res = robot.MoveL_Test(j_start, target.Pose())
        row["linear"] = "OK" if res == 0 else ("collision" if res > 0 else "NOT POSSIBLE")

    # Execute the move (PTP if LIN impossible, like main_cycle does)
    try:
        if kind == "L" and row["linear"] != "NOT POSSIBLE":
            robot.MoveL(target)
        else:
            robot.MoveJ(target.Joints())     # joints computed by main_cycle setup
    except Exception:
        row["reach"] = "STOPPED (collision or unreachable)"
        row["hit"] = collision_report()
        row["status"] = "FAIL"
        report.append(row)
        n_problems += 1
        print(f"{step:02d} {name:16s} STOPPED (collision or unreachable)")
        break

    j_end = robot.Joints()
    # Collision along the joint path (PTP) or at the end pose
    if kind == "J":
        row["collisions"] = max(0, robot.MoveJ_Test(j_start, j_end))
    else:
        row["collisions"] = max(0, RDK.Collisions())
    if row["collisions"] > 0:
        row["hit"] = collision_report()
    warnings = check_joints(j_end)
    row["warnings"] = "; ".join(warnings)

    if row["collisions"] > 0 or row["linear"] in ("collision", "NOT POSSIBLE"):
        row["status"] = "FAIL"
    elif warnings:
        row["status"] = "WARNING"
    if row["status"] != "OK":
        n_problems += 1

    report.append(row)
    print(f"{step:02d} {row['move']} {name:16s} reach={row['reach']:4s} "
          f"lin={row['linear']:12s} coll={row['collisions']} {row['status']} {row['warnings']}")

RDK.setCollisionActive(COLLISION_OFF)

# ---------------------------------------------------------------- report
station_path = RDK.getParam('PATH_OPENSTATION')
out_dir = station_path if station_path and os.path.isdir(station_path) else os.path.expanduser("~")
out_file = os.path.join(out_dir, "path_check_report.csv")
with open(out_file, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(report[0].keys()))
    w.writeheader()
    w.writerows(report)

lines = [f"{r['step']:02d} {r['move']} {r['target']}: {r['status']}"
         + (f" {r['reach']}" if r['reach'] != 'OK' else "")
         + (f" ({r['warnings']})" if r['warnings'] else "")
         + (f" collisions={r['collisions']}" if r['collisions'] else "")
         + (f"\n      HIT: {r['hit']}" if r['hit'] else "")
         + (f" LIN={r['linear']}" if r['linear'] not in ('-', 'OK') else "")
         for r in report]
summary = ("ALL PATHS OK" if n_problems == 0 else f"{n_problems} PROBLEM(S) FOUND")
RDK.ShowMessage(summary + "\n\n" + "\n".join(lines) + f"\n\nReport: {out_file}", True)
print(summary, "->", out_file)
