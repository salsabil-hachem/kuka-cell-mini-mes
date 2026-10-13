# =====================================================================
#  build_cell.py  -  builds the whole KUKA cell automatically in RoboDK
#
#  Before running:
#   - The station must already contain: KUKA KR 10 R1100-2 + RobotiQ gripper
#     + a tool named "Tool_Gripper" (TCP Z = 150).
#   - Unzip KUKA_Cell_Models.zip anywhere (Downloads, Documents, Desktop...)
#
#  Run it once (Python program in RoboDK -> double-click).
#  It creates frames, objects, targets, then SAVES the station
#  as KUKA_Cell.rdk next to the STL files.
#  Then run main_cycle.py to produce parts.
# =====================================================================
import os

from robodk.robolink import (Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL,
                             ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET)
from robodk.robomath import transl, eye

RDK = Robolink()

NEEDED = ["Table.stl", "Part_Shaft.stl", "CNC_Machine.stl", "Pallet_OK.stl", "Bin_NOK.stl"]


def find_models_dir():
    """Look for the folder that contains the 5 STL files (Documents, OneDrive,
    Downloads, Desktop...). If not found, ask the user to pick the folder."""
    home = os.path.expanduser("~")
    roots = [os.path.join(home, d) for d in
             ("Documents", "OneDrive", "Downloads",
              "Desktop", "Bureau")]
    roots += [os.path.join(home, "OneDrive", d) for d in ("Documents", "Desktop", "Bureau")]
    roots += ["C:\\KUKA_Cell_MES"]
    for root in roots:
        if not os.path.isdir(root):
            continue
        base_depth = root.rstrip("\\/").count(os.sep)
        for folder, subdirs, files in os.walk(root):
            if folder.count(os.sep) - base_depth > 4:
                subdirs[:] = []          # don't search too deep
                continue
            if all(f in files for f in NEEDED):
                return folder
    # Not found automatically -> folder picker
    try:
        import tkinter
        from tkinter import filedialog
        win = tkinter.Tk()
        win.withdraw()
        win.attributes("-topmost", True)
        folder = filedialog.askdirectory(title="Select the folder that contains the 5 STL files")
        win.destroy()
        if folder and all(os.path.exists(os.path.join(folder, f)) for f in NEEDED):
            return folder
    except Exception:
        pass
    return None


MODELS_DIR = find_models_dir()
if MODELS_DIR is None:
    raise Exception("The 5 STL files were not found. Unzip KUKA_Cell_Models.zip "
                    "into Documents\\KUKA_Cell_MES\\models and run again.")
# Project folder = the folder above "models" (or the models folder itself)
PROJECT_DIR = (os.path.dirname(MODELS_DIR)
               if os.path.basename(MODELS_DIR).lower() == "models" else MODELS_DIR)
STATION_FILE = os.path.join(PROJECT_DIR, "KUKA_Cell.rdk")
print(f"Models found in: {MODELS_DIR}")

# Frames: name -> (X, Y, Z) relative to the station
FRAMES = {
    "Frame_Infeed": (300, -650, 0),
    "Frame_CNC":    (1050, 0, 0),
    "Frame_Vision": (300, 650, 0),
    "Frame_OK":     (-450, 450, 0),
    "Frame_NOK":    (-450, -450, 0),
}

# Objects: name -> (STL file, frame, Z offset, color RGBA 0-1)
OBJECTS = {
    "Table_Infeed": ("Table.stl",       "Frame_Infeed", 0,   [0.75, 0.75, 0.75, 1]),
    "Part_Shaft":   ("Part_Shaft.stl",  "Frame_Infeed", 400, [0.85, 0.65, 0.20, 1]),
    "CNC_Machine":  ("CNC_Machine.stl", "Frame_CNC",    0,   [0.55, 0.60, 0.65, 1]),
    "Table_Vision": ("Table.stl",       "Frame_Vision", 0,   [0.75, 0.75, 0.75, 1]),
    "Pallet_OK":    ("Pallet_OK.stl",   "Frame_OK",     0,   [0.20, 0.70, 0.30, 1]),
    "Bin_NOK":      ("Bin_NOK.stl",     "Frame_NOK",    0,   [0.85, 0.15, 0.15, 1]),
}

# Targets: name -> frame (values are written later by main_cycle.py)
TARGETS = {
    "Approach_infeed": "Frame_Infeed", "pick_infeed": "Frame_Infeed",
    "approach_cnc": "Frame_CNC",       "load_cnc": "Frame_CNC",
    "approach_vision": "Frame_Vision", "place_vision": "Frame_Vision",
    "approach_ok": "Frame_OK",         "place_ok": "Frame_OK",
    "approach_nok": "Frame_NOK",       "place_nok": "Frame_NOK",
}
HOME_JOINTS = [0, -90, 90, 0, 90, 0]


def delete_if_exists(name, item_type):
    item = RDK.Item(name, item_type)
    while item.Valid() and item.Name() == name:
        item.Delete()
        item = RDK.Item(name, item_type)


# ---------------------------------------------------------------- checks
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
if not os.path.isdir(MODELS_DIR):
    raise Exception(f"Models folder not found: {MODELS_DIR}\n"
                    f"Unzip KUKA_Cell_Models.zip there.")

RDK.Render(False)

# Robot base at the origin of the station
robot.Parent().setPose(eye(4))
robot.setPoseTool(tool)

# ---------------------------------------------------------------- frames
frames = {}
for name, (x, y, z) in FRAMES.items():
    delete_if_exists(name, ITEM_TYPE_FRAME)   # also deletes its children
    fr = RDK.AddFrame(name)
    fr.setPose(transl(x, y, z))
    frames[name] = fr
    print(f"Frame   {name:14s} at ({x}, {y}, {z})")

# ---------------------------------------------------------------- objects
for name, (stl, frame, z, color) in OBJECTS.items():
    path = os.path.join(MODELS_DIR, stl)
    if not os.path.exists(path):
        raise Exception(f"Missing model file: {path}")
    obj = RDK.AddFile(path, frames[frame])
    obj.setName(name)
    obj.setPose(transl(0, 0, z))
    obj.setColor(color)
    print(f"Object  {name:14s} in {frame}")

# ---------------------------------------------------------------- targets
robot.setJoints(HOME_JOINTS)
delete_if_exists("home", ITEM_TYPE_TARGET)
home = RDK.AddTarget("home", robot.Parent(), robot)
home.setAsJointTarget()
home.setJoints(HOME_JOINTS)
print("Target  home (joints)")

for name, frame in TARGETS.items():
    tgt = RDK.AddTarget(name, frames[frame], robot)
    print(f"Target  {name:14s} in {frame}")

RDK.Render(True)

# ---------------------------------------------------------------- save
os.makedirs(PROJECT_DIR, exist_ok=True)
RDK.ActiveStation().setName("KUKA_Cell")
RDK.Save(STATION_FILE)
msg = f"Cell built and saved to {STATION_FILE}. Now run main_cycle.py"
print(msg)
RDK.ShowMessage(msg, False)
