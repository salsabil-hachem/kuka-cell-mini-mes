# =====================================================================
#  kpi_report.py  -  STEP 2: computes the cell KPIs from the MES logs
#
#  Reads:   kuka_cell_log.csv     (one line per part)
#           kuka_cell_events.csv  (one line per stop)
#  Computes: OEE = Availability x Performance x Quality
#            yield, cycle times, MTBF, MTTR, downtime Pareto
#  Writes:  kpi_report.txt  (+ shows it in RoboDK if run from RoboDK)
#
#  Run it from RoboDK (Python program, double-click) or with: python kpi_report.py
# =====================================================================
import csv
import os
from collections import defaultdict

# ---------------------------------------------------------------- settings
ONLY_LAST_ORDER = True     # True = KPIs of the last production order only
IDEAL_CYCLE_S = None       # ideal cycle time (s). None = best cycle observed


# ---------------------------------------------------------------- find the files
def find_log_dir():
    candidates = []
    try:                                           # folder of the RoboDK station
        from robodk.robolink import Robolink
        d = Robolink().getParam('PATH_OPENSTATION')
        if d:
            candidates.append(str(d))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    home = os.path.expanduser("~")
    candidates += [here, os.path.join(home, "Documents", "KUKA_Cell_MES")]
    for c in candidates:
        if os.path.exists(os.path.join(c, "kuka_cell_log.csv")):
            return c
    for root in [os.path.join(home, d) for d in ("Documents", "OneDrive", "Downloads", "Desktop")]:
        if not os.path.isdir(root):
            continue
        base = root.count(os.sep)
        for folder, subdirs, files in os.walk(root):
            if folder.count(os.sep) - base > 4:
                subdirs[:] = []
                continue
            if "kuka_cell_log.csv" in files:
                return folder
    raise Exception("kuka_cell_log.csv not found. Run main_cycle first.")


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------- KPI calculation
def compute(parts, events):
    n = len(parts)
    if n == 0:
        raise Exception("No parts in the log.")
    n_ok = sum(1 for p in parts if p["result"] == "OK")
    cycles = [float(p["cycle_time_s"]) for p in parts]
    run_time = sum(cycles)                                        # robot working
    downtime = sum(float(e["duration_s"]) for e in events)       # cell stopped
    planned = run_time + downtime                                # total production time
    ideal = IDEAL_CYCLE_S or min(cycles)

    availability = run_time / planned if planned else 0
    performance = min(1.0, ideal * n / run_time) if run_time else 0
    quality = n_ok / n
    oee = availability * performance * quality

    n_stops = len(events)
    mtbf = run_time / n_stops if n_stops else None              # mean time between failures
    mttr = downtime / n_stops if n_stops else None              # mean time to repair

    pareto = defaultdict(lambda: [0, 0.0])                      # reason -> [count, seconds]
    for e in events:
        pareto[e["reason"]][0] += 1
        pareto[e["reason"]][1] += float(e["duration_s"])
    pareto = sorted(pareto.items(), key=lambda kv: kv[1][1], reverse=True)

    return {
        "parts": n, "ok": n_ok, "nok": n - n_ok,
        "run_time": run_time, "downtime": downtime, "planned": planned,
        "ideal": ideal, "avg_cycle": run_time / n, "min_cycle": min(cycles), "max_cycle": max(cycles),
        "availability": availability, "performance": performance, "quality": quality, "oee": oee,
        "stops": n_stops, "mtbf": mtbf, "mttr": mttr, "pareto": pareto,
    }


def fmt_report(order, k):
    pct = lambda x: f"{100 * x:5.1f} %"
    sec = lambda x: "-" if x is None else f"{x:6.1f} s"
    lines = [
        f"KPI REPORT  -  order {order}",
        "=" * 46,
        f"Parts produced      : {k['parts']}   (OK {k['ok']} / NOK {k['nok']})",
        f"Planned time        : {k['planned']:.1f} s",
        f"Running time        : {k['run_time']:.1f} s",
        f"Downtime            : {k['downtime']:.1f} s  in {k['stops']} stops",
        "-" * 46,
        f"Availability (A)    : {pct(k['availability'])}   running / planned",
        f"Performance  (P)    : {pct(k['performance'])}   ideal x parts / running",
        f"Quality      (Q)    : {pct(k['quality'])}   OK / produced",
        f"OEE = A x P x Q     : {pct(k['oee'])}",
        "-" * 46,
        f"Cycle time avg/min/max : {k['avg_cycle']:.1f} / {k['min_cycle']:.1f} / {k['max_cycle']:.1f} s",
        f"Ideal cycle time       : {k['ideal']:.1f} s",
        f"MTBF (between stops)   : {sec(k['mtbf'])}",
        f"MTTR (repair time)     : {sec(k['mttr'])}",
        "-" * 46,
        "Downtime Pareto (biggest loss first):",
    ]
    for reason, (count, seconds) in k["pareto"]:
        share = seconds / k["downtime"] if k["downtime"] else 0
        lines.append(f"  {reason:26s} {count:2d}x  {seconds:6.1f} s  {pct(share)}")
    if not k["pareto"]:
        lines.append("  no stops")
    return "\n".join(lines)


# ---------------------------------------------------------------- main
log_dir = find_log_dir()
parts = read_csv(os.path.join(log_dir, "kuka_cell_log.csv"))
events = read_csv(os.path.join(log_dir, "kuka_cell_events.csv"))

if parts and "order" not in parts[0]:
    raise Exception("Old log format. Run the new main_cycle first.")

order = parts[-1]["order"] if (ONLY_LAST_ORDER and parts) else "ALL"
if ONLY_LAST_ORDER:
    parts = [p for p in parts if p["order"] == order]
    events = [e for e in events if e.get("order") == order]

kpi = compute(parts, events)
report = fmt_report(order, kpi)
print(report)

out = os.path.join(log_dir, "kpi_report.txt")
with open(out, "w") as f:
    f.write(report + "\n")

try:
    from robodk.robolink import Robolink
    Robolink().ShowMessage(report + f"\n\nSaved: {out}", True)
except Exception:
    pass
