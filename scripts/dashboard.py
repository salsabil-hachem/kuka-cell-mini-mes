# =====================================================================
#  dashboard.py  -  STEP 3: visual MES dashboard of the KUKA cell
#
#  Reads kuka_cell_log.csv + kuka_cell_events.csv, computes the KPIs
#  (same formulas as kpi_report.py) and writes ONE self-contained file:
#      dashboard.html   (opens in your web browser, no internet needed)
#
#  Run it from RoboDK (Python program, double-click) or: python dashboard.py
# =====================================================================
import csv
import html
import math
import os
import webbrowser
from collections import defaultdict
from datetime import datetime

ONLY_LAST_ORDER = True      # True = last production order only
IDEAL_CYCLE_S = None        # target cycle time (s). None = best cycle observed
LAST_PARTS_ROWS = 15        # rows in the "last parts" table


# ---------------------------------------------------------------- data
def find_log_dir():
    candidates = []
    try:
        from robodk.robolink import Robolink
        d = Robolink().getParam('PATH_OPENSTATION')
        if d:
            candidates.append(str(d))
    except Exception:
        pass
    home = os.path.expanduser("~")
    candidates += [os.path.dirname(os.path.abspath(__file__)),
                   os.path.join(home, "Documents", "KUKA_Cell_MES")]
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


def compute(parts, events):
    n = len(parts)
    n_ok = sum(1 for p in parts if p["result"] == "OK")
    cycles = [float(p["cycle_time_s"]) for p in parts]
    run_time = sum(cycles)
    downtime = sum(float(e["duration_s"]) for e in events)
    planned = run_time + downtime
    ideal = IDEAL_CYCLE_S or min(cycles)
    a = run_time / planned if planned else 0
    p = min(1.0, ideal * n / run_time) if run_time else 0
    q = n_ok / n if n else 0
    pareto = defaultdict(lambda: [0, 0.0])
    for e in events:
        pareto[e["reason"]][0] += 1
        pareto[e["reason"]][1] += float(e["duration_s"])
    pareto = sorted(pareto.items(), key=lambda kv: kv[1][1], reverse=True)
    stops = len(events)
    return dict(n=n, ok=n_ok, nok=n - n_ok, run=run_time, down=downtime, planned=planned,
                ideal=ideal, a=a, p=p, q=q, oee=a * p * q, stops=stops,
                mtbf=run_time / stops if stops else None, mttr=downtime / stops if stops else None,
                avg=run_time / n if n else 0, pareto=pareto)


# ---------------------------------------------------------------- SVG charts
E = html.escape


def cycle_chart(parts, ideal):
    """Vertical bars: cycle time per part + ideal cycle reference line."""
    W, H, L, R, T, B = 1100, 260, 44, 12, 22, 40
    vals = [float(p["cycle_time_s"]) for p in parts]
    vmax = max(10, 10 * math.ceil(max(vals + [ideal]) * 1.1 / 10))   # round axis max to 10
    vmax = vmax if vmax <= 60 else 20 * math.ceil(vmax / 20)
    n = len(vals)
    slot = (W - L - R) / max(n, 1)
    bw = max(4, min(36, slot - 2))
    y = lambda v: T + (H - T - B) * (1 - v / vmax)
    out = [f'<svg viewBox="0 0 {W} {H}" width="{W}" role="img" aria-label="Cycle time per part">']
    step = 10 if vmax <= 60 else (20 if vmax <= 120 else 50)
    for v in range(0, int(vmax) + 1, step):                  # grid + y labels
        out.append(f'<line class="grid" x1="{L}" x2="{W-R}" y1="{y(v):.1f}" y2="{y(v):.1f}"/>')
        out.append(f'<text class="axis" x="{L-6}" y="{y(v)+4:.1f}" text-anchor="end">{v:.0f}</text>')
    for i, (p, v) in enumerate(zip(parts, vals)):
        x = L + i * slot + (slot - bw) / 2
        yy, h = y(v), (H - B) - y(v)
        tip = f'{p["serial"]}: {v:.1f} s ({p["result"]})'
        out.append(f'<g class="hit"><title>{E(tip)}</title>'
                   f'<rect x="{x-1:.1f}" y="{T}" width="{bw+2:.1f}" height="{H-B-T}" fill="transparent"/>'
                   f'<path class="bar" d="M{x:.1f},{H-B} V{yy+4:.1f} Q{x:.1f},{yy:.1f} {x+4:.1f},{yy:.1f} '
                   f'H{x+bw-4:.1f} Q{x+bw:.1f},{yy:.1f} {x+bw:.1f},{yy+4:.1f} V{H-B} Z"/></g>')
        if n <= 15 or i % max(1, n // 10) == 0:
            out.append(f'<text class="axis" x="{x+bw/2:.1f}" y="{H-B+16}" text-anchor="middle">{E(p["serial"][-4:])}</text>')
    yi = y(ideal)
    out.append(f'<line class="ref" x1="{L}" x2="{W-R}" y1="{yi:.1f}" y2="{yi:.1f}"/>')
    out.append(f'<line class="base" x1="{L}" x2="{W-R}" y1="{H-B}" y2="{H-B}"/>')
    out.append(f'<text class="axis" x="{L}" y="{H-4}">serial</text></svg>')
    return "".join(out)


def pareto_chart(pareto, total_down):
    """Horizontal bars: downtime per stop reason, biggest first."""
    if not pareto:
        return '<p class="empty">No stops in this order.</p>'
    W, row, L, R = 520, 34, 170, 100
    H = row * len(pareto) + 8
    vmax = max(sec for _, (c, sec) in pareto) or 1
    out = [f'<svg viewBox="0 0 {W} {H}" width="{W}" role="img" aria-label="Downtime by stop reason">']
    for i, (reason, (count, sec)) in enumerate(pareto):
        yy = 4 + i * row
        w = max(4, (W - L - R) * sec / vmax)
        share = 100 * sec / total_down if total_down else 0
        tip = f"{reason}: {count} stop(s), {sec:.0f} s, {share:.0f} % of downtime"
        out.append(f'<g class="hit"><title>{E(tip)}</title>'
                   f'<rect x="0" y="{yy}" width="{W}" height="{row-4}" fill="transparent"/>'
                   f'<text class="label" x="{L-10}" y="{yy+row/2+2}" text-anchor="end">{E(reason)}</text>'
                   f'<path class="bar" d="M{L},{yy+6} H{L+w-4:.1f} Q{L+w:.1f},{yy+6} {L+w:.1f},{yy+10} '
                   f'V{yy+row-14} Q{L+w:.1f},{yy+row-10} {L+w-4:.1f},{yy+row-10} H{L} Z"/>'
                   f'<text class="value" x="{L+w+8:.1f}" y="{yy+row/2+2}">{sec:.0f} s &middot; {share:.0f} %</text></g>')
    out.append(f'<line class="base" x1="{L}" x2="{L}" y1="0" y2="{H}"/></svg>')
    return "".join(out)


def apq_bars(k):
    rows = [("Availability", k["a"], "running &divide; planned"),
            ("Performance", k["p"], "ideal &times; parts &divide; running"),
            ("Quality", k["q"], "OK &divide; produced")]
    out = []
    for name, v, formula in rows:
        out.append(f'<div class="apq"><div class="apq-head"><span>{name}</span>'
                   f'<strong>{100*v:.1f} %</strong></div>'
                   f'<div class="track"><div class="fill" style="width:{100*v:.1f}%"></div></div>'
                   f'<div class="muted small">{formula}</div></div>')
    return "".join(out)


def status_badge(result):
    if result == "OK":
        return '<span class="badge good">&#10003; OK</span>'
    return '<span class="badge critical">&#10005; NOK</span>'


# ---------------------------------------------------------------- page
def build_html(order, k, parts, events):
    oee = 100 * k["oee"]
    oee_word = "World class" if oee >= 85 else ("Typical plant" if oee >= 60 else "Needs improvement")
    top = k["pareto"][0][0] if k["pareto"] else None
    insight = (f"Biggest loss: <strong>{E(top)}</strong> &mdash; "
               f"{100*k['pareto'][0][1][1]/k['down']:.0f} % of downtime." if top else
               "No stops recorded in this order.")
    rows = []
    for p in reversed(parts[-LAST_PARTS_ROWS:]):
        rows.append(f'<tr><td>{E(p["serial"])}</td><td>{E(p["timestamp"].replace("T", " "))}</td>'
                    f'<td class="num">{float(p["diameter_mm"]):.3f}</td><td>{status_badge(p["result"])}</td>'
                    f'<td class="num">{float(p["cycle_time_s"]):.1f}</td>'
                    f'<td class="num">{float(p.get("downtime_s", 0) or 0):.0f}</td></tr>')
    ev_rows = "".join(f'<tr><td>{E(e["start"].replace("T", " "))}</td><td>{E(e["serial"])}</td>'
                      f'<td>{E(e["station"])}</td><td>{E(e["reason"])}</td>'
                      f'<td class="num">{float(e["duration_s"]):.0f}</td></tr>' for e in reversed(events))
    sec = lambda x: "&ndash;" if x is None else f"{x:.0f} s"
    generated = datetime.now().strftime("%Y-%m-%d %H:%M")

    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>KUKA Cell Dashboard</title>
<style>
:root {{
  color-scheme: light;
  --surface-0:#f4f3f0; --surface-1:#fcfcfb; --border:#e3e2de;
  --text-primary:#0b0b0b; --text-secondary:#52514e; --text-muted:#7a7974;
  --grid:#ecebe7; --series-1:#2a78d6; --track:#ecebe7;
  --good:#0ca30c; --good-text:#006300; --critical:#d03b3b; --critical-text:#b02a2a;
}}
@media (prefers-color-scheme: dark) {{
  :root:where(:not([data-theme="light"])) {{
    color-scheme: dark;
    --surface-0:#121211; --surface-1:#1a1a19; --border:#2e2e2b;
    --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8f8e86;
    --grid:#2a2a27; --series-1:#3987e5; --track:#2a2a27;
    --good:#0ca30c; --good-text:#4cc94c; --critical:#d03b3b; --critical-text:#ff8a8a;
  }}
}}
:root[data-theme="dark"] {{
  color-scheme: dark;
  --surface-0:#121211; --surface-1:#1a1a19; --border:#2e2e2b;
  --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8f8e86;
  --grid:#2a2a27; --series-1:#3987e5; --track:#2a2a27;
  --good:#0ca30c; --good-text:#4cc94c; --critical:#d03b3b; --critical-text:#ff8a8a;
}}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--surface-0); color:var(--text-primary);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }}
.wrap {{ max-width:1180px; margin:0 auto; padding:24px 16px 48px; }}
header {{ display:flex; flex-wrap:wrap; justify-content:space-between; align-items:baseline; gap:8px; margin-bottom:18px; }}
h1 {{ font-size:22px; margin:0; }}
h2 {{ font-size:15px; margin:0 0 12px; }}
.muted {{ color:var(--text-secondary); }} .small {{ font-size:12px; }}
.grid2 {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); gap:16px; }}
.tiles {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(160px,1fr)); gap:16px; margin-bottom:16px; }}
.card {{ background:var(--surface-1); border:1px solid var(--border); border-radius:12px; padding:18px; min-width:0; }}
.tile .label {{ color:var(--text-secondary); font-size:12px; text-transform:uppercase; letter-spacing:.04em; }}
.tile .big {{ font-size:30px; font-weight:650; margin-top:4px; font-variant-numeric:tabular-nums; }}
.hero .big {{ font-size:clamp(30px,6vw,44px); white-space:nowrap; }}
.tile .big {{ white-space:nowrap; }}
.apq {{ margin-bottom:14px; }}
.apq-head {{ display:flex; justify-content:space-between; margin-bottom:6px; }}
.track {{ height:8px; background:var(--track); border-radius:4px; overflow:hidden; }}
.fill {{ height:100%; background:var(--series-1); border-radius:4px; }}
svg {{ max-width:100%; height:auto; display:block; overflow:visible; }}
svg .grid {{ stroke:var(--grid); stroke-width:1; }}
svg .base {{ stroke:var(--text-muted); stroke-width:1; }}
svg .bar {{ fill:var(--series-1); }}
svg .hit:hover .bar {{ opacity:.8; }}
svg .ref {{ stroke:var(--text-secondary); stroke-width:1.5; }}
svg text {{ font:12px system-ui,-apple-system,"Segoe UI",sans-serif; fill:var(--text-secondary); }}
svg .label {{ fill:var(--text-primary); }}
svg .value, svg .reflabel {{ fill:var(--text-secondary); }}
.insight {{ margin-top:10px; padding:10px 12px; border-left:3px solid var(--series-1); background:var(--surface-0); border-radius:6px; }}
.table-wrap {{ overflow-x:auto; }}
table {{ width:100%; border-collapse:collapse; font-size:13px; }}
th, td {{ text-align:left; padding:8px 10px; border-bottom:1px solid var(--border); white-space:nowrap; }}
th {{ color:var(--text-secondary); font-weight:600; font-size:12px; }}
td.num, th.num {{ text-align:right; font-variant-numeric:tabular-nums; }}
.badge {{ display:inline-block; padding:2px 8px; border-radius:999px; font-size:12px; font-weight:600; }}
.badge.good {{ color:var(--good-text); background:color-mix(in srgb, var(--good) 14%, transparent); }}
.badge.critical {{ color:var(--critical-text); background:color-mix(in srgb, var(--critical) 14%, transparent); }}
.empty {{ color:var(--text-secondary); }}
section {{ margin-top:16px; }}
</style></head>
<body><div class="wrap">
<header>
  <div><h1>KUKA KR 10 machine-tending cell</h1>
  <div class="muted">Production order <strong>{E(order)}</strong> &middot; {k['n']} parts</div></div>
  <div class="muted small">Generated {generated}</div>
</header>

<div class="tiles">
  <div class="card tile hero"><div class="label">OEE</div><div class="big">{oee:.1f} %</div>
    <div class="muted small">{oee_word} &middot; world class &ge; 85 %</div></div>
  <div class="card tile"><div class="label">Good parts</div><div class="big">{k['ok']} / {k['n']}</div>
    <div class="small">{status_badge('OK')} {k['ok']} &nbsp; {status_badge('NOK')} {k['nok']}</div></div>
  <div class="card tile"><div class="label">Avg cycle time</div><div class="big">{k['avg']:.1f} s</div>
    <div class="muted small">ideal {k['ideal']:.1f} s</div></div>
  <div class="card tile"><div class="label">Downtime</div><div class="big">{k['down']:.0f} s</div>
    <div class="muted small">{k['stops']} stops &middot; MTBF {sec(k['mtbf'])} &middot; MTTR {sec(k['mttr'])}</div></div>
</div>

<div class="grid2">
  <div class="card"><h2>OEE = Availability &times; Performance &times; Quality</h2>{apq_bars(k)}
    <div class="insight">{insight}</div></div>
  <div class="card"><h2>Downtime by stop reason (Pareto)</h2>{pareto_chart(k['pareto'], k['down'])}</div>
</div>

<section class="card"><h2>Cycle time per part (s)</h2>{cycle_chart(parts, k['ideal'])}
  <div class="muted small">Hover a bar for details. Horizontal line = ideal cycle time ({k['ideal']:.1f} s).</div></section>

<div class="grid2">
<section class="card"><h2>Last parts (traceability)</h2><div class="table-wrap"><table>
<thead><tr><th>Serial</th><th>Time</th><th class="num">&Oslash; (mm)</th><th>Result</th><th class="num">Cycle (s)</th><th class="num">Stop (s)</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></div></section>
<section class="card"><h2>Stop events</h2><div class="table-wrap"><table>
<thead><tr><th>Start</th><th>Serial</th><th>Station</th><th>Reason</th><th class="num">Duration (s)</th></tr></thead>
<tbody>{ev_rows or '<tr><td colspan="5" class="empty">No stops</td></tr>'}</tbody></table></div></section>
</div>
</div></body></html>"""


# ---------------------------------------------------------------- main
log_dir = find_log_dir()
parts = read_csv(os.path.join(log_dir, "kuka_cell_log.csv"))
events = read_csv(os.path.join(log_dir, "kuka_cell_events.csv"))
if not parts:
    raise Exception("No parts in kuka_cell_log.csv. Run main_cycle first.")
if "order" not in parts[0]:
    raise Exception("Old log format. Run the new main_cycle first.")

order = parts[-1]["order"] if ONLY_LAST_ORDER else "ALL"
if ONLY_LAST_ORDER:
    parts = [p for p in parts if p["order"] == order]
    events = [e for e in events if e.get("order") == order]

kpi = compute(parts, events)
out = os.path.join(log_dir, "dashboard.html")
with open(out, "w", encoding="utf-8") as f:
    f.write(build_html(order, kpi, parts, events))
print(f"Dashboard written: {out}")
webbrowser.open("file:///" + out.replace("\\", "/"))
