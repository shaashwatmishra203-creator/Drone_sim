"""Is the hall course passable at the planner's inflation? BFS on true geometry."""
import sys, math, collections
sys.path.insert(0, "/home/shaash/drone_sim/tools")
import yaml, gen_hall
cfg = yaml.safe_load(open(gen_hall.DEFAULT_CFG))
M = cfg["mission"]
infl = M["airframe_radius_m"] + M["clearance_margin_m"] + 0.2   # + half a map cell
objs = gen_hall.objects(cfg)
R = 0.2
(x0, x1), (y0, y1) = cfg["hall"]["x"], cfg["hall"]["y"]
nx, ny = int((x1 - x0) / R), int((y1 - y0) / R)
def blocked(i, j):
    x, y = x0 + (i + 0.5) * R, y0 + (j + 0.5) * R
    for _, cls, cx, cy, cz, sx, sy, sz in objs:
        dx = max(abs(x - cx) - sx / 2, 0); dy = max(abs(y - cy) - sy / 2, 0)
        lim = (M["wall_margin_m"] if cls == "wall" else infl)
        if math.hypot(dx, dy) < lim:
            return True
    return False
free = [[not blocked(i, j) for j in range(ny)] for i in range(nx)]
def cell(x, y): return int((x - x0) / R), int((y - y0) / R)
s, g = cell(*cfg["pad"]["xy"]), cell(*M["scan_point"])
prev = {s: None}; q = collections.deque([s])
while q:
    c = q.popleft()
    if c == g: break
    for d in ((1,0),(-1,0),(0,1),(0,-1)):
        n = (c[0]+d[0], c[1]+d[1])
        if 0 <= n[0] < nx and 0 <= n[1] < ny and free[n[0]][n[1]] and n not in prev:
            prev[n] = c; q.append(n)
if g not in prev:
    print(f"NOT passable at inflation {infl:.2f} m"); raise SystemExit(1)
path = []; c = g
while c: path.append(c); c = prev[c]
ys_in_aisle = [y0 + (j + .5) * R for i, j in path if 4 < x0 + (i + .5) * R < 31]
print(f"passable at inflation {infl:.2f} m; shortest grid path {len(path) * R:.1f} m; "
      f"lateral travel inside aisle spans y {min(ys_in_aisle):.1f} .. {max(ys_in_aisle):.1f}")
# narrowest point: min free width across y inside the aisle
w = []
for i in range(nx):
    x = x0 + (i + .5) * R
    if 4 < x < 31:
        w.append((sum(free[i][j] for j in range(ny) if -5 < y0 + (j + .5) * R < 5) * R, x))
print(f"narrowest free corridor inside the aisle: {min(w)[0]:.1f} m wide at x = {min(w)[1]:.1f}")
