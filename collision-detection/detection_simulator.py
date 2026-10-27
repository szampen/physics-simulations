"""
3D Simulation made in Ursina

Boxes float in a cube, colliding ones are highlighted in red.
Collision detection methods can be changed during the simulation and compared with each other,
by their average computing time, on the screen

Installation: pip install ursina
Controls: 1 = Brute Force, 2 = Sweep and Prune, 3 = BVH
             +/- = add/remove 50 objects
             right/middle mouse button + move = camera movement (Ursina's EditorCamera)
"""

from ursina import *
from collision_algorithms import ALGORITHMS

HALF = 25.0                      # world is a cube [-HALF, HALF]^3
START_N, STEP = 300, 50
MAX_SPEED = 4.0

app = Ursina(title="3D Collision Detection Simulator")
window.color = color.rgb(16, 16, 16)
window.entity_counter.enabled = False
window.collider_counter.enabled = False
window.fps_counter.enabled = False
window.exit_button.enabled = False
EditorCamera() # enables interactive camera movement
camera.position = (0, 0, -90)

Entity(model='wireframe_cube', scale=HALF * 2, color=color.black)    # world's border

bodies = []        #  dict(e=Entity, v=Vec3, h=(hx,hy,hz), hit=boolean)
# e - object
# v - velocity of the object
# h - halves of the measures of the object
# hit - collision flag

def add_body():
    # randomizes the measures of the cube (its halves)
    h = tuple(random.uniform(0.3, 1.2) for _ in range(3))
    # creates lime cube, scales it to 2*h and places it in a random position inside the world
    e = Entity(model='cube', color=color.lime, scale=tuple(2 * x for x in h),
               position=tuple(random.uniform(-HALF + 2, HALF - 2) for _ in range(3)))
    # randomizes its velocity
    v = Vec3(*(random.uniform(-MAX_SPEED, MAX_SPEED) for _ in range(3)))
    # appends to dict
    bodies.append(dict(e=e, v=v, h=h, hit=False))

# removes k objects from the scene, always leaves at least 2 objects
def remove_bodies(k):
    for _ in range(min(k, len(bodies) - 2)):
        destroy(bodies.pop()['e'])

# starting loop
for _ in range(START_N):
    add_body()

algo_names = list(ALGORITHMS)
current = 1                                  # default: Sweep and Prune
avg_ms: dict[str, float | None] = {name: None for name in algo_names}  # average computing time for algorithms [ms]
hud_bg = Entity(
    parent=camera.ui,
    model='quad',
    color=color.rgba(0, 0, 0, 0.45),
    position=(-0.88, 0.48, 0.1),       # z=0.1 moves background behind the text
    scale=(0.85, 0.235),               # width and height of the padding
    origin=(-0.5, 0.5)                 # upper left corner
)

hud = Text(
    text='',
    position=(-0.86, 0.46, 0.0),      # z=0.0 places text in front of the background
    scale=1.1,
    color=color.white,
    origin=(-0.5, 0.5)
)

def input(key):
    global current
    if key in ('1', '2', '3'):
        current = int(key) - 1
    elif key in ('=', '+'):
        for _ in range(STEP): add_body()
        for n in algo_names: avg_ms[n] = None
    elif key == '-':
        remove_bodies(STEP)
        for n in algo_names: avg_ms[n] = None

def update():
    dt = min(time.dt, 0.03)                         # computes delta time (time from the last frame), limited by 30ms to prevent object teleportation
    boxes = []
    for b in bodies:                                # movement + wall collision
        e, v, h = b['e'], b['v'], b['h']
        e.position += v * dt                        # moves object along its velocity vector
        p = [e.x, e.y, e.z]; vel = [v.x, v.y, v.z]
        # wall collision detection
        for k in range(3):
            lim = HALF - h[k]
            if abs(p[k]) > lim:
                p[k] = lim if p[k] > 0 else -lim
                vel[k] = -vel[k]
        e.position = Vec3(*p); b['v'] = Vec3(*vel)
        boxes.append((p[0]-h[0], p[1]-h[1], p[2]-h[2], p[0]+h[0], p[1]+h[1], p[2]+h[2])) # prepares object's bounding box for detection algorithms
        # for every object we subtract/add half of measures (h) from/to its center

    name = algo_names[current]
    t = time.perf_counter()                       # retrieves precise timestamp before computing
    pairs, checks = ALGORITHMS[name](boxes)       # computes detection algorithm and returns pairs and amount of checks
    ms = (time.perf_counter() - t) * 1000         # calculates amount of time that have passed while algorithm was running
    avg_ms[name] = ms if avg_ms[name] is None else 0.9 * avg_ms[name] + 0.1 * ms # EMA (Exponential Moving Average) - smoothing out the averages, so it gives any valuable result

    hit = set()
    for i, j in pairs:
        hit.add(i); hit.add(j)
    for i, b in enumerate(bodies):               # color changes only when state has changed
        now = i in hit
        if now != b['hit']:
            b['e'].color = color.red if now else color.lime     # lime - safe, red - collision
            b['hit'] = now

    lines = [
        f"OBJECTS: {len(bodies):<4}  |  COLLISIONS: {len(pairs):<6}  |  CHECKS: {checks}",
        f"ALGORITHM:  {name:<18}  (buttons 1/2/3)",
        f"CONTROLS:  +/- number of objects",
        "=" * 50,
    ]
    for k, n in enumerate(algo_names):
        val = "    --    " if avg_ms[n] is None else f"{avg_ms[n]:8.2f} ms"
        marker = ">>>" if k == current else "   "
        lines.append(f"{marker} {k + 1}. {n:<16} {val}")
    hud.text = "\n".join(lines)


app.run()