"""
Ultimate Single-File 2D Simulation Engine (SAT polygon collisions)
Author: Dhanwanth (updated)
Features:
 - Spatial hash broadphase (fast)
 - Frame-friendly memory usage (minimal temporaries)
 - Semi-implicit Euler & Velocity Verlet integration (toggle with V)
 - Circle-circle, circle-polygon, polygon-polygon collisions using SAT
 - Object pooling for short-lived particles
 - Debug HUD, grid overlay, profiler
 - Recording frames (PNG sequence)
 - Example constraints (spring)
 - Runtime hotkeys: G,H,Q,V,P,R,+,-,Esc
"""

import pygame
import random
import math
import time
import os
from collections import deque
from typing import List, Tuple

# ---------------------------
# ========== CONFIG =========
# ---------------------------
WIDTH, HEIGHT = 1200, 700
FPS_TARGET = 60

# initial scene parameters
NUM_BALLS = 200
MIN_RADIUS, MAX_RADIUS = 4, 10
AUTO_CELL_FACTOR = 3         # cell_size = AUTO_CELL_FACTOR * max_radius

# physics defaults
GLOBAL_GRAVITY = 0.0
USE_SPATIAL_HASH_DEFAULT = True
USE_OBJECT_POOL = True
POOL_SIZE = 2000

# recording frames
FRAME_DIR = "frames"
RECORD_DEFAULT = False

class Shape:
    CIRCLE = 'circle'
    POLYGON = 'polygon'
    AABB = 'aabb'   # optional, can map to polygon

# ---------------------------
# ======= UTILITIES =========
# ---------------------------
SQRT = math.sqrt
INV_2PI = 1.0 / (2 * math.pi)

def clamp(x, a, b): return a if x < a else (b if x > b else x)

# 2D vector helpers using tuples for SAT where convenient
def dot2(a, b): return a[0]*b[0] + a[1]*b[1]
def sub2(a, b): return (a[0]-b[0], a[1]-b[1])
def perp2(v): return (-v[1], v[0])
def len2(v): return math.hypot(v[0], v[1])
def normalize2(v):
    L = len2(v)
    if L == 0: return (0.0, 0.0)
    return (v[0]/L, v[1]/L)

def support_polygon(verts, direction):
    """Support function for polygon: vertex with max dot product with direction."""
    best = verts[0]
    best_dot = dot2(best, direction)
    for v in verts[1:]:
        d = dot2(v, direction)
        if d > best_dot:
            best = v
            best_dot = d
    return best

def support_circle(center, radius, direction):
    """Support function for circle: center + radius * normalized direction."""
    length = len2(direction)
    if length == 0:
        return center
    norm_dir = (direction[0]/length, direction[1]/length)
    return (center[0] + norm_dir[0]*radius, center[1] + norm_dir[1]*radius)

def support(shapeA, shapeB, direction):
    """Support function for Minkowski difference of shapeA and shapeB."""
    # shapeA and shapeB are dicts with 'verts' or 'circle' keys
    if 'verts' in shapeA:
        p1 = support_polygon(shapeA['verts'], direction)
    else:
        p1 = support_circle(shapeA['center'], shapeA['radius'], direction)
    neg_dir = (-direction[0], -direction[1])
    if 'verts' in shapeB:
        p2 = support_polygon(shapeB['verts'], neg_dir)
    else:
        p2 = support_circle(shapeB['center'], shapeB['radius'], neg_dir)
    return (p1[0] - p2[0], p1[1] - p2[1])

def triple_product(a, b, c):
    # triple product of vectors a,b,c = b*(a·c) - c*(a·b)
    ac = dot2(a, c)
    ab = dot2(a, b)
    return (b[0]*ac - c[0]*ab, b[1]*ac - c[1]*ab)

def gjk_collision(shapeA, shapeB):
    direction = [1, 0]
    simplex = []
    p = support(shapeA, shapeB, direction)
    simplex.append(p)
    direction[0], direction[1] = -p[0], -p[1]
    for _ in range(30):
        p = support(shapeA, shapeB, direction)
        if dot2(p, direction) <= 0:
            return False, simplex
        simplex.append(p)
        if handle_simplex(simplex, direction):
            return True, simplex
    return False, simplex
def handle_simplex(simplex, direction):
    if len(simplex) == 2:
        a = simplex[-1]
        b = simplex[-2]
        ab = sub2(b, a)
        ao = (-a[0], -a[1])
        ab_perp = triple_product(ab, ao, ab)
        if len2(ab_perp) < 1e-6:
            ab_perp = perp2(ab)
        direction[0], direction[1] = ab_perp
        return False
    elif len(simplex) == 3:
        a = simplex[-1]
        b = simplex[-2]
        c = simplex[-3]
        ab = sub2(b, a)
        ac = sub2(c, a)
        ao = (-a[0], -a[1])
        ab_perp = triple_product(ac, ab, ab)
        ac_perp = triple_product(ab, ac, ac)
        if dot2(ab_perp, ao) > 0:
            simplex.pop(-3)  # remove c
            direction[0], direction[1] = ab_perp
            return False
        elif dot2(ac_perp, ao) > 0:
            simplex.pop(-2)  # remove b
            direction[0], direction[1] = ac_perp
            return False
        else:
            return True
    return False

def epa(shapeA, shapeB, simplex):
    """EPA algorithm to find penetration depth and normal."""
    polytope = simplex[:]
    edges = []
    for _ in range(30):
        # Find edge closest to origin
        min_dist = float('inf')
        closest_edge = None
        closest_index = -1
        for i in range(len(polytope)):
            j = (i + 1) % len(polytope)
            a = polytope[i]
            b = polytope[j]
            edge = sub2(b, a)
            normal = normalize2(perp2(edge))
            dist = dot2(normal, a)
            if dist < min_dist:
                min_dist = dist
                closest_edge = normal
                closest_index = j
        # Get support point in direction of normal
        p = support(shapeA, shapeB, closest_edge)
        d = dot2(p, closest_edge)
        if d - min_dist < 1e-6:
            # Penetration depth and normal found
            return min_dist, closest_edge
        else:
            polytope.insert(closest_index, p)
    # Failed to converge
    return 0, (0,0)

# ---------------------------
# === SPATIAL HASH GRID ====
# ---------------------------
class SpatialHashGrid:
    """High-performance spatial hash using preallocated cells and neighbor cache."""
    __slots__ = ("width","height","cell_size","cols","rows","cells","neighbor_cache")
    def __init__(self, width:int, height:int, cell_size:int):
        self.width = width
        self.height = height
        self.cell_size = max(4, int(cell_size))
        self.cols = max(1, (self.width + self.cell_size - 1) // self.cell_size)
        self.rows = max(1, (self.height + self.cell_size - 1) // self.cell_size)
        total = self.cols * self.rows
        self.cells = [[] for _ in range(total)]
        self.neighbor_cache = [None] * total
        for cy in range(self.rows):
            for cx in range(self.cols):
                idx = cy * self.cols + cx
                neigh = []
                y0 = max(0, cy - 1)
                y1 = min(self.rows - 1, cy + 1)
                x0 = max(0, cx - 1)
                x1 = min(self.cols - 1, cx + 1)
                for ny in range(y0, y1 + 1):
                    base = ny * self.cols
                    for nx in range(x0, x1 + 1):
                        neigh.append(base + nx)
                self.neighbor_cache[idx] = tuple(neigh)

    def clear(self):
        for lst in self.cells:
            lst.clear()

    def insert(self, ent):
        cx = int(ent.x // self.cell_size)
        cy = int(ent.y // self.cell_size)
        if cx < 0: cx = 0
        elif cx >= self.cols: cx = self.cols - 1
        if cy < 0: cy = 0
        elif cy >= self.rows: cy = self.rows - 1
        idx = cy * self.cols + cx
        self.cells[idx].append(ent)

    def get_neighbors_indices(self, ent):
        cx = int(ent.x // self.cell_size)
        cy = int(ent.y // self.cell_size)
        if cx < 0: cx = 0
        elif cx >= self.cols: cx = self.cols - 1
        if cy < 0: cy = 0
        elif cy >= self.rows: cy = self.rows - 1
        idx = cy * self.cols + cx
        return self.neighbor_cache[idx]

# ---------------------------
# ======== ENTITIES =========
# ---------------------------
class Entity:
    """
    Extended entity to optionally carry polygon vertex list (local-space),
    rotation (angle), and angular_velocity. If verts is None -> circle entity.
    """
    __slots__ = ("x","y","vx","vy","ax","ay","radius","mass","color",
                 "restitution","friction","alive","tag","verts","angle","angular_velocity","shape","inertia","inv_inertia")
    def __init__(self, x:float=0.0, y:float=0.0, radius:float=6.0, color:Tuple[int,int,int]=(200,200,200), mass:float=None, verts:List[Tuple[float,float]]=None):
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.radius = float(radius)            # bounding circle radius (for broadphase)
        self.mass = float(mass if mass is not None else max(0.1, self.radius * 0.6))
        self.color = color
        self.restitution = 0.9
        self.friction = 0.999
        self.alive = True
        self.tag = None
        # polygon data (local-space verts)
        self.verts = None
        self.angle = 0.0
        self.angular_velocity = 0.0
        if verts:
            # set polygon; recompute radius as bounding circle
            self.verts = [ (float(v[0]), float(v[1])) for v in verts ]
            self.radius = max( math.hypot(v[0], v[1]) for v in self.verts ) + 1e-6
            self.shape = Shape.POLYGON
            self.angular_velocity = random.uniform(-0.05, 0.05)
        else:
            self.shape = Shape.CIRCLE
            if self.shape == Shape.CIRCLE:
                self.inertia = 0.5 * self.mass * self.radius * self.radius
            elif self.shape == Shape.POLYGON:
                xs = [v[0] for v in self.verts]
                ys = [v[1] for v in self.verts]
                w = max(xs) - min(xs)
                h = max(ys) - min(ys)
                self.inertia = (1/12) * self.mass * (w*w + h*h)
            else:
                self.inertia = 1.0  # fallback
            self.inv_inertia = 0.0 if self.inertia == 0 else 1.0 / self.inertia

    def apply_force(self, fx:float, fy:float):
        self.ax += fx / self.mass
        self.ay += fy / self.mass

    def integrate_euler(self, engine):
        """Semi-implicit Euler integration (v then x). Includes simple angular update."""
        # apply gravity (acceleration units assumed)
        self.ay += engine.gravity
        # integrate velocity
        self.vx += self.ax
        self.vy += self.ay
        # integrate position
        self.x += self.vx
        self.y += self.vy
        # angular
        self.angle += self.angular_velocity
        # reset acceleration
        self.ax = 0.0
        self.ay = 0.0
        # damping
        self.vx *= self.friction
        self.vy *= self.friction
        self.angular_velocity *= 0.999
        # boundary collisions (contain in screen)
        r = self.radius
        if self.x - r < 0.0:
            self.x = r
            self.vx = -self.vx * self.restitution
        elif self.x + r > engine.width:
            self.x = engine.width - r
            self.vx = -self.vx * self.restitution
        if self.y - r < 0.0:
            self.y = r
            self.vy = -self.vy * self.restitution
        elif self.y + r > engine.height:
            self.y = engine.height - r
            self.vy = -self.vy * self.restitution

    # Verlet we use previous positions stored in physics world
    def integrate_verlet(self, prev_x, prev_y, engine):
        # acceleration includes gravity (engine.gravity)
        ax = self.ax + engine.gravity # per-mass already in apply_force design not used here
        ay = self.ay + engine.gravity
        new_x = 2.0 * self.x - prev_x + ax
        new_y = 2.0 * self.y - prev_y + ay
        # update vx/vy estimate
        self.vx = (new_x - prev_x) * 0.5
        self.vy = (new_y - prev_y) * 0.5
        # angular: simple integration from angular_velocity
        self.angle += self.angular_velocity
        # reset accel
        self.ax = 0.0
        self.ay = 0.0
        # friction/damping applied to velocity
        self.vx *= self.friction
        self.vy *= self.friction
        self.angular_velocity *= 0.999
        # boundary handling
        r = self.radius
        if new_x - r < 0.0:
            new_x = r
            self.vx = -self.vx * self.restitution
        elif new_x + r > engine.width:
            new_x = engine.width - r
            self.vx = -self.vx * self.restitution
        if new_y - r < 0.0:
            new_y = r
            self.vy = -self.vy * self.restitution
        elif new_y + r > engine.height:
            new_y = engine.height - r
            self.vy = -self.vy * self.restitution
        return new_x, new_y

    def local_to_world_verts(self):
        """Return list of world-space polygon vertices (rotated & translated)."""
        if not self.verts:
            return []
        ca = math.cos(self.angle)
        sa = math.sin(self.angle)
        pts = []
        for vx, vy in self.verts:
            wx = self.x + vx * ca - vy * sa
            wy = self.y + vx * sa + vy * ca
            pts.append((wx, wy))
        return pts

    def draw(self, surf, outline=True):
        if self.shape == Shape.CIRCLE:
            pygame.draw.circle(surf, self.color, (int(self.x), int(self.y)), int(self.radius))
            if outline:
                pygame.draw.circle(surf, (0,0,0), (int(self.x), int(self.y)), int(self.radius), 1)
        else:
            pts = self.local_to_world_verts()
            if len(pts) >= 3:
                pygame.draw.polygon(surf, self.color, [(int(px), int(py)) for px,py in pts])
                if outline:
                    pygame.draw.polygon(surf, (0,0,0), [(int(px), int(py)) for px,py in pts], 1)

# ---------------------------
# ======= OBJECT POOL =======
# ---------------------------
class ObjectPool:
    """Simple pool for Entity objects to avoid frequent allocation."""
    def __init__(self, cls, size:int, *args, **kwargs):
        self.cls = cls
        self.pool = deque()
        self.args = args
        self.kwargs = kwargs
        for _ in range(size):
            obj = cls(*args, **kwargs)
            obj.alive = False
            self.pool.append(obj)
    def acquire(self, **overrides):
        if self.pool:
            obj = self.pool.pop()
            obj.alive = True
            # reset important fields
            obj.x = overrides.get('x', obj.x)
            obj.y = overrides.get('y', obj.y)
            obj.vx = overrides.get('vx', 0.0)
            obj.vy = overrides.get('vy', 0.0)
            obj.ax = 0.0; obj.ay = 0.0
            obj.radius = overrides.get('radius', obj.radius)
            obj.color = overrides.get('color', obj.color)
            obj.mass = overrides.get('mass', obj.mass)
            # ensure circle type
            obj.verts = None
            obj.shape = Shape.CIRCLE
            return obj
        else:
            obj = self.cls(**self.kwargs)
            obj.alive = True
            return obj
    def release(self, obj):
        obj.alive = False
        # reset polygon-specific fields to safe defaults
        obj.verts = None
        obj.shape = Shape.CIRCLE
        obj.angle = 0.0
        obj.angular_velocity = 0.0
        self.pool.append(obj)

# ---------------------------
# ======= SPRING (EX) =======
# ---------------------------
class Spring:
    __slots__ = ("a","b","rest_length","k","damping")
    def __init__(self, a:Entity, b:Entity, k:float=0.2, rest_length:float=None, damping:float=0.02):
        self.a = a; self.b = b; self.k = k; self.damping = damping
        self.rest_length = rest_length if rest_length is not None else math.hypot(b.x-a.x, b.y-a.y)
    def apply(self):
        dx = self.b.x - self.a.x
        dy = self.b.y - self.a.y
        dist = math.hypot(dx,dy) if dx*dx+dy*dy != 0 else 1e-6
        nx = dx / dist; ny = dy / dist
        fs = self.k * (dist - self.rest_length)
        rv = (self.b.vx - self.a.vx) * nx + (self.b.vy - self.a.vy) * ny
        fd = self.damping * rv
        fx = (fs + fd) * nx
        fy = (fs + fd) * ny
        self.a.apply_force(fx, fy)
        self.b.apply_force(-fx, -fy)

# -----------------------
# ===== COLLISIONS ======
# -----------------------
def project_polygon_axis(axis, verts):
    """Project polygon verts (list of (x,y)) onto axis (normalized) -> min,max scalars."""
    minp = dot2(verts[0], axis); maxp = minp
    for v in verts[1:]:
        p = dot2(v, axis)
        if p < minp: minp = p
        if p > maxp: maxp = p
    return minp, maxp

def interval_distance(minA, maxA, minB, maxB):
    if minA < minB:
        return minB - maxA
    else:
        return minA - maxB


# ---------------------------
# ===== PHYSICS WORLD =======
# ---------------------------
class PhysicsWorld:
    """
    Manage objects, integrator switching (euler/verlet), spatial hash broadphase,
    collision resolution (now SAT for polygons), constraints, pooling.
    """
    __slots__ = ("width","height","gravity","entities","springs",
                 "use_spatial_hash","grid","max_radius","cell_size",
                 "use_pool","pool","integrator","prev_positions",
                 "collision_pairs_checked","time_profile")

    def __init__(self, width:int, height:int, max_radius:float=MAX_RADIUS,
                 use_spatial_hash:bool=True, use_pool:bool=True, integrator:str="euler"):
        self.width = width
        self.height = height
        self.gravity = GLOBAL_GRAVITY
        self.entities: List[Entity] = []
        self.springs: List[Spring] = []
        self.use_spatial_hash = use_spatial_hash
        self.max_radius = max_radius
        self.cell_size = max(4, int(AUTO_CELL_FACTOR * max_radius))
        self.grid = SpatialHashGrid(self.width, self.height, self.cell_size)
        self.use_pool = use_pool
        self.pool = ObjectPool(Entity, POOL_SIZE) if use_pool else None
        self.integrator = integrator  # "euler" or "verlet"
        self.prev_positions = {}  # entity -> (prev_x, prev_y)
        self.collision_pairs_checked = 0
        self.time_profile = {"integrate":0.0, "broadphase":0.0, "resolve":0.0, "constraints":0.0}

    def add_entity(self, ent:Entity):
        self.entities.append(ent)
        self.prev_positions[ent] = (ent.x - ent.vx, ent.y - ent.vy)

    def spawn_ball(self, x:float, y:float, radius:float, color=None):
        if self.use_pool:
            e = self.pool.acquire(x=x, y=y, radius=radius, color=(random.randint(60,255),random.randint(60,255),random.randint(60,255)))
        else:
            e = Entity(x, y, radius)
            e.color = color if color is not None else (random.randint(60,255),random.randint(60,255),random.randint(60,255))
        e.vx = random.uniform(-1.6, 1.6)
        e.vy = random.uniform(-1.6, 1.6)
        e.restitution = 0.92
        e.friction = 0.999
        e.mass = max(0.1, radius * 0.6)
        self.add_entity(e)
        return e

    def spawn_polygon(self, x:float, y:float, local_verts:List[Tuple[float,float]], color=None, mass:float=None):
        # create polygon entity (no pooling for polygons)
        bounding = max(math.hypot(v[0], v[1]) for v in local_verts) + 1e-6
        e = Entity(x, y, radius=bounding, color=color if color is not None else (random.randint(60,255),random.randint(60,255),random.randint(60,255)), mass=mass)
        e.verts = [ (float(v[0]), float(v[1])) for v in local_verts ]
        e.shape = Shape.POLYGON
        e.angular_velocity = random.uniform(-0.04, 0.04)
        self.add_entity(e)
        return e

    def remove_entity(self, ent:Entity):
        try:
            self.entities.remove(ent)
        except ValueError:
            pass
        if self.use_pool and self.pool:
            self.pool.release(ent)
        if ent in self.prev_positions:
            del self.prev_positions[ent]

    def add_spring(self, a:Entity, b:Entity, k=0.2, rest=None, damping=0.02):
        s = Spring(a,b,k,rest,damping)
        self.springs.append(s)
        return s

    def step(self):
        t0 = time.perf_counter()
        self.time_profile["integrate"] = 0.0
        self.time_profile["broadphase"] = 0.0
        self.time_profile["resolve"] = 0.0
        self.time_profile["constraints"] = 0.0

        tA = time.perf_counter()
        if self.integrator == "euler":
            for e in self.entities:
                if e.alive:
                    e.integrate_euler(self)
        else:
            for e in self.entities:
                if e.alive:
                    prev = self.prev_positions.get(e, (e.x - e.vx, e.y - e.vy))
                    new_x, new_y = e.integrate_verlet(prev[0], prev[1], self)
                    self.prev_positions[e] = (e.x, e.y)
                    e.x = new_x; e.y = new_y
        tB = time.perf_counter()
        self.time_profile["integrate"] = tB - tA

        tC = time.perf_counter()
        if self.springs:
            for s in self.springs:
                s.apply()
        tD = time.perf_counter()
        self.time_profile["constraints"] = tD - tC

        tE = time.perf_counter()
        if self.use_spatial_hash:
            self._broadphase_spatial()
        else:
            self._broadphase_naive()
        tF = time.perf_counter()
        self.time_profile["broadphase"] = tF - tE

        self.collision_pairs_checked = 0

    # Broadphase spatial
    def _broadphase_spatial(self):
        grid = self.grid
        grid.clear()
        for e in self.entities:
            if e.alive:
                grid.insert(e)
        checked = set()
        tR0 = time.perf_counter()
        for idx, cell in enumerate(grid.cells):
            if not cell:
                continue
            neigh_idxs = grid.neighbor_cache[idx]
            for a in cell:
                if not a.alive: continue
                for nidx in neigh_idxs:
                    other_list = grid.cells[nidx]
                    if not other_list: continue
                    for b in other_list:
                        if a is b or not b.alive: continue
                        aid = id(a); bid = id(b)
                        pair = (aid,bid) if aid < bid else (bid,aid)
                        if pair in checked: continue
                        checked.add(pair)
                        # quick circle-bound test
                        dx = b.x - a.x; dy = b.y - a.y
                        dist2 = dx*dx + dy*dy
                        rsum = a.radius + b.radius
                        if dist2 < (rsum * rsum):
                            # precise SAT / resolve
                            self._resolve_collision_pair(a,b)
        tR1 = time.perf_counter()
        self.time_profile["resolve"] = tR1 - tR0
        self.collision_pairs_checked = len(checked)

    # Naive broadphase
    def _broadphase_naive(self):
        n = len(self.entities)
        tR0 = time.perf_counter()
        for i in range(n):
            a = self.entities[i]
            if not a.alive: continue
            for j in range(i+1, n):
                b = self.entities[j]
                if not b.alive: continue
                dx = b.x - a.x; dy = b.y - a.y
                dist2 = dx*dx + dy*dy
                rsum = a.radius + b.radius
                if dist2 < (rsum*rsum):
                    self._resolve_collision_pair(a,b)
        tR1 = time.perf_counter()
        self.time_profile["resolve"] = tR1 - tR0

    # Collision resolution using SAT if polygons involved

    def _resolve_collision_pair(self, a:Entity, b:Entity):
        # Prepare shapes for GJK
        def get_shape(ent):
            if ent.shape == Shape.CIRCLE:
                return {'center': (ent.x, ent.y), 'radius': ent.radius}
            else:
                verts = ent.local_to_world_verts()
                return {'verts': verts}

        shapeA = get_shape(a)
        shapeB = get_shape(b)

        collision, simplex = gjk_collision(shapeA, shapeB)
        if not collision:
            return

        penetration, normal = epa(shapeA, shapeB, simplex)
        if penetration <= 0:
            return

        nx, ny = normal

        # Approximate contact point:
        if a.shape == Shape.CIRCLE and b.shape == Shape.CIRCLE:
            dx = b.x - a.x
            dy = b.y - a.y
            dist = math.hypot(dx, dy)
            if dist == 0:
                cx, cy = a.x, a.y
            else:
                cx = a.x + nx * (a.radius - penetration * 0.5)
                cy = a.y + ny * (a.radius - penetration * 0.5)
        else:
            # For polygon involved, approximate contact point as midpoint between centers
            cx = (a.x + b.x) * 0.5
            cy = (a.y + b.y) * 0.5

        # Vectors from centers to contact point
        rA = (cx - a.x, cy - a.y)
        rB = (cx - b.x, cy - b.y)

        # Relative velocity at contact point
        # v + w x r (2D cross product: w x r = (-w * r.y, w * r.x))
        velA = (a.vx - a.angular_velocity * rA[1], a.vy + a.angular_velocity * rA[0])
        velB = (b.vx - b.angular_velocity * rB[1], b.vy + b.angular_velocity * rB[0])
        rv = (velB[0] - velA[0], velB[1] - velA[1])

        vel_along_normal = rv[0]*nx + rv[1]*ny
        if vel_along_normal > 0:
            return

        e = min(a.restitution, b.restitution)
        invMassA = 0.0 if a.mass == 0 else 1.0 / a.mass
        invMassB = 0.0 if b.mass == 0 else 1.0 / b.mass
        invInertiaA = a.inv_inertia
        invInertiaB = b.inv_inertia

        # Calculate cross products r x n
        crossA = rA[0]*ny - rA[1]*nx
        crossB = rB[0]*ny - rB[1]*nx

        denom = invMassA + invMassB + (crossA*crossA)*invInertiaA + (crossB*crossB)*invInertiaB
        if denom == 0:
            return

        j = -(1 + e) * vel_along_normal / denom

        impulse = (j*nx, j*ny)

        # Apply linear impulse
        a.vx -= impulse[0] * invMassA
        a.vy -= impulse[1] * invMassA
        b.vx += impulse[0] * invMassB
        b.vy += impulse[1] * invMassB

        # Apply angular impulse
        a.angular_velocity -= crossA * j * invInertiaA
        b.angular_velocity += crossB * j * invInertiaB

        # Positional correction to avoid sinking
        percent = 0.2  # usually 20% to 80%
        slop = 0.01    # penetration allowance
        correction_mag = max(penetration - slop, 0) / (invMassA + invMassB) * percent
        correction = (correction_mag * nx, correction_mag * ny)
        a.x -= correction[0] * invMassA
        a.y -= correction[1] * invMassA
        b.x += correction[0] * invMassB
        b.y += correction[1] * invMassB



# ---------------------------
# ======== RENDERER =========
# ---------------------------
class Renderer:
    def __init__(self, screen, world:PhysicsWorld,clock):
        self.screen = screen
        self.world = world
        self.clock = clock
        self.font = pygame.font.SysFont(None, 20)

    def draw(self, draw_grid=False, draw_hud=True):
        for e in self.world.entities:
            if e.alive:
                e.draw(self.screen)

        if draw_grid:
            self._draw_grid_overlay()
        if draw_hud:
            self._draw_hud()

        # Draw buttons if Engine has them
        if hasattr(self.world, 'engine') and self.world.engine.buttons:
            for btn in self.world.engine.buttons:
                btn.draw(self.screen)


    def _draw_grid_overlay(self):
        cs = self.world.grid.cell_size
        cols = self.world.grid.cols
        rows = self.world.grid.rows
        for cx in range(1, cols):
            x = cx * cs
            pygame.draw.line(self.screen, (30,30,30), (x,0), (x,self.world.height))
        for cy in range(1, rows):
            y = cy * cs
            pygame.draw.line(self.screen, (30,30,30), (0,y), (self.world.width,y))

    def _draw_hud(self):
            w = self.world
            fps = int(self.clock.get_fps())
            text_lines = [
                f"Entities: {len(w.entities)}   Integrator: {w.integrator.upper()}   SpatialHash: {w.use_spatial_hash}",
                f"CellSize: {w.grid.cell_size}   PairsChecked: {w.collision_pairs_checked}",
                f"Integrate: {w.time_profile['integrate']*1000:.2f}ms  Broad: {w.time_profile['broadphase']*1000:.2f}ms  Resolve: {w.time_profile['resolve']*1000:.2f}ms  Constraints: {w.time_profile['constraints']*1000:.2f}ms",
                f"Gravity: {round(w.gravity,2)}  Pool: {'On' if w.use_pool else 'Off'}",
                f"FPS: {fps}"
            ]
            y = 6
            for line in text_lines:
                surf = self.font.render(line, True, (220,220,220))
                self.screen.blit(surf, (6,y))
                y += 18

class Button:
    def __init__(self, rect, text, font, bg_color=(50,50,50), fg_color=(220,220,220)):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.font = font
        self.bg_color = bg_color
        self.fg_color = fg_color
        self.surface = self.font.render(self.text, True, self.fg_color)
        self.surface_rect = self.surface.get_rect(center=self.rect.center)
    def draw(self, surf):
        pygame.draw.rect(surf, self.bg_color, self.rect, border_radius=6)
        pygame.draw.rect(surf, (200,200,200), self.rect, 2, border_radius=6)
        surf.blit(self.surface, self.surface_rect)
    def is_pressed(self, pos):
        return self.rect.collidepoint(pos)

def regular_polygon_vertices(sides: int, radius: float) -> List[Tuple[float, float]]:
    verts = []
    for i in range(sides):
        angle = 2 * math.pi * i / sides - math.pi / 2  # start pointing up
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        verts.append((x, y))
    return verts


# ---------------------------
# ======== ENGINE = =========
# ---------------------------

class Engine:
    def __init__(self):
        pygame.init()
        self.width = WIDTH
        self.height = HEIGHT
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Ultimate 2D Simulation Engine (SAT)")
        self.clock = pygame.time.Clock()
        self.world = PhysicsWorld(self.width, self.height, max_radius=MAX_RADIUS,
                                  use_spatial_hash=USE_SPATIAL_HASH_DEFAULT, use_pool=USE_OBJECT_POOL,
                                  integrator="euler")
        self._spawn_initial()
        self.renderer = Renderer(self.screen, self.world, self.clock)

        # Initialize font and buttons for GUI controls
        self.font = pygame.font.SysFont(None, 24)
        btn_w, btn_h = 90, 32
        margin = 6
        screen_w, screen_h = self.width, self.height

        self.buttons = [
            Button((margin, screen_h - btn_h - margin, btn_w, btn_h), "Grid (G)", self.font),
            Button((margin*2 + btn_w, screen_h - btn_h - margin, btn_w, btn_h), "HUD (H)", self.font),
            Button((margin*10 + btn_w*11, screen_h - btn_h - margin, btn_w+50, btn_h), "Integrator (V)", self.font),
            Button((margin*3 + btn_w*2, screen_h - btn_h - margin, btn_w, btn_h), "Pause (P)", self.font),
            Button((margin*4 + btn_w*3, screen_h - btn_h - margin, btn_w, btn_h), "Record (R)", self.font),
            Button((margin*5 + btn_w*4, screen_h - btn_h - margin, btn_w, btn_h), "+100 (+)", self.font),
            Button((margin*6 + btn_w*5, screen_h - btn_h - margin, btn_w, btn_h), "-100 (-)", self.font),
            Button((margin*7 + btn_w*6, screen_h - btn_h - margin, btn_w, btn_h), "Clear (C)", self.font),
            Button((margin*8 + btn_w*7, screen_h - btn_h - margin, btn_w + 50, btn_h), "Grav + (Up)", self.font),
            Button((margin*9 + btn_w*8+50, screen_h - btn_h - margin, btn_w + 50, btn_h), "Grav - (Down)", self.font),
        ]

        self.running = True
        self.draw_grid = False
        self.draw_hud = True
        self.paused = False
        self.recording = RECORD_DEFAULT
        self.frame_count = 0

    def _spawn_initial(self):
        margin = MAX_RADIUS + 16
        # spawn some circles
        for _ in range(NUM_BALLS):
            r = random.randint(MIN_RADIUS, MAX_RADIUS)
            x = random.uniform(margin, self.width - margin)
            y = random.uniform(margin, self.height - margin)
            self.world.spawn_ball(x,y,r)
        # spawn some polygons for demonstration
        # triangle
        self.world.spawn_polygon(300, 200, [(0,-18),(16,10),(-16,10)], color=(200,120,120), mass=4.0)
        # square
        self.world.spawn_polygon(500, 180, [(-18,-18),(18,-18),(18,18),(-18,18)], color=(120,200,200), mass=6.0)
        # pentagon
        poly = []
        for i in range(5):
            a = 2*math.pi*i/5
            poly.append((math.cos(a)*20, math.sin(a)*20))
        self.world.spawn_polygon(400, 120, poly, color=(180,180,90), mass=5.0)

    def run(self):
        if self.recording and not os.path.exists(FRAME_DIR):
            os.makedirs(FRAME_DIR)
        fps_target = FPS_TARGET
        while self.running:
            t0 = time.perf_counter()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False
                elif ev.type == pygame.KEYDOWN:
                    self._on_key(ev.key)
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    # Check if any button was pressed
                    for btn in self.buttons:
                        if btn.is_pressed((mx,my)):
                            self._on_button(btn.text)
                            break
                    else:
                        # If no button pressed, spawn balls on left click
                        if ev.button == 1:
                            for _ in range(12):
                                r = random.randint(3,7)
                                e = self.world.spawn_ball(mx + random.uniform(-8,8), my + random.uniform(-8,8), r)
                                e.vx = random.uniform(-3,3)
                                e.vy = random.uniform(-3,3)
            if not self.paused:
                self.world.step()
            self.screen.fill((18,18,24))
            self.renderer.draw(draw_grid=self.draw_grid, draw_hud=self.draw_hud)

            # Draw GUI buttons on top
            for btn in self.buttons:
                btn.draw(self.screen)

            pygame.display.flip()
            if self.recording:
                self._record_frame()
            self.clock.tick(fps_target)

        pygame.quit()

    def _on_key(self, key):
        # Keep your existing key handling here if you want keyboard support
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_g:
            self.draw_grid = not self.draw_grid
        elif key == pygame.K_h:
            self.draw_hud = not self.draw_hud
        elif key == pygame.K_q:
            self.world.use_spatial_hash = not self.world.use_spatial_hash
            print("Spatial Hash:", self.world.use_spatial_hash)
        elif key == pygame.K_v:
            self.world.integrator = "verlet" if self.world.integrator == "euler" else "euler"
            if self.world.integrator == "verlet":
                for e in self.world.entities:
                    self.world.prev_positions[e] = (e.x - e.vx, e.y - e.vy)
            print("Integrator:", self.world.integrator)
        elif key == pygame.K_p:
            self.paused = not self.paused
        elif key == pygame.K_r:
            self.recording = not self.recording
            if self.recording:
                if not os.path.exists(FRAME_DIR):
                    os.makedirs(FRAME_DIR)
                print("Recording frames to", FRAME_DIR)
        elif key == pygame.K_PLUS or key == pygame.K_EQUALS:
            for _ in range(100):
                r = random.randint(MIN_RADIUS, MAX_RADIUS)
                x = random.uniform(16 + r, self.width - 16 - r)
                y = random.uniform(16 + r, self.height - 16 - r)
                self.world.spawn_ball(x,y,r)
            print("Spawned +100, total:", len(self.world.entities))
        elif key == pygame.K_MINUS or key == pygame.K_UNDERSCORE:
            toremove = min(100, len(self.world.entities))
            for _ in range(toremove):
                ent = self.world.entities.pop()
                if self.world.use_pool and self.world.pool:
                    self.world.pool.release(ent)
            print("Removed -100, total:", len(self.world.entities))
        elif key == pygame.K_c:
            self._clear_and_respawn(100)
            print("Cleared & respawned 100")
        elif key == pygame.K_s:
            self.world.use_spatial_hash = not self.world.use_spatial_hash
            print("Spatial hash:", self.world.use_spatial_hash)
        elif key == pygame.K_t:
            self.world.use_pool = not self.world.use_pool
            print("Pool:", "On" if self.world.use_pool else "Off")
        elif key == pygame.K_UP:
            self.world.gravity += 0.1
            print("Gravity:", self.world.gravity)
        elif key == pygame.K_DOWN:
            self.world.gravity -= 0.1
            print("Gravity:", self.world.gravity)

        elif pygame.K_3 <= key <= pygame.K_9:
            sides = key - pygame.K_0  # converts key to int 3..9
            radius = 20
            x = random.uniform(50 + radius, self.width - 50 - radius)
            y = random.uniform(50 + radius, self.height - 50 - radius)
            verts = regular_polygon_vertices(sides, radius)
            self.world.spawn_polygon(x, y, verts, color=(random.randint(60,255),random.randint(60,255),random.randint(60,255)), mass=radius * sides * 0.5)
            print(f"Spawned regular polygon with {sides} sides")


    def _on_button(self, label):
        # Map button label to actions
        if "Grid" in label:
            self.draw_grid = not self.draw_grid
            print("Grid toggled:", self.draw_grid)
        elif "HUD" in label:
            self.draw_hud = not self.draw_hud
            print("HUD toggled:", self.draw_hud)
        elif "Integrator" in label:
            self.world.integrator = "verlet" if self.world.integrator == "euler" else "euler"
            if self.world.integrator == "verlet":
                for e in self.world.entities:
                    self.world.prev_positions[e] = (e.x - e.vx, e.y - e.vy)
            print("Integrator:", self.world.integrator)
        elif "Pause" in label:
            self.paused = not self.paused
            print("Paused:", self.paused)
        elif "Record" in label:
            self.recording = not self.recording
            if self.recording and not os.path.exists(FRAME_DIR):
                os.makedirs(FRAME_DIR)
            print("Recording:", self.recording)
        elif "+100" in label:
            for _ in range(100):
                r = random.randint(MIN_RADIUS, MAX_RADIUS)
                x = random.uniform(16 + r, self.width - 16 - r)
                y = random.uniform(16 + r, self.height - 16 - r)
                self.world.spawn_ball(x,y,r)
            print("Spawned +100, total:", len(self.world.entities))
        elif "-100" in label:
            toremove = min(100, len(self.world.entities))
            for _ in range(toremove):
                ent = self.world.entities.pop()
                if self.world.use_pool and self.world.pool:
                    self.world.pool.release(ent)
            print("Removed -100, total:", len(self.world.entities))
        elif "Clear" in label:
            self._clear_and_respawn(100)
            print("Cleared & respawned 100")
        elif "Grav +" in label:
            self.world.gravity += 0.1
            print("Gravity:", self.world.gravity)
        elif "Grav -" in label:
            self.world.gravity -= 0.1
            print("Gravity:", self.world.gravity)

    def _clear_and_respawn(self, n):
        self.world.entities.clear()
        if self.world.prev_positions: self.world.prev_positions.clear()
        for _ in range(n):
            r = random.randint(MIN_RADIUS, MAX_RADIUS)
            x = random.uniform(16 + r, self.width - 16 - r)
            y = random.uniform(16 + r, self.height - 16 - r)
            self.world.spawn_ball(x,y,r)

    def _record_frame(self):
        fname = os.path.join(FRAME_DIR, f"frame_{self.frame_count:06d}.png")
        pygame.image.save(self.screen, fname)
        self.frame_count += 1




# ---------------------------
# ======== ENTRYPOINT =======
# ---------------------------
def main():
    print("Ultimate Simulation Engine (SAT polygons enabled)")
    print("Hotkeys: G grid, H hud, V integrator toggle, P pause, R record, +/- spawn, C clear, Esc quit")
    eng = Engine()
    eng.run()

if __name__ == "__main__":
    main()
