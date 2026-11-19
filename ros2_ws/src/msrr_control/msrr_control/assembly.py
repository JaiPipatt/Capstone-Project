import math
from dataclasses import dataclass

from scipy.optimize import linear_sum_assignment

from msrr_control.topology import (
    bfs_from_root, coordinates_from_frame, coordinates_to_frame, wrap_angle)


def assign_modules_to_slots(slots, root, modules):
    if len(modules) != len(slots):
        raise ValueError(f'need {len(slots)} modules, got {len(modules)}')

    xc = sum(p[0] for p in modules.values()) / len(modules)
    yc = sum(p[1] for p in modules.values()) / len(modules)
    m_tau = min(modules, key=lambda m: math.hypot(modules[m][0] - xc, modules[m][1] - yc))

    local = {m: coordinates_to_frame(modules[m_tau], p) for m, p in modules.items()}
    free_modules = [m for m in modules if m != m_tau]
    free_slots = [s for s in slots if s != root]
    cost = [[math.hypot(local[m][0] - slots[s][0], local[m][1] - slots[s][1])
             for s in free_slots] for m in free_modules]
    rows, cols = linear_sum_assignment(cost)  # Kuhn-Munkres

    f = {root: m_tau}
    for r, c in zip(rows, cols):
        f[free_slots[c]] = free_modules[r]
    return f, m_tau


def parallel_assembly(topo, root, f):
    _, parent, depth = bfs_from_root(topo, root)
    rounds = []
    for d in range(1, max(depth.values()) + 1):
        group = [(f[v], cv, f[p], cp) for v, (p, cv, cp) in sorted(parent.items())
                 if depth[v] == d]
        if d == 1:
            rounds.append([a for a in group if a[3] in ('L', 'R')])
            rounds.append([a for a in group if a[3] in ('T', 'B')])
        else:
            rounds.append(group)
    return [r for r in rounds if r]


@dataclass
class DockingTask:
    action: tuple   # (m, c, m_parent, c_parent)
    pre_dock: tuple  # 1. navigate here (world x, y, theta), one grid cell out
    goal: tuple      # 2. adjust heading to goal theta, 3. approach straight to goal


def docking_tasks(topo, root, f, slots, modules, m_tau, grid):
    _, parent, _ = bfs_from_root(topo, root)
    slot_of = {m: s for s, m in f.items()}
    world = {s: coordinates_from_frame(modules[m_tau], p) for s, p in slots.items()}
    rounds = []
    for group in parallel_assembly(topo, root, f):
        tasks = []
        for action in group:
            s = slot_of[action[0]]
            gx, gy, gth = world[s]
            px, py, _ = world[parent[s][0]]
            d = math.hypot(gx - px, gy - py)
            pre = (gx + grid * (gx - px) / d, gy + grid * (gy - py) / d, gth)
            tasks.append(DockingTask(action, pre, world[s]))
        rounds.append(tasks)
    return rounds


def unicycle_step(pose, v, omega, dt):
    x, y, th = pose
    return (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, wrap_angle(th + omega * dt))