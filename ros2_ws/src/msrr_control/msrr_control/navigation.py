"""
Grid navigation (SMORES-EP, Liu et al., section 5.2-5.3.1).

A square grid is centred on the physical root module and aligned with its
heading. Modules drive along grid edges to their pre-dock point.
Moving modules do not avoid each other; only slots that are already
assembled are blocked.
"""
import heapq
import math

from msrr_control.topology import coordinates_from_frame, coordinates_to_frame


class Grid:
    # nodes are integer (i, j); node (0, 0) is the root, +i along the root's heading
    def __init__(self, origin, cell, half_size):
        self.origin = origin        # world pose (x, y, theta) of the root module
        self.cell = cell            # cell size [m]
        self.half_size = half_size  # nodes run from -half_size to +half_size

    def to_node(self, xy):
        x, y, _ = coordinates_to_frame(self.origin, (xy[0], xy[1], 0.0))
        return (round(x / self.cell), round(y / self.cell))

    def to_world(self, node):
        local = (node[0] * self.cell, node[1] * self.cell, 0.0)
        x, y, _ = coordinates_from_frame(self.origin, local)
        return (x, y)

    def inside(self, node):
        return abs(node[0]) <= self.half_size and abs(node[1]) <= self.half_size


def make_grid(origin, cell, points, margin=2):
    # smallest square grid around the root that covers every point, plus a margin
    probe = Grid(origin, cell, 0)
    half = max(max(abs(i), abs(j)) for i, j in (probe.to_node(p) for p in points))
    return Grid(origin, cell, half + margin)


def astar(grid, start, goal, blocked=()):
    # 4-connected A* over grid nodes; returns the node path, start and goal included
    blocked = set(blocked) - {start, goal}
    came_from = {start: None}
    cost = {start: 0}
    queue = [(0, start)]
    while queue:
        _, node = heapq.heappop(queue)
        if node == goal:
            path = []
            while node is not None:
                path.append(node)
                node = came_from[node]
            return path[::-1]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (node[0] + di, node[1] + dj)
            if not grid.inside(nxt) or nxt in blocked:
                continue
            new_cost = cost[node] + 1
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt] = new_cost
                came_from[nxt] = node
                h = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                heapq.heappush(queue, (new_cost + h, nxt))
    raise ValueError(f'no grid path from {start} to {goal}')


def plan_routes(task_rounds, f, root, slots, modules, m_tau, cell):
    """World waypoints for every moving module: start -> grid nodes -> pre_dock.

    task_rounds: output of docking_tasks()
    Slots assembled in earlier rounds (and the root) are blocked for later rounds.
    """
    slot_of = {m: s for s, m in f.items()}
    grid = make_grid(modules[m_tau], cell,
                     [p[:2] for p in modules.values()]
                     + [t.pre_dock[:2] for tasks in task_rounds for t in tasks])
    slot_node = {s: (round(p[0] / cell), round(p[1] / cell)) for s, p in slots.items()}

    filled = {root}
    routes = {}
    for tasks in task_rounds:
        blocked = {slot_node[s] for s in filled}
        for t in tasks:
            m = t.action[0]
            start = modules[m][:2]
            nodes = astar(grid, grid.to_node(start), grid.to_node(t.pre_dock), blocked)
            routes[m] = [start] + [grid.to_world(n) for n in nodes] + [t.pre_dock[:2]]
        filled |= {slot_of[t.action[0]] for t in tasks}
    return grid, routes