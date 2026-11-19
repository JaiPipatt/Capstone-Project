import math
from collections import deque

FACES = ('L', 'R', 'T', 'B')
FACE_ANGLE = {'T': 0.0, 'L': math.pi / 2, 'B': math.pi, 'R': -math.pi / 2}


class Topology:
    # ex. connections = [(1, 'B', 0, 'T'), (2, 'B', 0, 'L')]
    def __init__(self, n, connections):
        self.n = n
        self.adj = {v: {} for v in range(n)}  # v -> {face: (neighbour, neighbour_face)}
        for u, cu, v, cv in connections:
            for a, ca, b, cb in ((u, cu, v, cv), (v, cv, u, cu)):
                if ca not in FACES:
                    raise ValueError(f'unknown face {ca}')
                if ca in self.adj[a]:
                    raise ValueError(f'face {ca} of module {a} is used twice')
                self.adj[a][ca] = (b, cb)
        self.num_edges = len(connections)


def bfs_from_root(topo, root):
    # parent[v] = (p, cv, cp): v's face cv is docked to face cp of its parent p
    order, parent, depth = [root], {}, {root: 0}
    queue = deque([root])
    while queue:
        v = queue.popleft()
        for c, (u, cu) in topo.adj[v].items():
            if u not in depth:
                depth[u] = depth[v] + 1
                parent[u] = (v, cu, c)
                order.append(u)
                queue.append(u)
    return order, parent, depth


def find_root(topo, v0=0):
    n = topo.n
    order, parent, _ = bfs_from_root(topo, v0)

    height = {v: 1 for v in order}
    for v in reversed(order):
        if v in parent:
            p = parent[v][0]
            height[p] = max(height[p], height[v] + 1)

    cn = {v: {c: 0 for c in FACES} for v in range(n)} 
    for h in range(1, height[v0] + 1):
        for v in (v for v in order if height[v] == h):
            for c, (child, cc) in topo.adj[v].items():
                if parent.get(child, (None,))[0] != v:
                    continue
                below = sum(cn[child][f] for f in FACES if f != cc)
                cn[v][c] = 1 + below  
                cn[child][cc] = n - 1 - below

    roots = [v for v in range(n) if all(cn[v][c] <= n / 2 for c in FACES)]
    return roots[0], cn


def unfold(topo, root, w):
    # Slot pose p_bar[v] = [x, y, theta] of every module in the root's frame
    order, parent, _ = bfs_from_root(topo, root)
    poses = {root: (0.0, 0.0, 0.0)}
    for v in order[1:]:
        p, cv, cp = parent[v]
        a = FACE_ANGLE[cp]
        p_ji = (w * math.cos(a), w * math.sin(a), wrap_angle(a + math.pi - FACE_ANGLE[cv]))
        poses[v] = coordinates_from_frame(poses[p], p_ji)
    return poses


def check_topology(topo, w):
    order, _, _ = bfs_from_root(topo, 0)
    if len(order) != topo.n:
        raise ValueError('topology is not connected')
    if topo.num_edges != topo.n - 1:
        raise ValueError('topology has a loop; convert it to a spanning tree first')
    centres = {(round(x / w), round(y / w)) for x, y, _ in unfold(topo, 0, w).values()}
    if len(centres) != topo.n:
        raise ValueError('two modules would occupy the same location when unfolded')

# --------------------------- HELPERS --------------------------- #
def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def coordinates_from_frame(p_i, p_ji):
    # pose p_ji given in frame p_i, expressed in p_i's parent frame
    x, y, th = p_i
    c, s = math.cos(th), math.sin(th)
    return (x + c * p_ji[0] - s * p_ji[1],
            y + s * p_ji[0] + c * p_ji[1],
            wrap_angle(th + p_ji[2]))

def coordinates_to_frame(p_ref, p):
    # pose p expressed in the frame of p_ref (shift, then rotate by -theta_ref)
    dx, dy = p[0] - p_ref[0], p[1] - p_ref[1]
    c, s = math.cos(-p_ref[2]), math.sin(-p_ref[2])
    return (c * dx - s * dy, s * dx + c * dy, wrap_angle(p[2] - p_ref[2]))