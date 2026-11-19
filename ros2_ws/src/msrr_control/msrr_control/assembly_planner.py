from msrr_control.assembly import assign_modules_to_slots, docking_tasks
from msrr_control.navigation import plan_routes
from msrr_control.topology import check_topology, FACES, find_root, Topology, unfold


def main():
    w = 0.107     # module width [m]
    grid = 0.10  # navigation grid cell [m]
    shape = Topology(3, [(1, 'B', 0, 'T'), (2, 'B', 0, 'L')])
    modules = {  # made-up start poses, world frame
        'R0': (0.30, 0.25, 1.20),
        'R1': (0.05, 0.02, 0.30),
        'R2': (-0.25, 0.20, -2.00),
    }

    check_topology(shape, w)
    tau, cn = find_root(shape)
    print(f'root slot tau = {tau}')
    for v in range(shape.n):
        print(f'  CN^{v} =', {c: cn[v][c] for c in FACES if c in shape.adj[v]})

    slots = unfold(shape, tau, w)
    print('slot poses p_bar (root frame):')
    for s, p in slots.items():
        print(f'  slot {s}: x={p[0]:+.3f} y={p[1]:+.3f} th={p[2]:+.3f}')

    f, m_tau = assign_modules_to_slots(slots, tau, modules)
    print(f'physical root m_tau = {m_tau}')
    print('mapping f:', ', '.join(f'{s}->{m}' for s, m in sorted(f.items())))

    rounds = docking_tasks(shape, tau, f, slots, modules, m_tau, grid)
    _, routes = plan_routes(rounds, f, tau, slots, modules, m_tau, grid)
    for i, tasks in enumerate(rounds, 1):
        print(f'round {i}:')
        for t in tasks:
            print(f'  {t.action}  pre_dock=({t.pre_dock[0]:+.3f}, {t.pre_dock[1]:+.3f})'
                  f'  goal=({t.goal[0]:+.3f}, {t.goal[1]:+.3f}, {t.goal[2]:+.3f})')
            print('    route:', ' -> '.join(f'({x:+.2f}, {y:+.2f})'
                                          for x, y in routes[t.action[0]]))


if __name__ == '__main__':
    main()