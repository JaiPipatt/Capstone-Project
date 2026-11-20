# msrr_control

Central controller that self-assembles N identical modules (differential drive,
square body, EP faces) into a target shape. Works in Gazebo simulation; poses
come from Gazebo ground truth (AprilTag localization later).

## Build

```bash
cd ros2_ws
colcon build --packages-select msrr_control my_robot_description
source install/setup.bash
```

Dependencies: ROS 2 (Humble/Fortress setup used here), `numpy`, `scipy`,
`ros_gz_sim`, `ros_gz_bridge`, `teleop_twist_keyboard` (teleop only), `xterm` (teleop only).

## Run the assembly in simulation

Two terminals, both with `source install/setup.bash`.

```bash
# Terminal 1: Gazebo with 3 modules
ros2 launch my_robot_description sim.launch.py num_modules:=3

# Terminal 2: the controller (wait until all modules and controllers are up)
ros2 launch msrr_control algo.launch.py
```

The controller waits until every module has reported a pose, then plans and
runs the assembly. Log line `assembly finished` means done.

### Launch arguments

| Launch file | Argument | Default | Meaning |
|---|---|---|---|
| `sim.launch.py` (my_robot_description) | `num_modules` | `3` | modules spawned, in a row along y |
| | `spacing` | `0.3` | distance between modules [m] |
| | `gui` | `true` | `false` = headless Gazebo |
| `algo.launch.py` | `use_sim_time` | `true` | use Gazebo `/clock` |
| `teleop.launch.py` | `module` | `module_1` | module to drive by keyboard |
| | `sim` | `true` | `false` = do not start Gazebo (real robot) |
| | `gui`, `num_modules`, `spacing` | | passed to `sim.launch.py` |

Example, headless sim with 3 modules spaced 0.4 m:

```bash
ros2 launch my_robot_description sim.launch.py num_modules:=3 spacing:=0.4 gui:=false
```

### Drive one module by hand

```bash
ros2 launch msrr_control teleop.launch.py module:=module_2 num_modules:=3
```

Opens an xterm window; keep it focused while pressing keys. Do not run it
together with `algo.launch.py` (both publish to the same `cmd_vel` topic).

### Print the plan without a simulator

```bash
ros2 run msrr_control assembly_planner
```

Prints root slot, slot poses, module-to-slot mapping, rounds and grid routes
for made-up start poses.

## Parameters (`config/algo.yaml`, node `algo_node`)

Edit the file and rebuild, or override on the command line:

```bash
ros2 run msrr_control algo_node --ros-args --params-file src/msrr_control/config/algo.yaml \
  -p num_modules:=3 -p "connections:=['1,B,0,T','2,B,0,L']" -p use_sim_time:=true
```

| Parameter | Default | Meaning |
|---|---|---|
| `num_modules` | `3` | modules `module_1..module_N` |
| `pose_topic` | `ground_truth/odom` | read as `/module_i/<pose_topic>` (`nav_msgs/Odometry`) |
| `rate` | `10.0` | control rate [Hz] |
| `max_linear` | `0.049` | top speed [m/s] |
| `max_angular` | `1.0` | top turn rate [rad/s] |
| `goal_tolerance` | `0.01` | pre-dock / dock position tolerance [m] |
| `waypoint_tolerance` | `0.02` | intermediate grid node tolerance [m] |
| `heading_tolerance` | `0.05` | heading tolerance [rad] |
| `module_width` | `0.107` | centre-to-centre distance when docked [m] |
| `grid` | `0.107` | navigation cell size [m]; keep equal to `module_width` |
| `connections` | `['1,B,0,T', '2,B,0,L']` | target shape, see below |

### Describing the target shape

Each entry is `'u,cu,v,cv'`: slot `u`, face `cu` docks to slot `v`, face `cv`.
Numbers are slots of the target shape (slot 0 is the root), not robots; the
planner decides which physical module fills which slot. Faces:

| Face | Side | Angle from heading |
|---|---|---|
| `T` | top / front | 0 |
| `L` | left (wheel) | +90 deg |
| `B` | bottom / back | 180 deg |
| `R` | right (wheel) | -90 deg |

The shape must be a connected tree (no loops), with `num_modules - 1` connections.

Example, L shape (default), 3 modules:

```
  [2]
   |
  [0]-[1]
```
```yaml
num_modules: 3
connections: ['1,B,0,T', '2,B,0,L']
```

Example, 4-module line, back of each to front of the previous:

```yaml
num_modules: 4
connections: ['1,B,0,T', '2,B,1,T', '3,B,2,T']
```
(Start with `num_modules:=4` for `sim.launch.py` as well.)

### Side dock (wheel-face) demo

Modules spawn in a row along y, all facing +x, so they already sit side by
side. A side dock joins a wheel face (`L`/`R`) of the moving module to the
opposite wheel face of its parent. The moving module drives the plan
pivot -> straight -> pivot -> straight along the parent's side.

Two modules, slot 1's left face to slot 0's right face:

```
  [0] [1]      (from above, modules face up; slot 1 ends on the right of slot 0)
```
```bash
# Terminal 1
ros2 launch my_robot_description sim.launch.py num_modules:=2
# Terminal 2 (run the node directly to set the shape from the command line)
ros2 run msrr_control algo_node --ros-args --params-file src/msrr_control/config/algo.yaml \n  -p num_modules:=2 -p "connections:=['1,L,0,R']" -p use_sim_time:=true
```

Three modules, one docked on each side of the middle one:

```yaml
num_modules: 3
connections: ['1,L,0,R', '2,R,0,L']
```
```bash
ros2 launch my_robot_description sim.launch.py num_modules:=3 spacing:=0.3
# then algo.launch.py with the yaml above, or algo_node with -p as before
```

Watch the log for `module_N docked` (or `docked but wheel angle off by ...` if
the final wheel angle is outside tolerance). `module_N wheel dock failed: no
clear docking plan` means the module started too close to the parent to pivot
safely; start it farther away.

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/module_i/ground_truth/odom` | `nav_msgs/Odometry` |
| in | `/module_i/joint_states` | `sensor_msgs/JointState` |
| out | `/module_i/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` |

## How it works

1. Plan: pick root slot, assign modules to slots, split docking into rounds, plan grid routes (`assembly.py`, `navigation.py`, `topology.py`).
2. Each moving module in a round: `navigate` to the pre-dock point, then
   - end docking (`T`/`B`): `adjust` heading, `approach` straight in;
   - wheel-face docking (`L`/`R`): `wheel_dock` = pivot, straight, pivot, straight (`wheel_dock_plan.py` plans, `wheel_dock.py` runs it).
3. Moving modules do not avoid each other yet.

## Tests

```bash
cd ros2_ws/src/msrr_control
python -m pytest test/test_wheel_dock.py test/test_assembly_planner.py test/test_navigation.py
```
