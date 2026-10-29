# communcation

PC <-> module ESP32 link. The ESP32s run micro-ROS and talk to ROS 2 through
the micro-ROS agent, so there is no bridge node: `algo_node` and
`teleop.launch.py sim:=false` drive the real robot on the same topics as the
simulation.

## Run the agent (PC)

Start it before the robot nodes and keep it running. One agent serves every module.

```bash
# snap
sudo snap install micro-ros-agent
micro-ros-agent udp4 --port 8888

# or docker
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

- PC and ESP32s on the same WiFi; allow incoming UDP 8888 in the firewall.
- Check: agent log shows `session established`, `ros2 node list` shows `/module_1/esp32_module`.

## ESP32 side (micro-ROS, Humble), per module i

| | |
|---|---|
| transport | WiFi UDP to `<PC IP>:8888` |
| node | `esp32_module` in namespace `module_i` |
| subscribe | `diff_drive_controller/cmd_vel_unstamped`, `geometry_msgs/Twist` |
| units | `linear.x` m/s, `angular.z` rad/s; wheel maths on the ESP32 |
| safety | stop if no command for 0.5 s or the agent is lost |
