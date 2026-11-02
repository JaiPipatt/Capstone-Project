# communcation

PC <-> module ESP32 link. The ESP32s run micro-ROS and talk to ROS 2 through
the micro-ROS agent, so there is no bridge node: `algo_node` and
`teleop.launch.py sim:=false` drive the real robot on the same topics as the
simulation.

## Hotspot (PC)

The PC runs a hotspot for the ESP32s while staying on ChulaWiFi for internet.
The WiFi card (MT7921) allows one AP and one client at once, but both must share
a channel, and the ESP32 only does 2.4 GHz.

| | |
|---|---|
| SSID / password | `pop-os` / `12345678` |
| PC IP (agent IP) | `10.42.0.1` (`ap0`) |
| NM connection | `Hotspot-1` |

One-time setup (already done on this laptop):

```bash
sudo apt install iw
nmcli con modify "ChulaWiFi" 802-11-wireless.band bg   # keep internet on 2.4 GHz
nmcli con up "ChulaWiFi"                                # may take 10-20 s
sudo iw dev wlo1 interface add ap0 type __ap
nmcli dev wifi hotspot ifname ap0 ssid pop-os password 12345678 band bg
```

Start the hotspot (every session):

```bash
sudo iw dev wlo1 interface add ap0 type __ap   # only after a reboot; ap0 is not persistent
sleep 2
nmcli con up Hotspot-1
```

- Stop: `nmcli con down Hotspot-1`
- Check: `ip -br addr` shows `ap0 UP 10.42.0.1/24`; `ip neigh show dev ap0` lists connected ESP32s.
- If ChulaWiFi drops or changes channel the hotspot drops too; run `nmcli con up Hotspot-1` again.
- Undo the 2.4 GHz lock: `nmcli con modify "ChulaWiFi" 802-11-wireless.band ""`

## Run the agent (PC)

Start it after the hotspot, before the robot nodes, and keep it running. One agent serves every module.

```bash
# docker (needs your user in the docker group: sudo usermod -aG docker $USER, then log out/in)
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888

# or snap (not available on Pop!_OS by default)
sudo snap install micro-ros-agent
micro-ros-agent udp4 --port 8888
```

- ESP32s join the `pop-os` hotspot and send to `10.42.0.1:8888`; allow incoming UDP 8888 in the firewall.
- Check: agent log shows `session established`, `ros2 node list` shows `/module_1/esp32_module`.

## ESP32 side (micro-ROS, Humble), per module i

| | |
|---|---|
| transport | WiFi UDP to `10.42.0.1:8888` (hotspot `pop-os`) |
| node | `esp32_module` in namespace `module_i` |
| subscribe | `diff_drive_controller/cmd_vel_unstamped`, `geometry_msgs/Twist` |
| units | `linear.x` m/s, `angular.z` rad/s; wheel maths on the ESP32 |
| safety | stop if no command for 0.5 s or the agent is lost |
