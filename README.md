# Necrodrive - ros2_control hardware interface for NRND ODrives!
<div>
    <img alt="Static Badge" src="https://img.shields.io/badge/Jazzy-Version?style=for-the-badge&logo=ros&logoColor=%2322314E&logoSize=wide&label=ROS&color=blue">
    <img alt="Static Badge" src="https://img.shields.io/badge/24.04-Ubuntu?style=for-the-badge&logo=Ubuntu&logoColor=%23E95420&label=Ubuntu&color=%23E95420">
    <img alt="Static Badge" src="https://img.shields.io/badge/17-C%2B%2B?style=for-the-badge&logo=C%2B%2B&logoColor=%2300599C&label=C%2B%2B&labelColor=gainsboro">
</div>

Because the official ODrive ros2_control package doesn't respect their elders.

Table of contents:
- [Installation](#installation)
- [Testing](#testing)
- [Usage](#usage)
- [Setting up SocketCAN](#setting-up-socketcan)
- [Dependencies](#dependencies)

## Installation
First, I'm assuming you've already created a ros2 workspace. If you haven't, run
```bash
mkdir necrodrive_ws/src
cd necrodrive_ws/src
```
then, clone this directory into your src or add it as a submodule
```bash
git clone https://github.com/RobohanOU/necrodrive_ros2.git
```
OR
```bash
git submodule add https://github.com/RobohanOU/necrodrive_ros2.git
```
inside of **src**. Then, cd work into your workspace directory and build
```sh
colcon build --symlink-install
```
Then, you can move onto testing.
## Testing
First, here's the assumption Necrodrive makes about your ODrive.
- The ODrive is precalibrated, and can be immediately switched into closed_loop_control upon power on. Note that this generally means the ODrive is outfitted with an absolute encoder, such as a magnetic encoder. Future changes may remove this requirement but for now - that's how it is.

If the ODrive meets this requirement, you may move on to configuring Necrodrive. First, open `bringup_test/sdf/robot.sdf.xacro`. Scroll down and you will find the parameters for the hardware interface
```xml
<hardware>
    <plugin>necrodrive_ros2/NecrodriveSystem</plugin>
    <param name="can_interface">can0</param>
    <param name="odrive_id">0</param>
    <param name="write_timeout_ns">20</param>
    <param name="read_timeout_ns">10</param>
    <param name="heartbeat_timeout_ms">150</param>
</hardware>
``` 
The parameters are self-explanatory. Just make sure your SocketCAN interface is set up. If you'd like to know how, see [Setting up SocketCAN](#setting-up-socketcan).

Once that's set up. you should be ready to test Necrodrive. Note that Necrodrive will immediately fail if the specified can interface is not available/doesn't exist (I should probably fix that soon...)

```bash
ros2 launch necrodrive_ros2 necrodrive.launch.py
```
If everything goes well, you'll see something like
```
Configuration complete! Bus voltage: %f
```
and read/write messages. That means necrodrive is running!

The launchfile defaults to a [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html). This can be changed to a [forward_position_controller](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html) by setting the parameter `use_jtc` to false,

To send commands to the trajectory controller, open a new terminal and run
```bash
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory \
trajectory_msgs/JointTrajectory "{
  joint_names: ['bldc_joint'],
  points: [
    { positions: [0.5], time_from_start: {sec: 2} }
  ]
}"
```
and you should see the ODrive move!
## Usage
If you're not familiar with ros2_control, here's an overview of how things work. In ros2_control, hardware (motors, sensors, etc) are accessed through 'hardware interfaces'. The 'resource manager' loads and manages these hardware interfaces, which are used by 'controllers'. `Necrodrive` is a hardware interface, which mean it simply provides access to a piece of hardware (in this case the ODrive), so **you could use it with any controller of your choice**.

This comes down launching a:
- controller_manager ros2_control_node node which manages the controller
- robot_state_publisher robot_state_publisher node which publishes the robot description **containing the <ros2_control> tags**
- controller_manager spawner node which spawns the controller

probably in that order to get things working. For reference, see `necrodrive.launch.py`. Necrodrive only does the job of *bridging ros2_control with the ODrive*. How that is controlled is **determined by the controller**.
## Setting up SocketCAN
The can interface isn't usually set up by default, so you should probably put this into a script in deployment.

How SocketCAN is set up will depend on what firmware your CAN adapter: slcan or candleLight. 
To figure this out, run
```bash
ip link show
```
if you see something like `can0`, you're probably running candleLight. Otherwise, try checking dmesg for usb messages
```bash
sudo dmesg | grep -i tty
```
if you see something like `ttyACM0` or `ttyUSB0`, then you're probably using slcan.
### CandleLight
Set the baudrate of the can interface
```bash
sudo ip link set can0 type can bitrate 500000
```
Then bring the interface online
```bash
sudo ip link set can0 up
```

### slcan
Make sure the slcan module is running
```bash
sudo modprobe slcan
```
Then, set the baudrate
```
sudo slcand -o -c -s6 /dev/ttyACM0 can0
```
the `-s6` flag is the speed flag. Here's a list of each flag with their corresponding baudrates

| flag  | baudrate |
| :---: | :------: |
| `-s0` | 10 kpbs  |
| `-s1` | 20 kbps  |
| `-s2` | 50 kbps  |
| `-s3`	| 100 kbps |
| `-s4` | 125 kbps |
| `-s5`	| 250 kbps |
| `-s6` | 500 kbps |
| `-s8` | 1 Mbps   |

and now you should see can0 in your sockets. Finally, bring it online
```bash
sudo ip link set can0 up
```

## Dependencies
This package mainly depends on ros2_control. If your ros2 version is Jazzy and you have ros2_control installed, you should be good to go. Just in case, here are the commands you can run to install ros2_control 
```bash
sudo apt install \
    ros-rolling-ros2-control \
    ros-rolling-ros2-controllers
```
and for good practice, you can also run `rosdep` (although you probably don't need to)
```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```
