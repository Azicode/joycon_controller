# joycon_controller

## Introduction
Using Nintendo Switch Joy Controller on ROS2.
In this package, you can manipulate turtlesim.

## Dependencies

- ROS2 Foxy
- [pygame](https://www.pygame.org/docs/)
- [turtlesim](https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html)

## Installation of turtlesim

```shell
# Install the turtlesim package.
$ sudo apt update
$ sudo apt install ros-foxy-turtlesim

# Start turtlesim
$ ros2 run turtlesim turtlesim_node
```

## Installation of this package

```shell
$ . /opt/ros/foxy/setup.bash
$ cd
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src

$ git clone https://github.com/Azicode/joycon_controller.git
$ pip3 install pygame

$ cd ~/ros2_ws
$ rosdep install -i --from-path src --rosdistro foxy -y
$ colcon build --symlink-install
```

## Demo joycon_controller
Open a new terminal.
```shell
$ cd ros2_ws
$ . /opt/ros/foxy/setup.bash
$ . install/setup.bash
```
Next command will fail, if JoyCon is not connected to PC.
Please connnect by Bluetooth.
```shell
$ ros2 run joycon_controller joycon_controller
```

## Change Orientation
Look at line 16 in [joycon_controller.py](joycon_controller/joycon_controller.py) and edit the value of ROTATON.
```joycon_controller.py
11  #-------- Rotation --------#
12  # JoyCon R, Vertical   => 0
13  # JoyCon L, Vertical   => 1
14  # JoyCon R, Horizontal => 2
15  # JoyCon L, Horizontal => 3
16  ROTATION = 0
```

## About writer
- Azi : Japanese student majoring in marine system engineering.
- Twitter : https://twitter.com/Azi_mark01
- Qiita (Japanese) : https://qiita.com/Azi
