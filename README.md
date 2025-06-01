[![ros2](https://github.com/PlotJuggler/plotjuggler-ros-plugins/actions/workflows/ros2.yaml/badge.svg)](https://github.com/PlotJuggler/plotjuggler-ros-plugins/actions/workflows/ros2.yaml)

# ROS plugins for PlotJuggler

PlotJuggler works great with ROS, but it is not itself a "ROS" application.

ROS is supported through external plugins that can be found in this [repository](https://github.com/PlotJuggler/plotjuggler-ros-plugins/).

## Existing Plugins

- DataLoader for **rosbags** (ROS / ROS2).
- ROS **topic subscriber** (ROS / ROS2).
- **Logs/rosout** visualizer (ROS only).
- **Re-publisher** similar to `rosbag play` (ROS only).


## Install with Debians

Install PlotJuggler and its ROS plugins with:

    sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros

or, if have ROS2 installed:

    ros2 run plotjuggler plotjuggler

## How to compile PlotJuggler from source

Create a catkin workspace and clone the repositories:

     mkdir -p ~/ws_plotjuggler/src
     cd ~/ws_plotjuggler/src
     git clone https://github.com/PlotJuggler/plotjuggler_msgs.git
     git clone https://github.com/facontidavide/PlotJuggler.git
     git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git
     
Now, it is time to compile:

    cd ~/ws_plotjuggler
    rosdep install --from-paths src --ignore-src -y
    colcon build

# For ROS (1) users

From version 2.3+, this library will support only ROS2 (Hubmle or older)

    
     
