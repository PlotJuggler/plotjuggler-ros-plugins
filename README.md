# ROS plugins for PlotJuggler

PlotJuggler works great with ROS, but it is not itself a "ROS" application.

ROS is supported through external plugins that can be found in this [repository](https://github.com/PlotJuggler/plotjuggler-ros-plugins/).

## Existing Plugins

- DataLoader for **rosbags** (ROS / ROS2).
- ROS **topic subscriber** (ROS / ROS2).
- **Logs/rosout** visualizer (ROS only).
- **Re-publisher** similar to `rosbag play` (ROS only).


## Install with Debinas

Install PlotJuggler and its ROS plugins with:

    sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
    
And launch it with:
    
    rosrun plotjuggler PlotJuggler

## How to compile PlotJuggler from source

Create a catkin workspace and clone the repositories:

     mkdir -p ~/ws_plotjuggler/src
     cd https://github.com/PlotJuggler/PlotJuggler3
     git clone https://github.com/PlotJuggler/plotjuggler_msgs.git
     git clone https://github.com/PlotJuggler/PlotJuggler3.git
     git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git
     
Now, it is time to compile:

    cd ~/ws_plotjuggler
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    catkin build
    
Enjoy:

    source devel/setup.bash
    rosrun plotjuggler PlotJuggler
    
     
