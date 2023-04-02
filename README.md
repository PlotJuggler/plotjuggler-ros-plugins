# ROS plugins for PlotJuggler

PlotJuggler works great with ROS, but it is not itself a "ROS" application.

ROS is supported through external plugins that can be found in this [repository](https://github.com/PlotJuggler/plotjuggler-ros-plugins/).

## Existing Plugins

- DataLoader for **rosbags** (ROS / ROS2).
- ROS **topic subscriber** (ROS / ROS2).
- **Logs/rosout** visualizer (ROS only).
- **Re-publisher** similar to `rosbag play` (ROS only).

## Skip the topic selection window (useful when loading a layout)
When loading a layout that includes ROS topics, PlotJuggler shows the ROS topic selection dialog, in order to skip
clicking on `Ok` you can set the environment variable: `PLOTJUGGLER_ACCEPT_SELECT_ROS_TOPIC_DIALOG=1` and this dialog will be skipped.

Note that if you click on `Stop` and then on `Start` again, you will be able to edit the topics.


## Install with Debians (TODO)

Install PlotJuggler and its ROS plugins with:

    sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
    
To launch PlotJuggler on ROS, use the command:

      rosrun plotjuggler plotjuggler

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
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    catkin build
    
Enjoy:

    source devel/setup.bash
    roslaunch plotjuggler_ros plotjuggler.launch
    
     
