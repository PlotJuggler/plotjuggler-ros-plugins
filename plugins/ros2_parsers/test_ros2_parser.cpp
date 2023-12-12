
#include "ros2_parser.h"

int main()
{
    std::cout << CreateSchema("nav_msgs/Odometry") << std::endl; 
    std::cout << "\n\n" << CreateSchema("sensor_msgs/JointState") << std::endl;
    std::cout << "\n\n" << CreateSchema("tf2_msgs/TFMessage") << std::endl; 
}