# ros2_introspection

Spiritual successor of [ros_type_introspection](https://github.com/facontidavide/ros_type_introspection) for ROS2.

It allows the use to convert a ROS2 message into a vector of (string/value) pairs, even if its type is not known at compilation-time.

It leverage the nice work done by other people, in particular [Fast-CDR](https://github.com/eProsima/Fast-CDR) and [rosbag2](https://github.com/ros2/rosbag2).

Typical usage:

```C++

// you want to deserialize this:
// rmw_serialized_message_t* serialized_message; 

Ros2Introspection::Parser parser;
// register **once** your type
parser.registerMessageType("imu", "sensor_msgs/Imu");

Ros2Introspection::FlatMessage flat_msg;
unsigned max_array_size = 100;
parser.deserializeIntoFlatMessage("imu", serialized_msg, &flat_msg, max_array_size);  
  
// you can show the entire content of the message as a list of key-values
for(const auto& pair: flat_msg.values)
{
  std::cout << pair.first.toStdString() << " = " << pair.second << std::endl;
}
for(const auto& pair: flat_msg.strings)
{
  std::cout << pair.first.toStdString() << " = " << pair.second << std::endl;
}
```
