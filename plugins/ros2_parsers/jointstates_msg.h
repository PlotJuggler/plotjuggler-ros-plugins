#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_parser.h"

class JointStateMsgParser : public BuiltinMessageParser<sensor_msgs::msg::JointState>
{
public:
  JointStateMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<sensor_msgs::msg::JointState>(topic_name, plot_data)
  {
    _data.push_back(&getSeries(plot_data, topic_name + "/header/stamp/sec"));
    _data.push_back(&getSeries(plot_data, topic_name + "/header/stamp/nanosec"));
  }

  void parseMessageImpl(const sensor_msgs::msg::JointState& msg, double timestamp) override
  {
    if (_use_header_stamp)
    {
      timestamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    }

    _data[0]->pushBack({ timestamp, double(msg.header.stamp.sec) });
    _data[1]->pushBack({ timestamp, double(msg.header.stamp.nanosec) });

    for (int i = 0; i < msg.name.size(); i++)
    {
      const std::string prefix = _topic_name + "/" + msg.name[i];

      if (msg.name.size() == msg.position.size())
      {
        auto& series = getSeries(_plot_data, prefix + "/position");
        series.pushBack({ timestamp, msg.position[i] });
      }

      if (msg.name.size() == msg.velocity.size())
      {
        auto& series = getSeries(_plot_data, prefix + "/velocity");
        series.pushBack({ timestamp, msg.velocity[i] });
      }

      if (msg.name.size() == msg.effort.size())
      {
        auto& series = getSeries(_plot_data, prefix + "/effort");
        series.pushBack({ timestamp, msg.effort[i] });
      }
    }
  }

private:
  std::vector<PlotData*> _data;
};
