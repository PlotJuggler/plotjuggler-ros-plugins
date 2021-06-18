#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_parser.h"

class JointStateMsgParser : public BuiltinMessageParser<sensor_msgs::msg::JointState>
{
public:
  JointStateMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<sensor_msgs::msg::JointState>(topic_name, plot_data)
  {
    _data.push_back(&getSeries(topic_name + "/header/stamp"));
  }

  void parseMessageImpl(const sensor_msgs::msg::JointState& msg, double& timestamp) override
  {
    double header_stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    if (_use_header_stamp)
    {
      timestamp = header_stamp;
    }

    _data[0]->pushBack({ timestamp, header_stamp });

    for (int i = 0; i < msg.name.size(); i++)
    {
      const std::string prefix = _topic_name + "/" + msg.name[i];

      if (msg.name.size() == msg.position.size())
      {
        auto& series = getSeries(prefix + "/position");
        series.pushBack({ timestamp, msg.position[i] });
      }

      if (msg.name.size() == msg.velocity.size())
      {
        auto& series = getSeries(prefix + "/velocity");
        series.pushBack({ timestamp, msg.velocity[i] });
      }

      if (msg.name.size() == msg.effort.size())
      {
        auto& series = getSeries(prefix + "/effort");
        series.pushBack({ timestamp, msg.effort[i] });
      }
    }
  }

private:
  std::vector<PJ::PlotData*> _data;
};
