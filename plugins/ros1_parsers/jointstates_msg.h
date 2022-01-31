#pragma once

#include <sensor_msgs/JointState.h>
#include "ros1_parser.h"
#include "header_msg.h"

class JointStateMsgParser : public BuiltinMessageParser<sensor_msgs::JointState>
{
public:
  JointStateMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<sensor_msgs::JointState>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
  {
  }

  void parseMessageImpl(const sensor_msgs::JointState& msg, double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _config.use_header_stamp);

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
  HeaderMsgParser _header_parser;
};
