#pragma once

#include <sensor_msgs/JointState.h>
#include "ros1_parser.h"

class JointStateMsgParser : public BuiltinMessageParser<sensor_msgs::JointState>
{
public:
  JointStateMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<sensor_msgs::JointState>(topic_name, plot_data)
  {
    _data.emplace_back(&getSeries(topic_name + "/header/seq"));
    _data.emplace_back(&getSeries(topic_name + "/header/stamp"));
  }

  void parseMessageImpl(const sensor_msgs::JointState& msg, double& timestamp) override
  {
    double header_stamp = msg.header.stamp.toSec();
    timestamp = (_use_message_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _data[0]->pushBack({ timestamp, double(msg.header.seq) });
    _data[1]->pushBack({ timestamp, header_stamp });

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
