#pragma once

#include <nav_msgs/Odometry.h>
#include "pose_msg.h"
#include "twist_msg.h"
#include "ros1_parser.h"

class OdometryMsgParser : public BuiltinMessageParser<nav_msgs::Odometry>
{
public:
  OdometryMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<nav_msgs::Odometry>(topic_name, plot_data)
    , _pose_parser(topic_name + "/pose", plot_data)
    , _twist_parser(topic_name + "/twist", plot_data)
  {
    _data.emplace_back(&getSeries(topic_name + "/header/seq"));
    _data.emplace_back(&getSeries(topic_name + "/header/stamp"));
  }

  void parseMessageImpl(const nav_msgs::Odometry& msg, double& timestamp) override
  {
    double header_stamp = msg.header.stamp.toSec();
    timestamp = (_use_message_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _data[0]->pushBack({ timestamp, double(msg.header.seq) });
    _data[1]->pushBack({ timestamp, header_stamp });

    _pose_parser.parseMessageImpl(msg.pose, timestamp);
    _twist_parser.parseMessageImpl(msg.twist, timestamp);
  }

private:
  PoseCovarianceMsgParser _pose_parser;
  TwistCovarianceMsgParser _twist_parser;
  std::vector<PJ::PlotData*> _data;
};
