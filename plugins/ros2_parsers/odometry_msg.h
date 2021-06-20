#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include "pose_msg.h"
#include "twist_msg.h"
#include "ros2_parser.h"

class OdometryMsgParser : public BuiltinMessageParser<nav_msgs::msg::Odometry>
{
public:
  OdometryMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<nav_msgs::msg::Odometry>(topic_name, plot_data)
    , _pose_parser(topic_name + "/pose", plot_data)
    , _twist_parser(topic_name + "/twist", plot_data)
  {
    _data.push_back(&getSeries(topic_name + "/header/stamp"));
  }

  void parseMessageImpl(const nav_msgs::msg::Odometry& msg, double& timestamp) override
  {
    double header_stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    if (_use_header_stamp)
    {
      timestamp = header_stamp;
    }

    _data[0]->pushBack({ timestamp, header_stamp });

    _pose_parser.parseMessageImpl(msg.pose, timestamp);
    _twist_parser.parseMessageImpl(msg.twist, timestamp);
  }

private:
  PoseCovarianceMsgParser _pose_parser;
  TwistCovarianceMsgParser _twist_parser;
  std::vector<PJ::PlotData*> _data;
};
