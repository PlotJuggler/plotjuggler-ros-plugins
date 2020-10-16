#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include "quaternion_msg.h"
#include "ros2_parser.h"

class TwistMsgParser : public BuiltinMessageParser<geometry_msgs::msg::Twist>
{
public:
  TwistMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::msg::Twist>(topic_name, plot_data)
  {
    _data.push_back(&getSeries(plot_data, topic_name + "/linear/x"));
    _data.push_back(&getSeries(plot_data, topic_name + "/linear/y"));
    _data.push_back(&getSeries(plot_data, topic_name + "/linear/z"));

    _data.push_back(&getSeries(plot_data, topic_name + "/angular/x"));
    _data.push_back(&getSeries(plot_data, topic_name + "/angular/y"));
    _data.push_back(&getSeries(plot_data, topic_name + "/angular/z"));
  }

  void parseMessageImpl(const geometry_msgs::msg::Twist& msg, double timestamp) override
  {
    _data[0]->pushBack({ timestamp, msg.linear.x });
    _data[1]->pushBack({ timestamp, msg.linear.y });
    _data[2]->pushBack({ timestamp, msg.linear.z });

    _data[3]->pushBack({ timestamp, msg.angular.x });
    _data[4]->pushBack({ timestamp, msg.angular.y });
    _data[5]->pushBack({ timestamp, msg.angular.z });
  }

private:
  std::vector<PlotData*> _data;
};

class TwistStampedMsgParser : public BuiltinMessageParser<geometry_msgs::msg::TwistStamped>
{
public:
  TwistStampedMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::msg::TwistStamped>(topic_name, plot_data)
    , _twist_parser(topic_name, plot_data)
  {
    _data.push_back(&getSeries(plot_data, topic_name + "/header/stamp/sec"));
    _data.push_back(&getSeries(plot_data, topic_name + "/header/stamp/nanosec"));
  }

  void parseMessageImpl(const geometry_msgs::msg::TwistStamped& msg, double timestamp) override
  {
    if (_use_header_stamp)
    {
      timestamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    }

    _data[0]->pushBack({ timestamp, double(msg.header.stamp.sec) });
    _data[1]->pushBack({ timestamp, double(msg.header.stamp.nanosec) });

    _twist_parser.parseMessageImpl(msg.twist, timestamp);
  }

private:
  TwistMsgParser _twist_parser;
  std::vector<PlotData*> _data;
};

class TwistCovarianceMsgParser : public BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>
{
public:
  TwistCovarianceMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>(topic_name, plot_data)
    , _twist_parser(topic_name, plot_data)
    , _covariance(topic_name + "/covariance", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::msg::TwistWithCovariance& msg, double timestamp) override
  {
    _twist_parser.parseMessageImpl(msg.twist, timestamp);
    _covariance.parse(msg.covariance, timestamp);
  }

private:
  TwistMsgParser _twist_parser;
  CovarianceParser<6> _covariance;
};
