#pragma once

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include "quaternion_msg.h"
#include "ros1_parser.h"

class TwistMsgParser : public BuiltinMessageParser<geometry_msgs::Twist>
{
public:
  TwistMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::Twist>(topic_name, plot_data)
  {
    _data.push_back(&getSeries(topic_name + "/linear/x"));
    _data.push_back(&getSeries(topic_name + "/linear/y"));
    _data.push_back(&getSeries(topic_name + "/linear/z"));

    _data.push_back(&getSeries(topic_name + "/angular/x"));
    _data.push_back(&getSeries(topic_name + "/angular/y"));
    _data.push_back(&getSeries(topic_name + "/angular/z"));
  }

  void parseMessageImpl(const geometry_msgs::Twist& msg, double& timestamp) override
  {
    _data[0]->pushBack({ timestamp, msg.linear.x });
    _data[1]->pushBack({ timestamp, msg.linear.y });
    _data[2]->pushBack({ timestamp, msg.linear.z });

    _data[3]->pushBack({ timestamp, msg.angular.x });
    _data[4]->pushBack({ timestamp, msg.angular.y });
    _data[5]->pushBack({ timestamp, msg.angular.z });
  }

private:
  std::vector<PJ::PlotData*> _data;
};

class TwistStampedMsgParser : public BuiltinMessageParser<geometry_msgs::TwistStamped>
{
public:
  TwistStampedMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::TwistStamped>(topic_name, plot_data), _twist_parser(topic_name, plot_data)
  {
    _data.emplace_back(&getSeries(topic_name + "/header/seq"));
    _data.emplace_back(&getSeries(topic_name + "/header/stamp"));
  }

  void parseMessageImpl(const geometry_msgs::TwistStamped& msg, double& timestamp) override
  {
    double header_stamp = msg.header.stamp.toSec();
    timestamp = (_use_message_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _data[0]->pushBack({ timestamp, double(msg.header.seq) });
    _data[1]->pushBack({ timestamp, header_stamp });

    _twist_parser.parseMessageImpl(msg.twist, timestamp);
  }

private:
  TwistMsgParser _twist_parser;
  std::vector<PJ::PlotData*> _data;
};

class TwistCovarianceMsgParser : public BuiltinMessageParser<geometry_msgs::TwistWithCovariance>
{
public:
  TwistCovarianceMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::TwistWithCovariance>(topic_name, plot_data)
    , _twist_parser(topic_name, plot_data)
    , _covariance(topic_name + "/covariance", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::TwistWithCovariance& msg, double& timestamp) override
  {
    _twist_parser.parseMessageImpl(msg.twist, timestamp);
    _covariance.parse(msg.covariance, timestamp);
  }

private:
  TwistMsgParser _twist_parser;
  CovarianceParser<6> _covariance;
};
