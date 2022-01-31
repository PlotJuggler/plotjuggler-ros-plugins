#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "covariance_util.h"
#include "quaternion_msg.h"
#include "ros1_parser.h"
#include "header_msg.h"

class PoseMsgParser : public BuiltinMessageParser<geometry_msgs::Pose>
{
public:
  PoseMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::Pose>(topic_name, plot_data)
    , _quat_parser(topic_name + "/orientation", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::Pose& msg, double& timestamp) override
  {
    if( !_initialized )
    {
      _initialized = true;
      _data.push_back(&getSeries(_topic_name + "/position/x"));
      _data.push_back(&getSeries(_topic_name + "/position/y"));
      _data.push_back(&getSeries(_topic_name + "/position/z"));
    }

    _data[0]->pushBack({ timestamp, msg.position.x });
    _data[1]->pushBack({ timestamp, msg.position.y });
    _data[2]->pushBack({ timestamp, msg.position.z });

    _quat_parser.parseMessageImpl(msg.orientation, timestamp);
  }

private:
  QuaternionMsgParser _quat_parser;
  std::vector<PJ::PlotData*> _data;
  bool _initialized = false;
};

class PoseStampedMsgParser : public BuiltinMessageParser<geometry_msgs::PoseStamped>
{
public:
  PoseStampedMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::PoseStamped>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
    , _pose_parser(topic_name + "/pose", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::PoseStamped& msg, double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _config.use_header_stamp);
    _pose_parser.parseMessageImpl(msg.pose, timestamp);
  }

private:
  HeaderMsgParser _header_parser;
  PoseMsgParser _pose_parser;
  std::vector<PJ::PlotData*> _data;
};

class PoseCovarianceMsgParser
    : public BuiltinMessageParser<geometry_msgs::PoseWithCovariance>
{
public:
  PoseCovarianceMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::PoseWithCovariance>(topic_name, plot_data)
    , _pose_parser(topic_name + "/pose", plot_data)
    , _covariance(topic_name + "/covariance", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::PoseWithCovariance& msg, double& timestamp) override
  {
    _pose_parser.parseMessageImpl(msg.pose, timestamp);
    _covariance.parse(msg.covariance, timestamp);
  }

private:
  PoseMsgParser _pose_parser;
  CovarianceParser<6> _covariance;
};

class PoseCovarianceStampedMsgParser : public BuiltinMessageParser<geometry_msgs::PoseWithCovarianceStamped>
{
public:
  PoseCovarianceStampedMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::PoseWithCovarianceStamped>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
    , _pose_cov_parser(topic_name + "/pose", plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::PoseWithCovarianceStamped& msg, double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _config.use_header_stamp);
    _pose_cov_parser.parseMessageImpl(msg.pose, timestamp);
  }

private:
  HeaderMsgParser _header_parser;
  PoseCovarianceMsgParser _pose_cov_parser;
};
