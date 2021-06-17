#pragma once

#include <sensor_msgs/Imu.h>
#include "fmt/format.h"
#include "ros1_parser.h"
#include "covariance_util.h"
#include "quaternion_msg.h"

class ImuMsgParser : public BuiltinMessageParser<sensor_msgs::Imu>
{
public:
  ImuMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<sensor_msgs::Imu>(topic_name, plot_data)
    , _quat_parser(topic_name + "/orientation", plot_data)
    , _orientation_covariance(topic_name + "/orientation_covariance", plot_data)
    , _lin_acc_covariance(topic_name + "/linear_acceleration_covariance", plot_data)
    , _ang_vel_covariance(topic_name + "/angular_velocity_covariance", plot_data)
  {
    _data.emplace_back(&getSeries(topic_name + "/header/seq"));
    _data.emplace_back(&getSeries(topic_name + "/header/stamp"));

    _data.push_back(&getSeries(topic_name + "/angular_velocity/x"));
    _data.push_back(&getSeries(topic_name + "/angular_velocity/y"));
    _data.push_back(&getSeries(topic_name + "/angular_velocity/z"));

    _data.push_back(&getSeries(topic_name + "/linear_acceleration/x"));
    _data.push_back(&getSeries(topic_name + "/linear_acceleration/y"));
    _data.push_back(&getSeries(topic_name + "/linear_acceleration/z"));
  }

  void parseMessageImpl(const sensor_msgs::Imu& msg, double& timestamp) override
  {
    double header_stamp = msg.header.stamp.toSec();
    timestamp = (_use_message_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _data[0]->pushBack({ timestamp, double(msg.header.seq) });
    _data[1]->pushBack({ timestamp, header_stamp });

    _data[2]->pushBack({ timestamp, msg.angular_velocity.x });
    _data[3]->pushBack({ timestamp, msg.angular_velocity.y });
    _data[4]->pushBack({ timestamp, msg.angular_velocity.z });

    _data[5]->pushBack({ timestamp, msg.linear_acceleration.x });
    _data[6]->pushBack({ timestamp, msg.linear_acceleration.y });
    _data[7]->pushBack({ timestamp, msg.linear_acceleration.z });

    _quat_parser.parseMessageImpl(msg.orientation, timestamp);

    _orientation_covariance.parse(msg.orientation_covariance, timestamp);
    _lin_acc_covariance.parse(msg.linear_acceleration_covariance, timestamp);
    _ang_vel_covariance.parse(msg.angular_velocity_covariance, timestamp);
  }

private:
  QuaternionMsgParser _quat_parser;
  CovarianceParser<3> _orientation_covariance;
  CovarianceParser<3> _lin_acc_covariance;
  CovarianceParser<3> _ang_vel_covariance;
  std::vector<PJ::PlotData*> _data;
};
