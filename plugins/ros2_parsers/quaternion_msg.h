#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include "ros2_parser.h"

class QuaternionMsgParser : public BuiltinMessageParser<geometry_msgs::msg::Quaternion>
{
public:
  QuaternionMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::msg::Quaternion>(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const geometry_msgs::msg::Quaternion& msg, double& timestamp) override
  {
    if( !_initialized )
    {
      _initialized = true;
      _data.push_back(&getSeries(_topic_name + "/x"));
      _data.push_back(&getSeries(_topic_name + "/y"));
      _data.push_back(&getSeries(_topic_name + "/z"));
      _data.push_back(&getSeries(_topic_name + "/w"));

      _data.push_back(&getSeries(_topic_name + "/roll_deg"));
      _data.push_back(&getSeries(_topic_name + "/pitch_deg"));
      _data.push_back(&getSeries(_topic_name + "/yaw_deg"));
    }
    _data[0]->pushBack({ timestamp, msg.x });
    _data[1]->pushBack({ timestamp, msg.y });
    _data[2]->pushBack({ timestamp, msg.z });
    _data[3]->pushBack({ timestamp, msg.w });

    //-----------------------------
    auto q = msg;
    double quat_norm2 = (q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z);
    if (std::abs(quat_norm2 - 1.0) > std::numeric_limits<double>::epsilon())
    {
      double mult = 1.0 / std::sqrt(quat_norm2);
      q.x *= mult;
      q.y *= mult;
      q.z *= mult;
      q.w *= mult;
    }

    double roll, pitch, yaw;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
    {
      pitch = std::copysign(M_PI_2, sinp);  // use 90 degrees if out of range
    }
    else
    {
      pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    const double RAD_TO_DEG = 180.0 / M_PI;

    _data[4]->pushBack({ timestamp, RAD_TO_DEG * roll });
    _data[5]->pushBack({ timestamp, RAD_TO_DEG * pitch });
    _data[6]->pushBack({ timestamp, RAD_TO_DEG * yaw });
  }

private:
  std::vector<PJ::PlotData*> _data;
  bool _initialized = false;
};
