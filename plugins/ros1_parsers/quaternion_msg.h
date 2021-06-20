#pragma once

#include <geometry_msgs/Quaternion.h>
#include "ros1_parser.h"

class QuaternionMsgParser : public BuiltinMessageParser<geometry_msgs::Quaternion>
{
public:
  QuaternionMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<geometry_msgs::Quaternion>(topic_name, plot_data),
      _pitch_offset(0.0),
      _roll_offset(0.0),
      _yaw_offset(0.0),
      _prev_pitch(0.0),
      _prev_roll(0.0),
      _prev_yaw(0.0)
  {
  }

  void parseMessageImpl(const geometry_msgs::Quaternion& msg, double& timestamp) override
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
    const double WRAP_ANGLE = M_PI*2.0;
    const double WRAP_THRESHOLD = M_PI*1.95;

    //--------- wrap ------
    if( (roll - _prev_roll) > WRAP_THRESHOLD ) {
      _roll_offset -= WRAP_ANGLE;
    }
    else if( (_prev_roll - roll) > WRAP_THRESHOLD ) {
      _roll_offset += WRAP_ANGLE;
    }
    _prev_roll = roll;

    if( (pitch - _prev_pitch) > WRAP_THRESHOLD ) {
      _pitch_offset -= WRAP_ANGLE;
    }
    else if( (_prev_pitch - pitch) > WRAP_THRESHOLD ) {
      _pitch_offset += WRAP_ANGLE;
    }
    _prev_pitch = pitch;

    if( (yaw - _prev_yaw) > WRAP_THRESHOLD ) {
      _yaw_offset -= WRAP_ANGLE;
    }
    else if( (_prev_yaw - yaw) > WRAP_THRESHOLD ) {
      _yaw_offset += WRAP_ANGLE;
    }
    _prev_yaw = yaw;
    //---------------

    _data[4]->pushBack({ timestamp, RAD_TO_DEG * (roll + _roll_offset)});
    _data[5]->pushBack({ timestamp, RAD_TO_DEG * (pitch + _pitch_offset) });
    _data[6]->pushBack({ timestamp, RAD_TO_DEG * (yaw + _yaw_offset)});
  }

 private:
  std::vector<PJ::PlotData*> _data;
  double _pitch_offset;
  double _roll_offset;
  double _yaw_offset;

  double _prev_pitch;
  double _prev_roll;
  double _prev_yaw;

  bool _initialized = false;

};
