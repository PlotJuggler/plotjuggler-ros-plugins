#pragma once

#include <geometry_msgs/Quaternion.h>
#include "ros1_parser.h"

class HeaderMsgParser
{

public:
  HeaderMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
  {
    auto plot_data_ptr = &plot_data;
    _lazy_init = [=]()
    {
      _stamp    = &plot_data_ptr->getOrCreateNumberic(topic_name + "/seq");
      _sequence = &plot_data_ptr->getOrCreateNumberic(topic_name + "/stamp");
      _frame_id = &plot_data_ptr->getOrCreateStringSeries(topic_name + "/frame_id");
    };
  }

  void parse(const std_msgs::Header& msg,
             double& timestamp,
             bool use_header_stamp)
  {
    if( !_initialized )
    {
      _initialized = true;
      _lazy_init();
    }

    double header_stamp = msg.stamp.toSec();
    timestamp = (use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _sequence->pushBack({ timestamp, double(msg.seq) });
    _stamp->pushBack({ timestamp, header_stamp });
    _frame_id->pushBack({ timestamp, msg.frame_id });
  }

private:
  PJ::PlotData* _stamp;
  PJ::PlotData* _sequence;
  PJ::StringSeries* _frame_id;
  std::function<void()> _lazy_init;
  bool _initialized = false;
};
