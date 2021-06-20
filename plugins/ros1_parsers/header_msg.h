#pragma once

#include <geometry_msgs/Quaternion.h>
#include "ros1_parser.h"

class HeaderMsgParser
{

 public:
  HeaderMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
      : _topic_name( topic_name )
        , _plot_data( plot_data )
  {

  }

  void parse(const std_msgs::Header& msg,
             double& timestamp,
             bool use_header_stamp)
  {
    if( !_initialized )
    {
      _initialized = true;

      _sequence = &plot_data_ptr->getOrCreateNumeric(topic_name + "/seq");
      _stamp    = &plot_data_ptr->getOrCreateNumeric(topic_name + "/stamp");
      _frame_id = &plot_data_ptr->getOrCreateStringSeries(topic_name + "/frame_id");
    }

    double header_stamp = msg.stamp.toSec();
    timestamp = (use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _sequence->pushBack({ timestamp, double(msg.seq) });
    _stamp->pushBack({ timestamp, header_stamp });
    _frame_id->pushBack({ timestamp, msg.frame_id });
  }

 private:
  const std::string _topic_name;
  PJ::PlotDataMapRef& _plot_data;
  bool _initialized = false;

  PJ::PlotData* _stamp;
  PJ::PlotData* _sequence;
  PJ::StringSeries* _frame_id;
};
