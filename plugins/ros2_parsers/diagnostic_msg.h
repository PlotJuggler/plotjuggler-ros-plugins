#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <boost/spirit/include/qi.hpp>
#include "ros2_parser.h"
#include "fmt/format.h"

class DiagnosticMsgParser : public BuiltinMessageParser<diagnostic_msgs::msg::DiagnosticArray>
{
public:
 DiagnosticMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<diagnostic_msgs::msg::DiagnosticArray>(topic_name, plot_data)
  {
  }

  virtual void parseMessageImpl(const diagnostic_msgs::msg::DiagnosticArray& msg, double& timestamp) override
  {
    double header_stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    timestamp = (_use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

    auto stamp_series = &getSeries("/header/seq");
    stamp_series->pushBack( {timestamp, header_stamp} );

    std::string key;

    for (const auto& status : msg.status)
    {
      for (const auto& kv : status.values)
      {
        const char* start_ptr = kv.value.data();
        double val = 0;

        bool parsed = boost::spirit::qi::parse(start_ptr, start_ptr + kv.value.size(),
                                               boost::spirit::qi::double_, val);
        if (!parsed){
          continue;
        }

        if (status.hardware_id.empty())
        {
          key = fmt::format("{}/{}/{}", _topic_name, status.name, kv.key);
        }
        else
        {
          key = fmt::format("{}/{}/{}/{}", _topic_name, status.hardware_id, status.name, kv.key);
        }
        auto& series = getSeries(key);
        series.pushBack({ timestamp, val });
      }
    }
  }

};

