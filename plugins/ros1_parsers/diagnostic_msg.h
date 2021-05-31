#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <boost/spirit/include/qi.hpp>
#include "fmt/format.h"
#include "ros1_parser.h"

class DiagnosticMsgParser : public BuiltinMessageParser<diagnostic_msgs::DiagnosticArray>
{
public:
  DiagnosticMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<diagnostic_msgs::DiagnosticArray>(topic_name, plot_data)
  {
    _data.emplace_back(&getSeries("/header/seq"));
    _data.emplace_back(&getSeries("/header/stamp"));
  }

  virtual void parseMessageImpl(const diagnostic_msgs::DiagnosticArray& msg, double& timestamp) override
  {
    double header_stamp = msg.header.stamp.toSec();
    timestamp = (_use_message_stamp && header_stamp > 0) ? header_stamp : timestamp;

    _data[0]->pushBack({ timestamp, (double)msg.header.seq });
    _data[1]->pushBack({ timestamp, header_stamp });

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

private:
  std::vector<PJ::PlotData*> _data;
};
