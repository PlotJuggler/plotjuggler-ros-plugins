#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <boost/spirit/include/qi.hpp>
#include <boost/algorithm/string.hpp>
#include "fmt/format.h"
#include "ros1_parser.h"
#include "header_msg.h"

class DiagnosticMsgParser : public BuiltinMessageParser<diagnostic_msgs::DiagnosticArray>
{
public:
  DiagnosticMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<diagnostic_msgs::DiagnosticArray>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
  {
  }

  virtual void parseMessageImpl(const diagnostic_msgs::DiagnosticArray& msg,
                                double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _config.use_header_stamp);

    std::string key;

    for (const auto& status : msg.status)
    {
      for (const auto& kv : status.values)
      {
        double value = 0;

        if (status.hardware_id.empty())
        {
          key = fmt::format("{}/{}/{}", _topic_name, status.name, kv.key);
        }
        else
        {
          key = fmt::format("{}/{}/{}/{}", _topic_name, status.hardware_id, status.name, kv.key);
        }

        bool parsed = PJ::ParseDouble(kv.value, value,
                                      _config.remove_suffix_from_strings,
                                      _config.boolean_strings_to_number);
        if (parsed)
        {
          auto& series = getSeries(key);
          series.pushBack({ timestamp, value });
        }
        else{
          if(_plot_data.numeric.count(key) == 0)
          {
            auto& series = getStringSeries(key);
            series.pushBack( { timestamp, kv.value} );
          }
        }
      }
    }
  }

private:
  HeaderMsgParser _header_parser;

};
