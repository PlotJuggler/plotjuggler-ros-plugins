#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <boost/spirit/include/qi.hpp>
#include "ros2_parser.h"
#include "fmt/format.h"
#include "header_msg.h"

class DiagnosticMsgParser : public BuiltinMessageParser<diagnostic_msgs::msg::DiagnosticArray>
{
public:
 DiagnosticMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
     : BuiltinMessageParser<diagnostic_msgs::msg::DiagnosticArray>(topic_name, plot_data)
       , _header_parser(topic_name + "/header", plot_data)
 {
 }

  virtual void parseMessageImpl(const diagnostic_msgs::msg::DiagnosticArray& msg, double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _use_header_stamp);

    std::string key;

    for (const auto& status : msg.status)
    {
      for (const auto& kv : status.values)
      {
        const char* start_ptr = kv.value.data();
        std::vector<double> val;

        if (status.hardware_id.empty())
        {
          key = fmt::format("{}/{}/{}", _topic_name, status.name, kv.key);
        }
        else
        {
          key = fmt::format("{}/{}/{}/{}", _topic_name, status.hardware_id, status.name, kv.key);
        }

        bool parsed = boost::spirit::qi::phrase_parse(start_ptr, start_ptr + kv.value.size(),
                                               boost::spirit::qi::double_ % ',', boost::spirit::qi::ascii::space, val);
        if (val.size() > 1)
        {
          for (size_t index = 0; index < val.size(); index++)
          {
            std::string subkey = fmt::format("{}/{}", key, index);
            auto& series = getSeries(subkey);
            series.pushBack({ timestamp, val[index] });  
          }
        }
        else if (parsed)
        {
          auto& series = getSeries(key);
          series.pushBack({ timestamp, val[0] });
        }
        else{
          auto& series = getStringSeries(key);
          series.pushBack( { timestamp, kv.value} );
        }
      }
    }
  }

 private:
  HeaderMsgParser _header_parser;
};

