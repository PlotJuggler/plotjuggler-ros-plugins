#pragma once

#include <unordered_map>

#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include "pal_statistics_msgs/msg/statistics_values.hpp"

#include "ros2_parser.h"
#include "fmt/format.h"
#include "header_msg.h"

using TopicStatistics = std::unordered_map<uint32_t, std::vector<std::string>>;
static std::unordered_map<std::string, TopicStatistics> _stored_pal_statistics_names;


class PAL_StatisticsNamesParser : public BuiltinMessageParser<pal_statistics_msgs::msg::StatisticsNames>
{
public:
  PAL_StatisticsNamesParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<pal_statistics_msgs::msg::StatisticsNames>(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const pal_statistics_msgs::msg::StatisticsNames& msg, double& timestamp) override
  {
    std::string values_topic_name = _topic_name;
    values_topic_name.replace(values_topic_name.find("names"), 5, "values");
    _stored_pal_statistics_names[values_topic_name].insert({ msg.names_version, msg.names });
  }
};

//-----------------------------------------------------
class PAL_StatisticsValuesParser : public BuiltinMessageParser<pal_statistics_msgs::msg::StatisticsValues>
{
public:
  PAL_StatisticsValuesParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<pal_statistics_msgs::msg::StatisticsValues>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
  {
  }

  void parseMessageImpl(const pal_statistics_msgs::msg::StatisticsValues& msg, double& timestamp) override
  {
    auto& values = _data[msg.names_version];

    _header_parser.parse(msg.header, timestamp, _config.use_header_stamp);
    auto& statistics_map = _stored_pal_statistics_names[_topic_name];

    auto names_it = statistics_map.find(msg.names_version);
    if (names_it == statistics_map.end())
    {
      return;  // missing vocabulary
    }
    const auto& names = names_it->second;

    if (msg.values.size() != names.size())
    {
      return;  // weird... skip
    }

    for (size_t index = 0; index < msg.values.size(); index++)
    {
      const auto& name = names[index];
      if (index >= values.size())
      {
        values.emplace_back(&getSeries(fmt::format("{}/{}",_topic_name, name)));
      }
      values[index]->pushBack({ timestamp, msg.values[index] });
    }
  }

private:
  std::unordered_map<uint32_t, std::vector<PJ::PlotData*>> _data;
  HeaderMsgParser _header_parser;
};

