#pragma once

#include "plotjuggler_msgs/msg/dictionary.hpp"
#include "plotjuggler_msgs/msg/data_points.hpp"
#include "ros2_parser.h"

static std::unordered_map<unsigned, std::vector<std::string>> _plotjuggler_msgs_dictionaries;

class PlotJugglerDictionaryParser : public BuiltinMessageParser<plotjuggler_msgs::msg::Dictionary>
{
public:
  PlotJugglerDictionaryParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<plotjuggler_msgs::msg::Dictionary>(topic_name, plot_data)
  {
  }

  virtual void setMaxArrayPolicy(Ros2Introspection::MaxArrayPolicy, size_t)
  {
  }

  void parseMessageImpl(const plotjuggler_msgs::msg::Dictionary& msg, double timestamp) override
  {
    _plotjuggler_msgs_dictionaries[msg.dictionary_uuid] = msg.names;
  }
};

//------------------------------------
class PlotJugglerDataPointsParser : public BuiltinMessageParser<plotjuggler_msgs::msg::DataPoints>
{
public:
  PlotJugglerDataPointsParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : BuiltinMessageParser<plotjuggler_msgs::msg::DataPoints>(topic_name, plot_data)
  {
    _prefix = topic_name + "/";
  }

  virtual void setMaxArrayPolicy(Ros2Introspection::MaxArrayPolicy, size_t)
  {
  }

  void parseMessageImpl(const plotjuggler_msgs::msg::DataPoints& msg, double timestamp) override
  {
    auto it = _plotjuggler_msgs_dictionaries.find(msg.dictionary_uuid);
    if (it == _plotjuggler_msgs_dictionaries.end())
    {
      //        const auto& names = it->second;
      //        for( const auto& sample: msg.samples)
      //        {
      //          auto& series = getSeries(_plot_data, _prefix + std::to_string(sample.name_index));
      //          series.pushBack( {sample.stamp, sample.value} );
      //        }
      // just skip... ?
    }
    else
    {
      const auto& names = it->second;
      for (const auto& sample : msg.samples)
      {
        auto& series = getSeries(_plot_data, _prefix + names[sample.name_index]);
        series.pushBack({ sample.stamp, sample.value });
      }
    }
  }

private:
  std::string _prefix;
};
