#pragma once

#include <plotjuggler_msgs/Dictionary.h>
#include <plotjuggler_msgs/DataPoints.h>
#include "ros1_parser.h"

static std::unordered_map<unsigned, std::vector<std::string>> _plotjuggler_msgs_dictionaries;

class PlotJugglerDictionaryParser : public BuiltinMessageParser<plotjuggler_msgs::Dictionary>
{
public:
  PlotJugglerDictionaryParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<plotjuggler_msgs::Dictionary>(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const plotjuggler_msgs::Dictionary& msg, double& timestamp) override
  {
    _plotjuggler_msgs_dictionaries[msg.dictionary_uuid] = msg.names;
  }
};

//------------------------------------
class PlotJugglerDataPointsParser : public BuiltinMessageParser<plotjuggler_msgs::DataPoints>
{
public:
  PlotJugglerDataPointsParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<plotjuggler_msgs::DataPoints>(topic_name, plot_data)
  {
    _prefix = topic_name + "/";
  }

  void parseMessageImpl(const plotjuggler_msgs::DataPoints& msg, double& timestamp) override
  {
    auto it = _plotjuggler_msgs_dictionaries.find(msg.dictionary_uuid);
    if (it == _plotjuggler_msgs_dictionaries.end())
    {
      const auto& names = it->second;
      for (const auto& sample : msg.samples)
      {
        auto& series = getSeries(_prefix + std::to_string(sample.name_index));
        series.pushBack({ sample.stamp, sample.value });
      }
    }
    else
    {
      const auto& names = it->second;
      for (const auto& sample : msg.samples)
      {
        auto& series = getSeries(_prefix + names[sample.name_index]);
        series.pushBack({ sample.stamp, sample.value });
      }
    }
  }

private:
  std::string _prefix;
};
