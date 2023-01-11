#ifndef PAL_STATISTICS_MSG_H
#define PAL_STATISTICS_MSG_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <std_msgs/Header.h>
#include "ros1_parser.h"

struct PalStatisticsNames_
{
  //    Header header
  //    string[] names
  //    uint32 names_version #This is increased each time names change
  std_msgs::Header header;
  std::vector<std::string> names;
  uint32_t names_version;
};

struct PalStatisticsValues_
{
  //    Header header
  //    float64[] names
  //    uint32 names_version #This is increased each time names change
  std_msgs::Header header;
  std::vector<double> values;
  uint32_t names_version;
};

//-----------------------------------------------------

namespace ros
{
namespace serialization
{
template <>
struct Serializer<::PalStatisticsNames_>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.names);
    stream.next(m.names_version);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

template <>
struct Serializer<::PalStatisticsValues_>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.values);
    stream.next(m.names_version);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

}  // namespace serialization
}  // namespace ros

//-----------------------------------------------------

using TopicStatistics = std::unordered_map<uint32_t, std::vector<std::string>>;
static std::unordered_map<std::string, TopicStatistics> _stored_pal_statistics_names;

class PalStatisticsNamesParser : public RosMessageParser
{
public:
  PalStatisticsNamesParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : RosMessageParser(topic_name, plot_data)
  {
  }

  virtual bool parseMessage(MessageRef msg, double& timestamp) override
  {
    PalStatisticsNames_ pal_names;
    ros::serialization::IStream is(const_cast<uint8_t*>(msg.data()), msg.size());
    ros::serialization::deserialize(is, pal_names);
    std::string values_topic_name = _topic_name;
    values_topic_name.replace(values_topic_name.find("names"), 5, "values");
    _stored_pal_statistics_names[values_topic_name].insert(
        { pal_names.names_version, std::move(pal_names.names) });
    return true;
  }
};

//-----------------------------------------------------
class PalStatisticsValuesParser : public RosMessageParser
{
public:
  PalStatisticsValuesParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : RosMessageParser(topic_name, plot_data)
  {
  }

  virtual bool parseMessage(MessageRef msg, double& timestamp) override
  {
    PalStatisticsValues_ pal_msg;
    ros::serialization::IStream is(const_cast<uint8_t*>(msg.data()), msg.size());
    ros::serialization::deserialize(is, pal_msg);

    auto& values = _data[pal_msg.names_version];

    double header_stamp = pal_msg.header.stamp.toSec();
    timestamp = (_config.use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

    auto& statistics_map = _stored_pal_statistics_names[_topic_name];
    auto names_it = statistics_map.find(pal_msg.names_version);
    if (names_it == statistics_map.end())
    {
      return false;  // missing vocabulary
    }
    const auto& names = names_it->second;

    if (pal_msg.values.size() != names.size())
    {
      return false;  // weird... skip
    }

    for (size_t index = 0; index < pal_msg.values.size(); index++)
    {
      const auto& name = names[index];
      if (index >= values.size())
      {
        values.emplace_back(&getSeries(fmt::format("{}/{}",_topic_name, name)));
      }
      values[index]->pushBack({ timestamp, pal_msg.values[index] });
    }
    return true;
  }

private:
  std::unordered_map<uint32_t, std::vector<PJ::PlotData*>> _data;
};

#endif  // PAL_STATISTICS_MSG_H
