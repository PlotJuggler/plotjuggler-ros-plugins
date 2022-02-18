#ifndef FIVEAI_STAMPED_DIAGNOSTIC_H
#define FIVEAI_STAMPED_DIAGNOSTIC_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <boost/algorithm/string.hpp>
#include "ros1_parser.h"

struct StampedDiagnostic_
{
  uint8_t status;
  ros::Time stamp;
  std::string key;
  std::string value;
};

struct NodeDiagnostics_
{
  std::vector<StampedDiagnostic_> diagnostics;
};
//-----------------------------------------------------

namespace ros
{
namespace serialization
{
template <>
struct Serializer< ::StampedDiagnostic_>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.status);
    stream.next(m.stamp);
    stream.next(m.key);
    stream.next(m.value);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

template <>
struct Serializer< ::NodeDiagnostics_>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.diagnostics);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER
};

}  // namespace serialization
}  // namespace ros

//-----------------------------------------------------

class FiveAiDiagnosticMsg : public RosMessageParser
{
  public:
  FiveAiDiagnosticMsg(const std::string& topic_name,
                      PJ::PlotDataMapRef& plot_data)
      : RosMessageParser(topic_name, plot_data)
  {}

  bool parseMessage(MessageRef msg, double& timestamp) override
  {
    NodeDiagnostics_ diagnostic;
    ros::serialization::IStream is(const_cast<uint8_t*>(msg.data()), msg.size());
    ros::serialization::deserialize(is, diagnostic);

    for (const StampedDiagnostic_& diag : diagnostic.diagnostics)
    {
      timestamp = diag.stamp.toSec();

      double value = 0;
      bool parsed = PJ::ParseDouble(diag.value, value,
                                    _config.remove_suffix_from_strings,
                                    _config.boolean_strings_to_number);

      std::string replaced_key = diag.key;
      std::replace(replaced_key.begin(), replaced_key.end(), ' ', '_');

      if (parsed)
      {
        auto key = fmt::format("{}/{}/value", _topic_name, replaced_key);
        auto& series = getSeries(key);
        series.pushBack({ timestamp, value });
      }
      else
      {
        auto key = fmt::format("{}/{}/value", _topic_name, replaced_key);
        auto& series = getStringSeries(key);
        series.pushBack({ timestamp, diag.value });
      }

      {
        auto key = fmt::format("{}/{}/status", _topic_name, replaced_key);
        auto& series = getSeries(key);
        series.pushBack({ timestamp, static_cast<double>(diag.status) });
      }

    }
    return true;
  }

};

#endif  // FIVEAI_STAMPED_DIAGNOSTIC_H
