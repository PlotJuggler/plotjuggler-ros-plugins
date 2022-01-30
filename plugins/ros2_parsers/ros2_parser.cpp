#include "ros2_parser.h"
#include "jointstates_msg.h"
#include "imu_msg.h"
#include "odometry_msg.h"
#include "tf_msg.h"
#include "plotjuggler_msgs.h"
#include "diagnostic_msg.h"
#include "pj_statistics_msg.h"
//  #include "pal_statistics_msg.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

bool IntrospectionParser::parseMessage(MessageRef serialized_msg, double& timestamp)
{
  rcutils_uint8_array_t msg_ref;
  msg_ref.buffer = serialized_msg.data();
  msg_ref.buffer_length = serialized_msg.size();

  _intropection_parser.deserializeIntoFlatMessage(&msg_ref, &_flat_msg);

  if (_config.use_header_stamp && _intropection_parser.topicInfo().has_header_stamp)
  {
    double sec = _flat_msg.values[0].second;
    double nsec = _flat_msg.values[1].second;
    timestamp = sec + (nsec * 1e-9);
  }

  std::string key;
  bool header_found = false;

  // special case: messages which start with header
  if( _flat_msg.values.size() >= 2 )
  {
    _flat_msg.values[0].first.toStr(key);

    if( boost::algorithm::ends_with(key, "/header/stamp/sec" ) )
    {
      _flat_msg.values[1].first.toStr(key);
      if( boost::algorithm::ends_with(key, "/header/stamp/nanosec" ) )
      {
        header_found = true;
        double header_stamp = _flat_msg.values[0].second +
                              _flat_msg.values[1].second * 1e-9;

        constexpr size_t suffix_length = sizeof("/nanosec") - 1;

        auto new_name = key.substr(0, key.size() - suffix_length );
        auto& series = getSeries(new_name);
        series.pushBack({ timestamp, header_stamp });
      }
    }
  }

  for (size_t i = header_found ? 2:0; i < _flat_msg.values.size(); i++)
  {
    const auto& it = _flat_msg.values[i];

    it.first.toStr(key);
    double value = it.second;

    auto& series = getSeries(key);
    if (!std::isnan(value) && !std::isinf(value))
    {
      series.pushBack({ timestamp, value });
    }
  }

  for (const auto& it : _flat_msg.strings)
  {
    it.first.toStr(key);
    auto& series = getStringSeries(key);
    series.pushBack({ timestamp, it.second });
  }

  return true;
}

//-----------------------------------------

void Ros2CompositeParser::registerMessageType(const std::string& topic_name,
                                              const std::string& topic_type)
{
  std::shared_ptr<RosMessageParser> parser;
  if (_parsers.count(topic_name) > 0)
  {
    return;
  }

  std::string type = topic_type;

  // replace verbose name
  size_t str_index = type.find("/msg/", 0);
  if (str_index != std::string::npos)
  {
    type.erase(str_index, 4);
  }

  if (type == "sensor_msgs/JointState")
  {
    parser.reset(new JointStateMsgParser(topic_name, _plot_data));
  }
  else if (type == "diagnostic_msgs/DiagnosticArray")
  {
    parser.reset(new DiagnosticMsgParser(topic_name, _plot_data));
  }
  else if (type == "tf2_msgs/TFMessage")
  {
    parser.reset(new TfMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/Quaternion")
  {
    parser.reset(new QuaternionMsgParser(topic_name, _plot_data));
  }
  else if (type == "sensor_msgs/Imu")
  {
    parser.reset(new ImuMsgParser(topic_name, _plot_data));
  }
  else if (type == "nav_msgs/Odometry")
  {
    parser.reset(new OdometryMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/Pose")
  {
    parser.reset(new PoseMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/PoseStamped")
  {
    parser.reset(new PoseStampedMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/PoseWithCovariance")
  {
    parser.reset(new PoseCovarianceMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/Twist")
  {
    parser.reset(new TwistMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/TwistStamped")
  {
    parser.reset(new TwistStampedMsgParser(topic_name, _plot_data));
  }
  else if (type == "geometry_msgs/TwistWithCovariance")
  {
    parser.reset(new TwistCovarianceMsgParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/Dictionary")
  {
    parser.reset(new PlotJugglerDictionaryParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/DataPoints")
  {
    parser.reset(new PlotJugglerDataPointsParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/StatisticsNames")
  {
    parser.reset(new PJ_StatisticsNamesParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/StatisticsValues")
  {
    parser.reset(new PJ_StatisticsValuesParser(topic_name, _plot_data));
  }
//  else if (type == "pal_statistics_msgs/StatisticsNames")
//  {
//    parser.reset(new PAL_StatisticsNamesParser(topic_name, _plot_data));
//  }
//  else if (type == "pal_statistics_msgs/StatisticsValues")
//  {
//    parser.reset(new PAL_StatisticsValuesParser(topic_name, _plot_data));
//  }
  else
  {
    parser.reset(new IntrospectionParser(topic_name, type, _plot_data));
  }

  parser->setConfig(_config);
  _parsers.insert({ topic_name, parser });
}

const rosidl_message_type_support_t *
Ros2CompositeParser::typeSupport(const std::string &topic_name) const
{
  auto it = _parsers.find(topic_name);
  if (it == _parsers.end())
  {
    return nullptr;
  }
  if( auto parser = dynamic_cast<Ros2MessageParser*>(it->second.get()))
  {
    return parser->typeSupport();
  }
  return nullptr;
}

