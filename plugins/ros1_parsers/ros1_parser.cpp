#include "ros1_parser.h"
#include "jointstates_msg.h"
#include "imu_msg.h"
#include "diagnostic_msg.h"
#include "odometry_msg.h"
#include "pal_statistics_msg.h"
#include "tf_msg.h"
#include "fiveai_stamped_diagnostic.h"
#include "plotjuggler_msgs.h"


bool IntrospectionParser::parseMessage(MessageRef serialized_msg, double& timestamp)
{
  RosIntrospection::Span<uint8_t> span( serialized_msg.data(), serialized_msg.size() );
  _parser.deserializeIntoFlatContainer(_topic_name, span, &_flat_msg, _config.max_array_size);

  if (_config.use_header_stamp)
  {
    for (const auto& it : _flat_msg.value)
    {
      if (it.second.getTypeID() != RosIntrospection::TIME)
      {
        continue;
      }
      const RosIntrospection::StringTreeNode* leaf1 = it.first.node_ptr;
      const RosIntrospection::StringTreeNode* leaf2 = leaf1->parent();
      if (leaf2 && leaf2->value() == "header" && leaf1->value() == "stamp")
      {
        double header_stamp = it.second.convert<double>();

        if (header_stamp > 0)
        {
          timestamp = header_stamp;
        }
        break;
      }
    }
  }

  _parser.applyNameTransform(_topic_name, _flat_msg, &_renamed);

  for (const auto& it : _renamed)
  {
    const auto& key = it.first;
    double value = 0;

    if( it.second.getTypeID() ==  RosIntrospection::BuiltinType::UINT64 )
    {
        uint64_t raw_value = it.second.extract<uint64_t>();
        //      if( raw_value >= (1l<<53) ){
        //        TODO: warn the user?
        //      }
        value = static_cast<double>(raw_value);
    }
    else if( it.second.getTypeID() ==  RosIntrospection::BuiltinType::INT64 )
    {
        int64_t raw_value = it.second.extract<int64_t>();
        //      if( raw_value >= (1l<<53) ){
        //        TODO: warn the user?
        //      }
        value = static_cast<double>(raw_value);
    }
    else if( it.second.getTypeID() ==  RosIntrospection::BuiltinType::STRING )
    {
        // special case for strings
        auto str = it.second.extract<std::string>();
        bool parsed = PJ::ParseDouble(str, value,
                                      _config.remove_suffix_from_strings,
                                      _config.boolean_strings_to_number);

        if( !parsed && _plot_data.numeric.count(key) == 0 )
        {
          auto& series = _plot_data.getOrCreateStringSeries( key );
          series.pushBack({ timestamp, str});
        }
        continue;
    }
    else{
      value = it.second.convert<double>();
    }

    auto& series = getSeries(key);

    if (!std::isnan(static_cast<double>(value)) && !std::isinf(value))
    {
      series.pushBack({ timestamp, value });
    }
  }
  return true;
}

//-----------------------------------------

void RosCompositeParser::registerMessageType(const std::string& topic_name,
                                             const std::string& topic_type,
                                             const std::string& definition)
{
  std::shared_ptr<RosMessageParser> parser;
  if (_parsers.count(topic_name) > 0)
  {
    return;
  }

  const std::string& type = topic_type;

  if (type == "sensor_msgs/JointState")
  {
    parser.reset(new JointStateMsgParser(topic_name, _plot_data));
  }
  else if (type == "diagnostic_msgs/DiagnosticArray")
  {
    parser.reset(new DiagnosticMsgParser(topic_name, _plot_data));
  }
  else if (type == "tf/tfMessage")
  {
    parser.reset(new TfMsgParser(topic_name, _plot_data));
  }
  else if (type == "tf2_msgs/TFMessage")
  {
    parser.reset(new Tf2MsgParser(topic_name, _plot_data));
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
  else if (type == "geometry_msgs/PoseWithCovarianceStamped")
  {
    parser.reset(new PoseCovarianceStampedMsgParser(topic_name, _plot_data));
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
  else if (type == "pal_statistics_msgs/StatisticsNames")
  {
    parser.reset(new PalStatisticsNamesParser(topic_name, _plot_data));
  }
  else if (type == "pal_statistics_msgs/StatisticsValues")
  {
    parser.reset(new PalStatisticsValuesParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/Dictionary")
  {
    parser.reset(new PlotJugglerDictionaryParser(topic_name, _plot_data));
  }
  else if (type == "plotjuggler_msgs/DataPoints")
  {
    parser.reset(new PlotJugglerDataPointsParser(topic_name, _plot_data));
  }
  else if (type == "fiveai_node_msgs/NodeDiagnostics")
  {
    parser.reset(new FiveAiDiagnosticMsg(topic_name, _plot_data));
  }
  else
  {
    parser.reset(new IntrospectionParser(topic_name, type, definition, _plot_data));
  }

  parser->setConfig(_config);
  _parsers.insert({ topic_name, parser });
}



