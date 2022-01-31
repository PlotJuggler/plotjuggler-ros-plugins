#pragma once

#include <tf2_msgs/msg/tf_message.hpp>
#include "fmt/format.h"
#include "ros2_parser.h"

class TfMsgParser : public BuiltinMessageParser<tf2_msgs::msg::TFMessage>
{
public:

  TfMsgParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data) :
    BuiltinMessageParser<tf2_msgs::msg::TFMessage>(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const tf2_msgs::msg::TFMessage& msg, double& timestamp) override
  {
    using namespace PJ;

    for (const auto& trans : msg.transforms)
    {
      double header_stamp = double(trans.header.stamp.sec) +
                            double(trans.header.stamp.nanosec) * 1e-9;
      timestamp = (_config.use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

      std::string prefix;
      if (trans.header.frame_id.empty())
      {
        prefix = fmt::format("{}/{}", _topic_name, trans.child_frame_id);
      }
      else
      {
        prefix = fmt::format("{}/{}/{}", _topic_name, trans.header.frame_id, trans.child_frame_id);
      }

      PlotData* series = &getSeries( prefix + "/header/stamp");
      series->pushBack({ timestamp, header_stamp });

      series = &getSeries( prefix + "/translation/x");
      series->pushBack({ timestamp, trans.transform.translation.x });

      series = &getSeries( prefix + "/translation/y");
      series->pushBack({ timestamp, trans.transform.translation.y });

      series = &getSeries( prefix + "/translation/z");
      series->pushBack({ timestamp, trans.transform.translation.z });

      series = &getSeries( prefix + "/rotation/x");
      series->pushBack({ timestamp, trans.transform.rotation.x });

      series = &getSeries( prefix + "/rotation/y");
      series->pushBack({ timestamp, trans.transform.rotation.y });

      series = &getSeries( prefix + "/rotation/z");
      series->pushBack({ timestamp, trans.transform.rotation.z });

      series = &getSeries( prefix + "/rotation/w");
      series->pushBack({ timestamp, trans.transform.rotation.w });
    }
  }
private:

};

