#pragma once

#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>
#include "fmt/format.h"
#include "ros1_parser.h"
#include "header_msg.h"

template <typename TfMsgType>
class TfMsgParserImpl : public BuiltinMessageParser<TfMsgType>
{

public:
  using BaseParser = BuiltinMessageParser<TfMsgType>;
  using MessageParser::getSeries;
  using MessageParser::getStringSeries;

  TfMsgParserImpl(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BaseParser(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const TfMsgType& msg, double& timestamp) override
  {
    for (const auto& trans : msg.transforms)
    {
      double header_stamp = trans.header.stamp.toSec();
      timestamp = (BaseParser::getConfig().use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

      std::string prefix;
      if (trans.header.frame_id.empty())
      {
        prefix = fmt::format("{}/{}", BaseParser::_topic_name, trans.child_frame_id);
      }
      else
      {
        prefix = fmt::format("{}/{}/{}",
                             BaseParser::_topic_name,
                             trans.header.frame_id,
                             trans.child_frame_id);
      }

      PlotData* series;

      series = &getSeries(prefix + "/header/stamp");
      series->pushBack({ timestamp, header_stamp });

      series = &getSeries(prefix + "/header/seq");
      series->pushBack({ timestamp, double(trans.header.seq) });

      series = &getSeries(prefix + "/translation/x");
      series->pushBack({ timestamp, trans.transform.translation.x });

      series = &getSeries(prefix + "/translation/y");
      series->pushBack({ timestamp, trans.transform.translation.y });

      series = &getSeries(prefix + "/translation/z");
      series->pushBack({ timestamp, trans.transform.translation.z });

      series = &getSeries(prefix + "/rotation/x");
      series->pushBack({ timestamp, trans.transform.rotation.x });

      series = &getSeries(prefix + "/rotation/y");
      series->pushBack({ timestamp, trans.transform.rotation.y });

      series = &getSeries(prefix + "/rotation/z");
      series->pushBack({ timestamp, trans.transform.rotation.z });

      series = &getSeries(prefix + "/rotation/w");
      series->pushBack({ timestamp, trans.transform.rotation.w });
    }
  }
};

using TfMsgParser = TfMsgParserImpl<tf::tfMessage>;
using Tf2MsgParser = TfMsgParserImpl<tf2_msgs::TFMessage>;
