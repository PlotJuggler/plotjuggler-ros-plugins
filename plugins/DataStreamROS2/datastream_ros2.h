#ifndef DATASTREAM_ROS2_TOPIC_H
#define DATASTREAM_ROS2_TOPIC_H

#include <QtPlugin>
#include <QTimer>

#include <PlotJuggler/datastreamer_base.h>

#include "dialog_select_ros_topics.h"
#include "ros2_parsers/ros2_parser.h"
#include "ros2_parsers/generic_subscription.hpp"

class DataStreamROS2 : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.ROS2DataStreamer")
  Q_INTERFACES(PJ::DataStreamer)

public:
  DataStreamROS2();

  bool start(QStringList* selected_datasources) override;

  void shutdown() override;

  bool isRunning() const override;

  ~DataStreamROS2() override;

  const char* name() const override
  {
    return "ROS2 Topic Subscriber";
  }

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

  const std::vector<QAction*>& availableActions() override;

private:
  void messageCallback(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg);

private:
  std::shared_ptr<rclcpp::Context> _context;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
  std::shared_ptr<rclcpp::Node> _node;

  std::unique_ptr<Ros2CompositeParser> _parser;

  bool _running;
  bool _first_warning;

  std::thread _spinner;

  RosParserConfig _config;

  rclcpp::Clock _clock;

  rcl_time_point_value_t _start_time;

  void saveDefaultSettings();

  void loadDefaultSettings();

  std::unordered_map<std::string, rosbag2_transport::GenericSubscription::Ptr> _subscriptions;

  void subscribeToTopic(const std::string& topic_name, const std::string& topic_type);
  void waitOneSecond();
};

#endif
