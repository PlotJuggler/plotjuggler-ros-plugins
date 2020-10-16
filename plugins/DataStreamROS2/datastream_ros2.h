#ifndef DATASTREAM_ROS2_TOPIC_H
#define DATASTREAM_ROS2_TOPIC_H

#include <QtPlugin>
#include <QTimer>

#include "PlotJuggler/datastreamer_base.h"

#include "dialog_select_ros_topics.h"
#include "ros2_parsers/ros2_parser.h"
#include "ros2_parsers/generic_subscription.hpp"

class DataStreamROS2 : public DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.ROS2DataStreamer")
  Q_INTERFACES(DataStreamer)

public:
  DataStreamROS2();

  virtual bool start(QStringList* selected_datasources) override;

  virtual void shutdown() override;

  virtual bool isRunning() const override;

  virtual ~DataStreamROS2() override;

  virtual const char* name() const override
  {
    return "ROS2 Topic Subscriber";
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

  virtual void addActionsToParentMenu(QMenu* menu) override;

private:
  void messageCallback(const std::string& topic_name, std::shared_ptr<rmw_serialized_message_t> msg);

private:
  std::shared_ptr<rclcpp::Context> _context;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
  std::shared_ptr<rclcpp::Node> _node;

  std::unique_ptr<CompositeParser> _parser;

  bool _running;

  std::thread _spinner;

  DialogSelectRosTopics::Configuration _config;

  rclcpp::Clock _clock;
  rcl_time_point_value_t _start_time;

  void saveDefaultSettings();

  void loadDefaultSettings();

  std::unordered_map<std::string, rosbag2_transport::GenericSubscription::Ptr> _subscriptions;

  void subscribeToTopic(const std::string& topic_name, const std::string& topic_type);
  void waitOneSecond();
};

#endif
