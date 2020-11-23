#ifndef STATE_PUBLISHER_ROS2TOPIC_H
#define STATE_PUBLISHER_ROS2TOPIC_H

#include <QObject>
#include <QtPlugin>
#include <map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <PlotJuggler/statepublisher_base.h>
#include "ros2_parser.h"
#include "generic_publisher.h"

using MessageRefPtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;


class TopicPublisherROS2 : public PJ::StatePublisher
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.StatePublisher")
  Q_INTERFACES(PJ::StatePublisher)

public:
  TopicPublisherROS2();

  virtual ~TopicPublisherROS2() override;

  void updateState(double current_time) override;

  const char* name() const override
  {
    return "ROS2 Topic Re-Publisher";
  }

  bool enabled() const override
  {
    return _enabled;
  }

  const std::vector<QAction*>& availableActions() override;

  void play(double interval) override;

public slots:

  void setEnabled(bool enabled) override;

  void filterDialog();

private:

  std::shared_ptr<rclcpp::Context> _context;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
  std::shared_ptr<rclcpp::Node> _node;

  bool _enabled;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;

  std::unordered_map<std::string, std::shared_ptr<GenericPublisher>> _publishers;

  QAction* _select_topics_to_publish;

  std::unordered_map<std::string, bool> _topics_to_publish;

  double previous_time;

  int _previous_play_index;

  std::vector<TopicInfo> _topics_info;

  std::vector<QAction*> _available_actions;

  void broadcastTF(double current_time);

  void updatePublishers();
};

#endif  // STATE_PUBLISHER_ROS2TOPIC_H
