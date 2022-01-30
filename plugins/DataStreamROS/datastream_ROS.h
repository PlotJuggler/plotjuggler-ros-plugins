#ifndef DATASTREAM_ROS_TOPIC_H
#define DATASTREAM_ROS_TOPIC_H

#include <QtPlugin>
#include <QAction>
#include <QTimer>
#include <thread>
#include <ros_type_introspection/utils/shape_shifter.hpp>
#include <PlotJuggler/datastreamer_base.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <rosgraph_msgs/Clock.h>
#include "qnodedialog.h"
#include "dialog_select_ros_topics.h"
#include "ros1_parsers/ros1_parser.h"

class DataStreamROS : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataStreamer")
  Q_INTERFACES(PJ::DataStreamer)

public:
  DataStreamROS();

  virtual bool start(QStringList* selected_datasources) override;

  virtual void shutdown() override;

  virtual bool isRunning() const override;

  virtual ~DataStreamROS() override;

  virtual const char* name() const override
  {
    return "ROS Topic Subscriber";
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

  virtual const std::vector<QAction*>& availableActions() override;

private:
  void topicCallback(const RosIntrospection::ShapeShifter::ConstPtr& msg, const std::string& topic_name);

  void extractInitialSamples();

  void timerCallback();

  void subscribe();

  bool _running;
  bool _first_warning;

  std::shared_ptr<ros::AsyncSpinner> _spinner;

  double _initial_time;

  std::string _prefix;

  ros::NodeHandlePtr _node;

  std::map<std::string, ros::Subscriber> _subscribers;

  RosIntrospection::SubstitutionRuleMap _rules;

  int _received_msg_count;

  QAction* _action_saveIntoRosbag;

  QAction* _action_saveAny;

  std::vector<QAction*> _available_actions;

  std::map<std::string, int> _msg_index;

  RosParserConfig _config;

  std::unique_ptr<RosCompositeParser> _parser;

  QTimer* _periodic_timer;

  double _prev_clock_time;

  std::unordered_map<std::string, PlotDataAny> _user_defined_buffers;

private:
  void saveIntoRosbag();
};

#endif  // DATALOAD_CSV_H
