#ifndef DATALOAD_ROS2_H
#define DATALOAD_ROS2_H

#include <QObject>
#include <QtPlugin>
#include <QSettings>

#include <rosbag2/readers/sequential_reader.hpp>
#include "PlotJuggler/optional.hpp"
#include "PlotJuggler/dataloader_base.h"
#include "ros2_parsers/ros2_parser.h"
#include "dialog_select_ros_topics.h"

class DataLoadROS2 : public DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.ROS2DataLoader")
  Q_INTERFACES(DataLoader)

public:
  DataLoadROS2();

  virtual const std::vector<const char*>& compatibleFileExtensions() const override;

  virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

  virtual const char* name() const override
  {
    return "DataLoad ROS2 bags";
  }

  virtual ~DataLoadROS2() override = default;

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

private:
  std::shared_ptr<rosbag2::readers::SequentialReader> _bag_reader;

  PlotDataMapRef _temp_plot_map;

  CompositeParser _parser;

  std::vector<const char*> _extensions;

  DialogSelectRosTopics::Configuration _config;

  std::vector<std::pair<QString, QString>> getAndRegisterAllTopics();

  void saveDefaultSettings();

  void loadDefaultSettings();
};

#endif  // DATALOAD_ROS2_H
