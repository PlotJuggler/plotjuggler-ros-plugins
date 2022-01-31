#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <QObject>
#include <QtPlugin>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <PlotJuggler/dataloader_base.h>
#include "dialog_select_ros_topics.h"
#include "ros1_parsers/ros1_parser.h"

class DataLoadROS : public PJ::DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataLoader"
                        "../dataloader.json")
  Q_INTERFACES(PJ::DataLoader)

public:
  DataLoadROS();

  virtual ~DataLoadROS() override;

  virtual const std::vector<const char*>& compatibleFileExtensions() const override;

  virtual bool readDataFromFile(PJ::FileLoadInfo* fileload_info,
                                PJ::PlotDataMapRef& destination) override;

  virtual const char* name() const override
  {
    return "DataLoad ROS bags";
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

protected:
  void loadSubstitutionRule(QStringList all_topic_names);

  std::shared_ptr<rosbag::Bag> _bag;

private:
  std::vector<const char*> _extensions;

  RosParserConfig _config;

  std::vector<std::pair<QString, QString>> getAllTopics(const rosbag::Bag* bag, RosCompositeParser &parser);
};

#endif  // DATALOAD_CSV_H
