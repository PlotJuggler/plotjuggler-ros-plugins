#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QProcess>
#include <rosbag/view.h>
#include <QSettings>
#include <QElapsedTimer>
#include <condition_variable>
#include <functional>
#include <thread>

#include "dialog_select_ros_topics.h"
#include "shape_shifter_factory.hpp"
#include "rule_editing.h"
#include "dialog_with_itemlist.h"

DataLoadROS::DataLoadROS()
{
  _extensions.push_back("bag");
  QSettings settings;
  _config.loadFromSettings(settings, "DataLoadROS");
}

DataLoadROS::~DataLoadROS()
{
}

void StrCat(const std::string& a, const std::string& b, std::string& out)
{
  out.clear();
  out.reserve(a.size() + b.size());
  out.assign(a);
  out.append(b);
}

const std::vector<const char*>& DataLoadROS::compatibleFileExtensions() const
{
  return _extensions;
}

std::vector<std::pair<QString, QString>>
DataLoadROS::getAllTopics(const rosbag::Bag* bag, RosCompositeParser& parser)
{
  std::vector<std::pair<QString, QString>> all_topics;
  rosbag::View bag_view(*bag, ros::TIME_MIN, ros::TIME_MAX, true);

  bool ignoreAll = false;

  for (auto& conn : bag_view.getConnections())
  {
    const auto& topic = conn->topic;
    const auto& md5sum = conn->md5sum;
    const auto& datatype = conn->datatype;
    const auto& definition = conn->msg_def;

    all_topics.push_back(std::make_pair(QString(topic.c_str()), QString(datatype.c_str())));
    try
    {
      parser.registerMessageType(topic, datatype, definition);

      RosIntrospectionFactory::registerMessage(topic, md5sum, datatype, definition);
    }
    catch (std::exception& ex)
    {
      // there was a problem with this topic
      // a real life problem example can be found here:
      // https://github.com/rosjava/rosjava_bootstrap/issues/16
      all_topics.pop_back();

      if (ignoreAll)
      {
        // this is not the first error with this load and the
        // user has accepted to ignore all errors
        continue;
      }

      // prompt user to abort or continue
      QMessageBox msgBox(nullptr);
      msgBox.setWindowTitle("ROS bag error");
      msgBox.setText(QString("Topic ") + QString(topic.c_str()) + QString(": ") + QString(ex.what()));

      QPushButton* buttonCancel = msgBox.addButton(tr("Cancel"), QMessageBox::RejectRole);
      QPushButton* buttonIgnore = msgBox.addButton(tr("Ignore"), QMessageBox::YesRole);
      QPushButton* buttonIgnoreAll = msgBox.addButton(tr("Ignore all"), QMessageBox::AcceptRole);
      msgBox.setDefaultButton(buttonIgnoreAll);
      msgBox.exec();
      if (msgBox.clickedButton() == buttonCancel)
      {
        // abort the file loading
        throw;
      }
      if (msgBox.clickedButton() == buttonIgnoreAll)
      {
        // accept this and all future errors for this load
        ignoreAll = true;
      }
    }
  }
  return all_topics;
}

bool DataLoadROS::readDataFromFile(PJ::FileLoadInfo* info, PJ::PlotDataMapRef& plot_map)
{
  auto temp_bag = std::make_shared<rosbag::Bag>();

  try
  {
    temp_bag->open(info->filename.toStdString(), rosbag::bagmode::Read);
  }
  catch (rosbag::BagException& ex)
  {
    QMessageBox::warning(nullptr, tr("Error"), QString("rosbag::open thrown an exception:\n") + QString(ex.what()));
    return false;
  }

  RosCompositeParser ros_parser(plot_map);
  auto all_topics = getAllTopics(temp_bag.get(), ros_parser);

  //----------------------------------

  if (info->plugin_config.hasChildNodes())
  {
    xmlLoadState(info->plugin_config.firstChildElement());
  }

  if (!info->selected_datasources.empty())
  {
    _config.topics = info->selected_datasources;
  }
  else
  {
    DialogSelectRosTopics* dialog = new DialogSelectRosTopics(all_topics, _config);

    if (dialog->exec() != static_cast<int>(QDialog::Accepted))
    {
      return false;
    }
    _config = dialog->getResult();
  }

  //--- Swith the previous bag with this one
  // clean up previous MessageInstances
  plot_map.user_defined.clear();
  if (_bag)
  {
    _bag->close();
  }
  _bag = temp_bag;
  //---------------------------------------

  {
    QSettings settings;
    _config.saveToSettings(settings, "DataLoadROS");
  }

  ros_parser.setConfig(_config);

  //-----------------------------------
  std::set<std::string> topic_selected;
  for (const auto& topic : _config.topics)
  {
    topic_selected.insert(topic.toStdString());
  }

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);

  rosbag::View bag_view(*_bag);

  progress_dialog.setRange(0, bag_view.size() - 1);
  progress_dialog.show();

  int msg_count = 0;

  QElapsedTimer timer;
  timer.start();

  PlotDataAny& plot_consecutive = plot_map.addUserDefined("plotjuggler::rosbag1::consecutive_messages")->second;

  std::vector<uint8_t> buffer;

  for (const rosbag::MessageInstance& msg_instance : bag_view)
  {
    const std::string& topic_name = msg_instance.getTopic();
    double msg_time = msg_instance.getTime().toSec();

    //----- skip not selected -----------
    if (topic_selected.find(topic_name) == topic_selected.end())
    {
      continue;
    }

    //------ progress dialog --------------
    if (msg_count++ % 100 == 0)
    {
      progress_dialog.setValue(msg_count);
      QApplication::processEvents();

      if (progress_dialog.wasCanceled())
      {
        return false;
      }
    }

    //----- parse! -----------
    const size_t msg_size = msg_instance.size();
    buffer.resize(msg_size);
    ros::serialization::OStream stream(buffer.data(), msg_size);
    msg_instance.write(stream);

    MessageRef msg_serialized(buffer.data(), buffer.size());

    double tmp_timestamp = msg_time;
    ros_parser.parseMessage(topic_name, msg_serialized, tmp_timestamp);

    //------ save msg reference in PlotAny ----
    auto data_point = PlotDataAny::Point(tmp_timestamp, std::any(msg_instance));
    plot_consecutive.pushBack(data_point);

    auto plot_pair = plot_map.user_defined.find(topic_name);
    if (plot_pair == plot_map.user_defined.end())
    {
      plot_pair = plot_map.addUserDefined(topic_name);
    }
    PlotDataAny& plot_raw = plot_pair->second;
    plot_raw.pushBack(data_point);
  }

  qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

  info->selected_datasources = _config.topics;
  return true;
}

bool DataLoadROS::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  _config.xmlSaveState(doc, plugin_elem);
  return true;
}

bool DataLoadROS::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

