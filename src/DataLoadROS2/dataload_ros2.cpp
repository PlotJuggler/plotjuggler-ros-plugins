#include "dataload_ros2.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QDir>
#include <QProcess>
#include <QSettings>
#include <QElapsedTimer>
#include <set>
#include <QDebug>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "dialog_select_ros_topics.h"
#include "ros_parsers/ros2_parser.h"

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

DataLoadROS2::DataLoadROS2()
{
  _extensions.push_back("yaml");
  loadDefaultSettings();
}

const std::vector<const char*>& DataLoadROS2::compatibleFileExtensions() const
{
  return _extensions;
}

bool DataLoadROS2::readDataFromFile(PJ::FileLoadInfo* info,
                                    PJ::PlotDataMapRef& plot_map)
{
  auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();

  QString bagDir;
  {
    QFileInfo finfo(info->filename);
    bagDir = finfo.dir().path();
  }

  rosbag2_storage::StorageOptions storageOptions;
  const auto bag_metadata = metadata_io->read_metadata(bagDir.toStdString());
  storageOptions.uri = bagDir.toStdString();
  storageOptions.storage_id = bag_metadata.storage_identifier;
  rosbag2_cpp::ConverterOptions converterOptions;
  converterOptions.input_serialization_format = "cdr";
  converterOptions.output_serialization_format = rmw_get_serialization_format();

  std::shared_ptr<rosbag2_cpp::Reader> temp_bag_reader =
      rosbag2_transport::ReaderWriterFactory::make_reader(storageOptions);

  QString oldPath = QDir::currentPath();
  QDir::setCurrent(QDir::cleanPath(bagDir + QDir::separator() + ".."));

  try
  {
    temp_bag_reader->open(storageOptions, converterOptions);
  }
  catch (std::runtime_error& ex)
  {
    QMessageBox::warning(nullptr, tr("Error"), QString("rosbag::open thrown an exception:\n") + QString(ex.what()));
    return false;
  }

  QDir::setCurrent(oldPath);

  // Temporarily change the current directory as a workaround for rosbag2 relative directories not working properly
  std::vector<rosbag2_storage::TopicMetadata> topic_metadata = temp_bag_reader->get_all_topics_and_types();

  std::unordered_map<std::string, std::string> topicTypesByName;

  std::vector<std::pair<QString, QString>> all_topics_qt;

  std::vector<TopicInfo> topics_info;

  std::set<std::string> blacklist_topic_name;
  std::set<std::string> failed_topic_type;

  for (const rosbag2_storage::TopicMetadata& topic : topic_metadata)
  {
    all_topics_qt.push_back( {QString::fromStdString(topic.name),
                              QString::fromStdString(topic.type)} );
    topicTypesByName.emplace(topic.name, topic.type);

    const auto& typesupport_identifier = rosidl_typesupport_cpp::typesupport_identifier;
    try
    {
      topics_info.emplace_back( CreateTopicInfo(topic.name, topic.type) );

    } catch (...) {
      failed_topic_type.insert(topic.type);
      blacklist_topic_name.insert(topic.type);
    }
  }

  if(!failed_topic_type.empty())
  {
    QString msg("Can not recognize the following message types and those topics will be ignored:\n\n");
    for(const auto& type: failed_topic_type)
    {
      msg += "  " + QString::fromStdString(type) + "\n";
    }
    msg += "\nHave you forgotten to source your workspace?";
    QMessageBox::warning(nullptr, tr("Error"), msg);
  }

  // FIXME: we keep this for backward compatibility
  if (info->plugin_config.hasChildNodes())
  {
    xmlLoadState(info->plugin_config.firstChildElement());
  }

  if (!info->plugin_config.hasChildNodes() || _config.topics.empty())
  {
    loadDefaultSettings();
    DialogSelectRosTopics* dialog = new DialogSelectRosTopics(all_topics_qt, _config);

    if (dialog->exec() != static_cast<int>(QDialog::Accepted))
    {
      delete dialog;
      return false;
    }
    _config = dialog->getResult();
    delete dialog;
  }

  //--- Swith the previous bag with this one
  // clean up previous MessageInstances
  plot_map.user_defined.clear();

  if (_bag_reader)
  {
    _bag_reader->close();
  }
  _bag_reader = temp_bag_reader;
  //---------------------------------------

  saveDefaultSettings();

  PJ::CompositeParser parser;

  std::set<std::string> topic_selected;
  for (const auto& topic_qt : _config.topics)
  {
    std::string topic_name = topic_qt.toStdString();
    std::string topic_type = topicTypesByName.at(topic_name);
    topic_selected.insert(topic_name);

    auto ros2_parser = CreateParserROS2( *parserFactories(), 
                                         topic_name, topic_type, plot_map);
    parser.addParser(topic_name, ros2_parser);

  }

  parser.setConfig(_config);

  QProgressDialog progress_dialog;
  progress_dialog.setWindowTitle("Loading the rosbag");
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.setRange(0, bag_metadata.message_count);
  progress_dialog.show();
  int msg_count = 0;

  PJ::PlotDataAny& plot_consecutive =
      plot_map.addUserDefined("plotjuggler::rosbag2_cpp::consecutive_messages")->second;
  PJ::PlotDataAny& metadata_storage =
      plot_map.addUserDefined("plotjuggler::rosbag2_cpp::topics_metadata")->second;

  // dirty trick. Store it in a one point timeseries
  metadata_storage.pushBack( {0, std::any(topics_info) } );

  auto time_prev = std::chrono::high_resolution_clock::now();

  while (_bag_reader->has_next())
  {
    auto msg = _bag_reader->read_next();
    const std::string& topic_name = msg->topic_name;
    if(blacklist_topic_name.count(topic_name))
    {
      continue;
    }

#ifdef ROS_HUMBLE
    const double msg_timestamp = 1e-9 * double(msg->time_stamp);  // nanoseconds to seconds
#else
    // from jazzy and later
    const double msg_timestamp = 1e-9 * double(msg->send_timestamp);  // nanoseconds to seconds
#endif

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

    //----- skip not selected -----------
    if (topic_selected.find(topic_name) == topic_selected.end())
    {
      continue;
    }
    //----- parse! -----------
    double timestamp = msg_timestamp;
    PJ::MessageRef msg_ref(msg->serialized_data->buffer, msg->serialized_data->buffer_length);
    parser.parseMessage(topic_name, msg_ref, timestamp);

    //---- save msg reference in PlotAny ----
    auto data_point = PJ::PlotDataAny::Point(timestamp, std::any(msg));
    plot_consecutive.pushBack(data_point);

    auto plot_pair = plot_map.user_defined.find(topic_name);
    if (plot_pair == plot_map.user_defined.end())
    {
      plot_pair = plot_map.addUserDefined(topic_name);
    }
    PJ::PlotDataAny& plot_raw = plot_pair->second;
    plot_raw.pushBack(data_point);
  }

  auto now = std::chrono::high_resolution_clock::now();
  double diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_prev).count();

  qDebug() << "The rosbag loaded the data in " << diff << " milliseconds";

  return true;
}

bool DataLoadROS2::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  _config.xmlSaveState(doc, plugin_elem);
  return true;
}

bool DataLoadROS2::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

void DataLoadROS2::saveDefaultSettings()
{
  QSettings settings;
  _config.saveToSettings(settings, "DataLoadROS2");
}

void DataLoadROS2::loadDefaultSettings()
{
  QSettings settings;
  _config.loadFromSettings(settings, "DataLoadROS2");
}
