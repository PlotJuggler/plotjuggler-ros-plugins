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
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "../dialog_select_ros_topics.h"

DataLoadROS2::DataLoadROS2() : _parser(_temp_plot_map)
{
  _extensions.push_back("yaml");
  loadDefaultSettings();
}

const std::vector<const char*>& DataLoadROS2::compatibleFileExtensions() const
{
  return _extensions;
}

bool DataLoadROS2::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_map)
{
  auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();

  auto temp_bag_reader = std::make_shared<rosbag2::readers::SequentialReader>();

  QString bagDir;
  {
    QFileInfo finfo(info->filename);
    bagDir = finfo.dir().path();
  }

  rosbag2::StorageOptions storageOptions;
  storageOptions.uri = bagDir.toStdString();
  storageOptions.storage_id = "sqlite3";
  rosbag2::ConverterOptions converterOptions;
  converterOptions.input_serialization_format = "cdr";
  converterOptions.output_serialization_format = rmw_get_serialization_format();

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
  const auto bag_metadata = metadata_io->read_metadata(storageOptions.uri);

  QDir::setCurrent(oldPath);

  // Temporarily change the current directory as a workaround for rosbag2 relative directories not working properly
  std::vector<rosbag2::TopicMetadata> topic_metadata = temp_bag_reader->get_all_topics_and_types();

  std::unordered_map<std::string, std::string> topicTypesByName;

  std::vector<std::pair<QString, QString>> all_topics_qt;

  std::vector<TopicInfo> topics_info;

  for (const rosbag2::TopicMetadata& topic : topic_metadata)
  {
    all_topics_qt.push_back( {QString::fromStdString(topic.name),
                              QString::fromStdString(topic.type)} );
    topicTypesByName.emplace(topic.name, topic.type);

    TopicInfo topic_info;
    topic_info.name = topic.name;
    topic_info.type = topic.type;
    topic_info.type_support = rosbag2::get_typesupport(
        topic.type, rosidl_typesupport_cpp::typesupport_identifier);
    topics_info.emplace_back( std::move(topic_info) );
  }

  if (info->plugin_config.hasChildNodes())
  {
    xmlLoadState(info->plugin_config.firstChildElement());
  }

  if (!info->selected_datasources.empty())
  {
    _config.selected_topics = info->selected_datasources;
  }
  else
  {
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
    _bag_reader->reset();
  }
  _bag_reader = temp_bag_reader;
  //---------------------------------------

  saveDefaultSettings();

  std::set<std::string> topic_selected;
  for (const auto& topic_qt : _config.selected_topics)
  {
    const std::string topic_name = topic_qt.toStdString();
    const std::string& topic_type = topicTypesByName.at(topic_name);
    topic_selected.insert(topic_name);

    _parser.registerMessageType(topic_name, topic_type);
  }

  if (_config.discard_large_arrays)
  {
    _parser.setMaxArrayPolicy(DISCARD_LARGE_ARRAYS, _config.max_array_size);
  }
  else
  {
    _parser.setMaxArrayPolicy(KEEP_LARGE_ARRAYS, _config.max_array_size);
  }
  _parser.setUseHeaderStamp(_config.use_header_stamp);

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.setRange(0, bag_metadata.message_count);
  progress_dialog.show();
  int msg_count = 0;

  PlotDataAny& plot_consecutive = plot_map.addUserDefined("rosbag2::plotjuggler::consecutive_messages")->second;
  PlotDataAny& metadata_storage = plot_map.addUserDefined("rosbag2::plotjuggler::topics_metadata")->second;
  // dirty trick. Store it in a one point timeseries
  metadata_storage.pushBack( {0, nonstd::any(topics_info) } );

  auto time_prev = std::chrono::high_resolution_clock::now();

  while (_bag_reader->has_next())
  {
    auto msg = _bag_reader->read_next();
    const std::string& topic_name = msg->topic_name;
    double timestamp = 1e-9 * double(msg->time_stamp);  // nanoseconds to seconds

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

    //---- save msg reference in PlotAny ----
    auto data_point = PlotDataAny::Point(timestamp, nonstd::any(msg));
    plot_consecutive.pushBack(data_point);

    auto plot_pair = plot_map.user_defined.find(topic_name);
    if (plot_pair == plot_map.user_defined.end())
    {
      plot_pair = plot_map.addUserDefined(topic_name);
    }
    PlotDataAny& plot_raw = plot_pair->second;
    plot_raw.pushBack(data_point);

    //----- skip not selected -----------
    if (topic_selected.find(topic_name) == topic_selected.end())
    {
      continue;
    }
    //----- parse! -----------
    _parser.parseMessage(topic_name, msg->serialized_data.get(), timestamp);
  }

  // move data from _temp_plot_map to plot_map
  MoveData(_temp_plot_map, plot_map);

  auto now = std::chrono::high_resolution_clock::now();
  double diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_prev).count();

  qDebug() << "The rosbag loaded the data in " << diff << " milliseconds";

  info->selected_datasources = _config.selected_topics;
  return true;
}

bool DataLoadROS2::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  QDomElement stamp_elem = doc.createElement("use_header_stamp");
  stamp_elem.setAttribute("value", _config.use_header_stamp ? "true" : "false");
  plugin_elem.appendChild(stamp_elem);

  QDomElement discard_elem = doc.createElement("discard_large_arrays");
  discard_elem.setAttribute("value", _config.discard_large_arrays ? "true" : "false");
  plugin_elem.appendChild(discard_elem);

  QDomElement max_elem = doc.createElement("max_array_size");
  max_elem.setAttribute("value", QString::number(_config.max_array_size));
  plugin_elem.appendChild(max_elem);

  return true;
}

bool DataLoadROS2::xmlLoadState(const QDomElement& parent_element)
{
  QDomElement stamp_elem = parent_element.firstChildElement("use_header_stamp");
  _config.use_header_stamp = (stamp_elem.attribute("value") == "true");

  QDomElement discard_elem = parent_element.firstChildElement("discard_large_arrays");
  _config.discard_large_arrays = (discard_elem.attribute("value") == "true");

  QDomElement max_elem = parent_element.firstChildElement("max_array_size");
  _config.max_array_size = static_cast<size_t>(max_elem.attribute("value").toInt());

  return true;
}

void DataLoadROS2::saveDefaultSettings()
{
  QSettings settings;
  settings.setValue("DataLoadROS2/default_topics", _config.selected_topics);
  settings.setValue("DataLoadROS2/use_header_stamp", _config.use_header_stamp);
  settings.setValue("DataLoadROS2/max_array_size", (int)_config.max_array_size);
  settings.setValue("DataLoadROS2/discard_large_arrays", _config.discard_large_arrays);
}

void DataLoadROS2::loadDefaultSettings()
{
  QSettings settings;
  _config.selected_topics = settings.value("DataLoadROS2/default_topics", false).toStringList();
  _config.use_header_stamp = settings.value("DataLoadROS2/use_header_stamp", false).toBool();
  _config.max_array_size = settings.value("DataLoadROS2/max_array_size", 100).toInt();
  _config.discard_large_arrays = settings.value("DataLoadROS2/discard_large_arrays", true).toBool();
}
