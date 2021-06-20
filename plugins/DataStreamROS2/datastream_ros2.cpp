#include "datastream_ros2.h"

#include <QDebug>
#include <QTimer>
#include <QSettings>
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>
#include "rclcpp/generic_subscription.hpp"

DataStreamROS2::DataStreamROS2() :
    DataStreamer(),
    _node(nullptr),
    _running(false),
    _first_warning(false),
    _config()
{
  loadDefaultSettings();

  _context = std::make_shared<rclcpp::Context>();
  _context->init(0, nullptr);

  auto exec_args = rclcpp::ExecutorOptions();
  exec_args.context = _context;
  _executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(exec_args, 2);

}

void DataStreamROS2::waitOneSecond()
{
  using namespace std::chrono;
  milliseconds wait_time_ms(1000);

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Collecting ROS topic samples to understand data layout.");
  progress_dialog.setRange(0, wait_time_ms.count());
  progress_dialog.setAutoClose(true);
  progress_dialog.setAutoReset(true);
  progress_dialog.show();

  auto start_time = system_clock::now();

  while (system_clock::now() - start_time < (wait_time_ms))
  {
    int msec = duration_cast<milliseconds>(system_clock::now() - start_time).count();
    progress_dialog.setValue(msec);
    QApplication::processEvents();
    if (progress_dialog.wasCanceled())
    {
      break;
    }
  }

  if (progress_dialog.wasCanceled() == false)
  {
    progress_dialog.cancel();
  }
}

bool DataStreamROS2::start(QStringList* selected_datasources)
{
  if (!_node)
  {
    auto node_opts = rclcpp::NodeOptions();
    node_opts.context(_context);
    _node = std::make_shared<rclcpp::Node>("plotjuggler", node_opts);
    _executor->add_node(_node);
  }

  {
    std::lock_guard<std::mutex> lock(mutex());
    dataMap().numeric.clear();
    dataMap().user_defined.clear();
    _parser.reset( new CompositeParser(dataMap()) );
  }

  // Display the dialog which allows users to select ros topics to subscribe to,
  // and continuously update the list of available topics
  // We start with an empty topic list
  std::vector<std::pair<QString, QString>> dialog_topics;
  DialogSelectRosTopics dialog(dialog_topics, _config);

  QTimer update_list_timer;
  update_list_timer.setSingleShot(false);
  update_list_timer.setInterval(1000);
  update_list_timer.start();

  auto getTopicsFromNode = [&]() {
    dialog_topics.clear();
    auto topic_list = _node->get_topic_names_and_types();
    for (const auto& topic : topic_list)
    {
      // TODO: Handle topics with multiple types
      auto topic_name = QString::fromStdString(topic.first);
      auto type_name = QString::fromStdString(topic.second[0]);
      dialog_topics.push_back({ topic_name, type_name });
      dialog.updateTopicList(dialog_topics);
    }
  };

  getTopicsFromNode();

  connect(&update_list_timer, &QTimer::timeout, getTopicsFromNode);

  int res = dialog.exec();
  _config = dialog.getResult();
  update_list_timer.stop();

  // If no topics were selected, or the OK button was not pressed, do nothing
  if (res != QDialog::Accepted || _config.selected_topics.empty())
  {
    return false;
  }

  saveDefaultSettings();
  if (_config.discard_large_arrays)
  {
    _parser->setMaxArrayPolicy(DISCARD_LARGE_ARRAYS, _config.max_array_size);
  }
  else
  {
    _parser->setMaxArrayPolicy(KEEP_LARGE_ARRAYS, _config.max_array_size);
  }
  _parser->setUseHeaderStamp(_config.use_header_stamp);

  //--------- subscribe ---------
  for (const auto& topic : dialog_topics)
  {
    if (_config.selected_topics.contains(topic.first))
    {
      subscribeToTopic(topic.first.toStdString(), topic.second.toStdString());
    }
  }
  //-----------------------------
  _clock = rclcpp::Clock();
  _start_time = _clock.now().nanoseconds();
  _running = true;
  _first_warning = true;

  _spinner = std::thread([this]() {
    while (_running)
    {
      if (_executor)
      {
        _executor->spin_once(std::chrono::milliseconds(5));
      }
    }
  });

  //-----------------------------
  waitOneSecond();
  return true;
}

bool DataStreamROS2::isRunning() const
{
  return _running;
}

void DataStreamROS2::shutdown()
{
  _running = false;
  if (_spinner.joinable())
  {
    _spinner.join();
  }

  _subscriptions.clear();
  if (_node)
  {
    _executor->remove_node(_node);
    _node.reset();
  }
}

DataStreamROS2::~DataStreamROS2()
{
  shutdown();
}

const std::vector<QAction*> &DataStreamROS2::availableActions()
{
  static std::vector<QAction*> empty;
  return empty;
}

void DataStreamROS2::subscribeToTopic(const std::string& topic_name, const std::string& topic_type)
{
  if (_subscriptions.find(topic_name) != _subscriptions.end())
  {
    return;
  }

  _parser->registerMessageType(topic_name, topic_type);

  auto bound_callback = [=](std::shared_ptr<rclcpp::SerializedMessage> msg) { messageCallback(topic_name, msg); };

  // double subscription, latching or not
  for (bool transient : { true, false })
  {
    auto subscription = _node->create_generic_subscription(topic_name,
                                                           topic_type,
                                                           transient,
                                                           bound_callback);

    _subscriptions[topic_name + (transient ? "/transient_" : "")] = subscription;
    _node->get_node_topics_interface()->add_subscription(subscription, nullptr);
  }
}

void DataStreamROS2::messageCallback(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  double timestamp = _node->get_clock()->now().seconds();
  try
  {
      std::unique_lock<std::mutex> lock(mutex());
      _parser->parseMessage(topic_name, &(msg.get()->get_rcl_serialized_message()), timestamp);
  }
  catch (std::runtime_error& ex)
  {
      if( _first_warning ) {
          _first_warning = false;
          QMessageBox::warning(nullptr, tr("Error"),
                               QString("rosbag::open thrown an exception:\n") +
                               QString(ex.what()) + "\nThis message will be shown only once.");
      }
  }

  emit dataReceived();
}

void DataStreamROS2::saveDefaultSettings()
{
  QSettings settings;
  settings.setValue("DataStreamROS2/default_topics", _config.selected_topics);
  settings.setValue("DataStreamROS2/use_header_stamp", _config.use_header_stamp);
  settings.setValue("DataStreamROS2/discard_large_arrays", _config.discard_large_arrays);
  settings.setValue("DataStreamROS2/max_array_size", (int)_config.max_array_size);
}

void DataStreamROS2::loadDefaultSettings()
{
  QSettings settings;
  _config.selected_topics = settings.value("DataStreamROS2/default_topics", false).toStringList();
  _config.use_header_stamp = settings.value("DataStreamROS2/use_header_stamp", false).toBool();
  _config.discard_large_arrays = settings.value("DataStreamROS2/discard_large_arrays", true).toBool();
  _config.max_array_size = settings.value("DataStreamROS2/max_array_size", 100).toInt();
}

bool DataStreamROS2::xmlLoadState(const QDomElement& parent_element)
{
  qDebug() << "DataStreamROS2::xmlLoadState";

  QDomElement stamp_elem = parent_element.firstChildElement("use_header_stamp");
  _config.use_header_stamp = (stamp_elem.attribute("value") == "true");

  QDomElement discard_elem = parent_element.firstChildElement("discard_large_arrays");
  _config.discard_large_arrays = (stamp_elem.attribute("value") == "true");

  QDomElement max_elem = parent_element.firstChildElement("max_array_size");
  _config.max_array_size = (stamp_elem.attribute("value") == "true");

  _config.selected_topics.clear();
  QDomElement topic_elem = parent_element.firstChildElement("selected_topics").firstChildElement("topic");
  while (!topic_elem.isNull())
  {
    qDebug() << "Value: " << topic_elem.attribute("value");
    _config.selected_topics.push_back(topic_elem.attribute("name"));
    topic_elem = topic_elem.nextSiblingElement("topic");
  }
  return true;
}

bool DataStreamROS2::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  QDomElement stamp_elem = doc.createElement("use_header_stamp");
  stamp_elem.setAttribute("value", _config.use_header_stamp ? "true" : "false");
  plugin_elem.appendChild(stamp_elem);

  // TODO: Implement discarding large arrays
  QDomElement discard_elem = doc.createElement("discard_large_arrays");
  discard_elem.setAttribute("value", _config.discard_large_arrays ? "true" : "false");
  plugin_elem.appendChild(discard_elem);

  QDomElement max_elem = doc.createElement("max_array_size");
  max_elem.setAttribute("value", _config.max_array_size ? "true" : "false");
  plugin_elem.appendChild(max_elem);

  QDomElement topics_elem = doc.createElement("selected_topics");
  for (auto topic : _config.selected_topics)
  {
    QDomElement topic_elem = doc.createElement("topic");
    topic_elem.setAttribute("name", topic);
    topics_elem.appendChild(topic_elem);
  }
  plugin_elem.appendChild(topics_elem);

  return true;
}
