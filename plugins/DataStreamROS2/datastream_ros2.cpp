#include "datastream_ros2.h"

#include <QDebug>
#include <QTimer>
#include <QSettings>
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>
#include "ros2_parsers/generic_subscription.hpp"
#include "rosbag2_helper.hpp"

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

  auto exec_args = rclcpp::executor::ExecutorArgs();
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
    _parser.reset( new Ros2CompositeParser(dataMap()) );
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
  if (res != QDialog::Accepted || _config.topics.empty())
  {
    return false;
  }

  saveDefaultSettings();
  _parser->setConfig(_config);

  //--------- subscribe ---------
  for (const auto& topic : dialog_topics)
  {
    if (_config.topics.contains(topic.first))
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

  auto publisher_info = _node->get_publishers_info_by_topic(topic_name);
  auto detected_qos = PJ::adapt_request_to_offers(topic_name, publisher_info);

  // double subscription, latching or not
  auto subscription = std::make_shared<rosbag2_transport::GenericSubscription>(
      _node->get_node_base_interface().get(),
      *_parser->typeSupport(topic_name),
      topic_name, detected_qos, bound_callback);
  _subscriptions[topic_name] = subscription;
  _node->get_node_topics_interface()->add_subscription(subscription, nullptr);

}

void DataStreamROS2::messageCallback(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  double timestamp = _node->get_clock()->now().seconds();
  try
  {
      std::unique_lock<std::mutex> lock(mutex());

      auto msg_ptr = msg.get()->get_rcl_serialized_message();
      MessageRef msg_ref( msg_ptr.buffer, msg_ptr.buffer_length );

      _parser->parseMessage(topic_name, msg_ref, timestamp);
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
  _config.saveToSettings(settings, "DataStreamROS2");
}

void DataStreamROS2::loadDefaultSettings()
{
  QSettings settings;
  _config.loadFromSettings(settings, "DataStreamROS2");
}

bool DataStreamROS2::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

bool DataStreamROS2::xmlSaveState(QDomDocument& doc,
                                  QDomElement& parent_element) const
{
  _config.xmlSaveState(doc, parent_element);
  return true;
}
