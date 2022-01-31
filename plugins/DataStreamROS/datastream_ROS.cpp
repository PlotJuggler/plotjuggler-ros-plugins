#include "datastream_ROS.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>
#include <QProgressDialog>
#include <QtGlobal>
#include <QApplication>
#include <QProcess>
#include <QCheckBox>
#include <QSettings>
#include <QFileDialog>
#include <ros/callback_queue.h>
#include <rosbag/bag.h>
#include <ros_type_introspection/utils/shape_shifter.hpp>
#include <ros/transport_hints.h>

#include "dialog_select_ros_topics.h"
#include "rule_editing.h"
#include "qnodedialog.h"
#include "shape_shifter_factory.hpp"

DataStreamROS::DataStreamROS() : DataStreamer(), _node(nullptr)
, _action_saveIntoRosbag(nullptr)
, _prev_clock_time(0)
{
  _running = false;
  _first_warning = true;
  _periodic_timer = new QTimer();
  connect(_periodic_timer, &QTimer::timeout, this, &DataStreamROS::timerCallback);

  {
    QSettings settings;
    _config.loadFromSettings(settings, "DataStreamROS");
  }

  _action_saveIntoRosbag = new QAction(QString("Save cached value in a rosbag"));
  connect(_action_saveIntoRosbag, &QAction::changed, this, [this]() {
    DataStreamROS::saveIntoRosbag();
  });

  _action_saveAny = new QAction(QString("Store messages in memory for Re-publishing"));
  connect(_action_saveAny, &QAction::triggered, this, [this]() {
    QSettings settings;
    settings.value("DataStreamROS/storemessagesInMemory", _action_saveAny->isChecked() );
  });
  _action_saveAny->setCheckable( true );

  QSettings settings;
  bool store_msg = settings.value("DataStreamROS/storemessagesInMemory", false).toBool();
  _action_saveAny->setChecked( store_msg );

  _available_actions.push_back( _action_saveIntoRosbag );
  _available_actions.push_back( _action_saveAny );
}

void DataStreamROS::topicCallback(const RosIntrospection::ShapeShifter::ConstPtr& msg, const std::string& topic_name)
{
  if (!_running)
  {
    return;
  }

  emit dataReceived();

  using namespace RosIntrospection;
  const auto& md5sum = msg->getMD5Sum();
  const auto& datatype = msg->getDataType();
  const auto& definition = msg->getMessageDefinition();

  // register the message type
  _parser->registerMessageType(topic_name, datatype, definition);

  RosIntrospectionFactory::registerMessage(topic_name, md5sum, datatype, definition);

  //------------------------------------
  std::vector<uint8_t> buffer;

  buffer.resize(msg->size());

  ros::serialization::OStream stream(buffer.data(), buffer.size());
  msg->write(stream);

  double msg_time = ros::Time::now().toSec();
  if (msg_time == 0)
  {
    // corner case: use_sim_time == true but topic /clock is not published
    msg_time = ros::WallTime::now().toSec();

    auto tmp_config = _config;
    tmp_config.use_header_stamp = false;
    _parser->setConfig(tmp_config);
  }

  // time wrapping may happen using use_sim_time = true and
  // rosbag play --clock --loop
  if (msg_time < _prev_clock_time)
  {
    // clear
    for (auto& it : dataMap().numeric)
    {
      it.second.clear();
    }
    for (auto& it : _user_defined_buffers)
    {
      it.second.clear();
    }
    emit clearBuffers();
  }
  _prev_clock_time = msg_time;

  MessageRef buffer_view(buffer);

  try
  {
      // before pushing, lock the mutex
      std::lock_guard<std::mutex> lock(mutex());
      _parser->parseMessage(topic_name, buffer_view, msg_time);
  }
  catch (std::runtime_error& ex)
  {
      if( _first_warning ) {
          _first_warning = false;
          QMessageBox::warning(nullptr, tr("Error"),
                               QString("rosbag::open thrown an exception:\n") +
                               QString(ex.what()) +
                               "\nThis message will be shown only once.");
      }
  }

  const std::string prefixed_topic_name = _prefix + topic_name;

  bool save_any = _action_saveAny->isChecked();
  // adding raw serialized msg for future uses.
  // do this before msg_time normalization
  if( save_any )
  {
    auto plot_pair = _user_defined_buffers.find(prefixed_topic_name);
    if (plot_pair == _user_defined_buffers.end())
    {
      plot_pair = _user_defined_buffers.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(prefixed_topic_name),
            std::forward_as_tuple(prefixed_topic_name, PlotGroup::Ptr())).first;
    }
    PlotDataAny& user_defined_data = plot_pair->second;
    user_defined_data.pushBack(PlotDataAny::Point(msg_time, std::any(std::move(buffer))));
  }

  //------------------------------
  if( save_any )
  {
    int& index = _msg_index[topic_name];
    index++;
    const std::string key = prefixed_topic_name + ("/_MSG_INDEX_");
    auto index_it = dataMap().numeric.find(key);
    if (index_it == dataMap().numeric.end())
    {
      index_it = dataMap().addNumeric(key);
    }
    index_it->second.pushBack(PlotData::Point(msg_time, index));
  }
}

void DataStreamROS::extractInitialSamples()
{
  using namespace std::chrono;
  milliseconds wait_time_ms(1000);

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Collecting ROS topic samples to understand data layout. ");
  progress_dialog.setRange(0, wait_time_ms.count());
  progress_dialog.setAutoClose(true);
  progress_dialog.setAutoReset(true);

  progress_dialog.show();

  auto start_time = system_clock::now();

  while (system_clock::now() - start_time < (wait_time_ms))
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    int i = duration_cast<milliseconds>(system_clock::now() - start_time).count();
    progress_dialog.setValue(i);
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

void DataStreamROS::timerCallback()
{
  if (_running && ros::master::check() == false)
  {
    auto ret = QMessageBox::warning(nullptr, tr("Disconnected!"),
                                    tr("The roscore master is not reachable anymore.\n\n"
                                       "Do you want to try reconnecting to it? \n"),
                                    tr("Stop Streaming"), tr("Try reconnect"), nullptr);
    if (ret == 1)
    {
      this->shutdown();
      _node = RosManager::getNode();

      if (!_node)
      {
        emit closed();
        return;
      }
      _parser.reset( new RosCompositeParser(dataMap()) );
      subscribe();

      _running = true;
      _spinner = std::make_shared<ros::AsyncSpinner>(1);
      _spinner->start();
      _periodic_timer->start();
    }
    else if (ret == 0)
    {
      this->shutdown();
      emit closed();
    }
  }

  if( !ros::ok() )
  {
    QMessageBox::warning(nullptr, tr("ROS Stopped"), "The plugin will be stopped");
    this->shutdown();

    emit closed();
  }
}

void DataStreamROS::saveIntoRosbag()
{
  if (_user_defined_buffers.empty())
  {
    QMessageBox::warning(nullptr, tr("Warning"), tr("Your buffer is empty. Nothing to save.\n"));
    return;
  }

  QFileDialog saveDialog;
  saveDialog.setAcceptMode(QFileDialog::AcceptSave);
  saveDialog.setDefaultSuffix("bag");
  saveDialog.exec();

  if (saveDialog.result() != QDialog::Accepted || saveDialog.selectedFiles().empty())
  {
    return;
  }

  QString fileName = saveDialog.selectedFiles().first();

  if (fileName.size() > 0)
  {
    rosbag::Bag rosbag(fileName.toStdString(), rosbag::bagmode::Write);

    for (const auto& it : _user_defined_buffers)
    {
      const std::string& topicname = it.first;
      const auto& plotdata = it.second;

      auto registered_msg_type = RosIntrospectionFactory::get().getShapeShifter(topicname);
      if (!registered_msg_type){
        continue;
      }

      RosIntrospection::ShapeShifter msg;
      msg.morph(registered_msg_type->getMD5Sum(), registered_msg_type->getDataType(),
                registered_msg_type->getMessageDefinition());

      for (int i = 0; i < plotdata.size(); i++)
      {
        const auto& point = plotdata.at(i);
        const double msg_time = point.x;
        const std::any& type_erased_buffer = point.y;

        if (type_erased_buffer.type() != typeid(std::vector<uint8_t>))
        {
          // can't cast to expected type
          continue;
        }

        std::vector<uint8_t> raw_buffer = std::any_cast<std::vector<uint8_t>>(type_erased_buffer);
        ros::serialization::IStream stream(raw_buffer.data(), raw_buffer.size());
        msg.read(stream);

        rosbag.write(topicname, ros::Time(msg_time), msg);
      }
    }
    rosbag.close();

    QProcess process;
    QStringList args;
    args << "reindex" << fileName;
    process.start("rosbag", args);
  }
}

void DataStreamROS::subscribe()
{
  _subscribers.clear();

  for (int i = 0; i < _config.topics.size(); i++)
  {
    const std::string topic_name = _config.topics[i].toStdString();
    boost::function<void(const RosIntrospection::ShapeShifter::ConstPtr&)> callback;
    callback = [this, topic_name](const RosIntrospection::ShapeShifter::ConstPtr& msg) -> void {
      this->topicCallback(msg, topic_name);
    };

    ros::SubscribeOptions ops;
    ops.initByFullCallbackType(topic_name, 1, callback);
    ops.transport_hints = ros::TransportHints().tcpNoDelay();

    _subscribers.insert({ topic_name, _node->subscribe(ops) });
  }
}

bool DataStreamROS::start(QStringList* selected_datasources)
{
  if (!_node)
  {
    _node = RosManager::getNode();
  }

  if (!_node)
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex());
    dataMap().numeric.clear();
    dataMap().user_defined.clear();
    _parser.reset( new RosCompositeParser(dataMap()) );
  }

  using namespace RosIntrospection;

  std::vector<std::pair<QString, QString>> all_topics;
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  for (ros::master::TopicInfo topic_info : topic_infos)
  {
    all_topics.push_back(std::make_pair(QString(topic_info.name.c_str()), QString(topic_info.datatype.c_str())));
  }

  QTimer timer;
  timer.setSingleShot(false);
  timer.setInterval(1000);
  timer.start();

  DialogSelectRosTopics dialog(all_topics, _config);

  connect(&timer, &QTimer::timeout, [&]() {
    all_topics.clear();
    topic_infos.clear();
    ros::master::getTopics(topic_infos);
    for (ros::master::TopicInfo topic_info : topic_infos)
    {
      all_topics.push_back({ QString::fromStdString(topic_info.name), QString::fromStdString(topic_info.datatype) });
    }
    dialog.updateTopicList(all_topics);
  });

  int res = dialog.exec();
  _config = dialog.getResult();
  timer.stop();

  if (res != QDialog::Accepted || _config.topics.empty())
  {
    return false;
  }

  {
    QSettings settings;
    _config.saveToSettings(settings, "DataStreamROS");
  }

  _parser->setConfig(_config);

  //-------------------------
  subscribe();
  _running = true;
  _first_warning = true;

  extractInitialSamples();

  _spinner = std::make_shared<ros::AsyncSpinner>(1);
  _spinner->start();

  _periodic_timer->setInterval(500);
  _periodic_timer->start();

  return true;
}

bool DataStreamROS::isRunning() const
{
  return _running;
}

void DataStreamROS::shutdown()
{
  _periodic_timer->stop();

  if (_spinner)
  {
    _spinner->stop();
  }
  _spinner.reset();

  if(_node ){
    _node->shutdown();
  }
  _node.reset();

  _subscribers.clear();
  _running = false;
  _parser.reset();
}

DataStreamROS::~DataStreamROS()
{
  shutdown();
}

bool DataStreamROS::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  _config.xmlSaveState(doc, plugin_elem);
  return true;
}

bool DataStreamROS::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

const std::vector<QAction *> &DataStreamROS::availableActions()
{
  return _available_actions;
}

