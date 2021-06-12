#include "rosout_publisher.h"
#include "shape_shifter_factory.hpp"
#include "qnodedialog.h"
#include <QSettings>
#include <rosbag/bag.h>

RosoutPublisher::RosoutPublisher() : _enabled(false), _tablemodel(nullptr), _log_window(nullptr)
{
}

RosoutPublisher::~RosoutPublisher()
{
}

void RosoutPublisher::setEnabled(bool to_enable)
{
  _enabled = to_enable;

  if (enabled())
  {
    _minimum_time_usec = std::numeric_limits<int64_t>::max();
    _maximum_time_usec = std::numeric_limits<int64_t>::min();

    if (_tablemodel == nullptr)
    {
      _tablemodel = new LogsTableModel(this);
    }

    _log_window = new RosoutWindow();

    auto logwidget = new rqt_console_plus::LogWidget(*_tablemodel, _log_window);
    _log_window->setCentralWidget(logwidget);
    Qt::WindowFlags flags = _log_window->windowFlags();
    _log_window->setWindowFlags(flags | Qt::SubWindow);

    connect(this, &RosoutPublisher::timeRangeChanged, logwidget, &rqt_console_plus::LogWidget::on_timeRangeChanged);

    connect(_log_window, &RosoutWindow::closed, this, &RosoutPublisher::onWindowClosed);

    QSettings settings;
    _log_window->restoreGeometry(settings.value("RosoutPublisher.geometry").toByteArray());

    _log_window->show();
  }
  else
  {
    if (_log_window)
    {
      _log_window->close();
    }
  }
}

void RosoutPublisher::onWindowClosed()
{
  QSettings settings;
  settings.setValue("RosoutPublisher.geometry", _log_window->saveGeometry());

  if (_tablemodel)
  {
    _tablemodel->deleteLater();
    _tablemodel = nullptr;
  }
  if (_log_window)
  {
    _log_window->deleteLater();
    _log_window = nullptr;
  }
  _enabled = false;

  emit closed();
}

std::vector<const PlotDataAny*> RosoutPublisher::findRosoutTimeseries()
{
  std::vector<const PlotDataAny*> logs_timeseries;

  for (const auto& data_it : _datamap->user_defined)
  {
    const std::string& topic_name = data_it.first;

    // check if I registered this message before
    const RosIntrospection::ShapeShifter* registered_shapeshifted_msg =
        RosIntrospectionFactory::get().getShapeShifter(topic_name);
    if (!registered_shapeshifted_msg)
    {
      continue;  // will not be able to use this anyway, just skip
    }

    if (registered_shapeshifted_msg->getMD5Sum() !=
        std::string(ros::message_traits::MD5Sum<rosgraph_msgs::Log>::value()))
    {
      continue;  // it is NOT a rosgraph_msgs::Log
    }

    logs_timeseries.push_back(&data_it.second);
  }

  return logs_timeseries;
}

void RosoutPublisher::syncWithTableModel(const std::vector<const PlotDataAny*>& logs_timeseries)
{
  const int64_t threshold_time = _maximum_time_usec;

  std::vector<rosgraph_msgs::LogConstPtr> logs;
  logs.reserve(100);

  // most of the time we expect logs_timeseries to have just 1 element
  for (const PlotDataAny* type_erased_logs : logs_timeseries)
  {
    const int first_index = type_erased_logs->getIndexFromX(threshold_time);

    if (first_index != -1)
    {
      for (int i = first_index; i < type_erased_logs->size(); i++)
      {
        const auto& any_msg = type_erased_logs->at(i);
        const std::any& any_value = any_msg.y;

        const bool isRawBuffer = any_value.type() == typeid(std::vector<uint8_t>);
        const bool isRosbagMessage = any_value.type() == typeid(rosbag::MessageInstance);
        std::vector<uint8_t> raw_buffer;

        if (isRawBuffer)
        {
          raw_buffer = std::any_cast<std::vector<uint8_t>>(any_value);
        }
        else if (isRosbagMessage)
        {
          const rosbag::MessageInstance& msg_instance = std::any_cast<rosbag::MessageInstance>(any_value);
          raw_buffer.resize(msg_instance.size());
          ros::serialization::OStream stream(raw_buffer.data(), raw_buffer.size());
          msg_instance.write(stream);
        }
        else
        {
          continue;
        }

        rosgraph_msgs::LogPtr p(boost::make_shared<rosgraph_msgs::Log>());
        ros::serialization::IStream stream(raw_buffer.data(), raw_buffer.size());
        ros::serialization::deserialize(stream, *p);

        int64_t usec = p->header.stamp.toNSec() / 1000;
        _minimum_time_usec = std::min(_minimum_time_usec, usec);
        _maximum_time_usec = std::max(_maximum_time_usec, usec);

        if (usec >= threshold_time)
        {
          logs.push_back(p);
        }
      }
    }
  }
  std::sort(logs.begin(), logs.end(), [](const rosgraph_msgs::LogConstPtr& a, const rosgraph_msgs::LogConstPtr& b) {
    return a->header.stamp < b->header.stamp;
  });
  _tablemodel->push_back(logs);
}

void RosoutPublisher::updateState(double current_time)
{
  if (!_enabled && !_tablemodel)
    return;

  std::vector<const PlotDataAny*> logs_timeseries = findRosoutTimeseries();

  syncWithTableModel(logs_timeseries);

  using namespace std::chrono;
  TimePoint p_min = TimePoint() + microseconds(_minimum_time_usec);
  // TimePoint p_max  = TimePoint() + microseconds(_maximum_time_usec);
  TimePoint p_curr = TimePoint() + microseconds((int64_t)(current_time * 1000000));

  emit timeRangeChanged(p_min, p_curr);
}
