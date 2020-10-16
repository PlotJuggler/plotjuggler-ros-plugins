#include "logs_table_model.hpp"
#include <QDateTime>
#include <QBrush>
#include <QColor>
#include <QDebug>

LogsTableModel::LogsTableModel(QObject* parent) : QAbstractTableModel(parent), _logs(MAX_CAPACITY)  // initial capacity
{
  _count = 0;
}

QVariant LogsTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (role != Qt::DisplayRole)
    return QVariant();

  if (orientation == Qt::Horizontal)
  {
    switch (section)
    {
      case 0:
        return "#";
        break;
      case 1:
        return "Time";
        break;
      case 2:
        return "Severity";
        break;
      case 3:
        return "Node";
        break;
      case 4:
        return "Message";
        break;
      case 5:
        return "Source";
        break;
    }
  }
  else
  {
    return QString("%1").arg(section);
  }

  return QVariant();
}

int LogsTableModel::rowCount(const QModelIndex& parent) const
{
  if (parent.isValid())
    return 0;

  return _logs.size();
}

int LogsTableModel::columnCount(const QModelIndex& parent) const
{
  if (parent.isValid())
    return 0;

  return 5;
}

QVariant LogsTableModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
    return QVariant();

  if (index.row() >= _logs.size())
    return QVariant();

  const LogItem& log = _logs[index.row()];

  if (role == Qt::DisplayRole)
  {
    switch (index.column())
    {
      case 0:
        return (int)log.count;
      case 1:
        return log.time_text;
      case 2: {
        switch (log.level_raw)
        {
          case DEBUG:
            return "DEBUG";
          case INFO:
            return "INFO";
          case WARNINGS:
            return "WARNINGS";
          case ERROR:
            return "ERROR";
        }
      }
      break;
      case 3:
        return (*log.node);
      case 4:
        return log.message;
      case 5:
        return (*log.source);
    }
  }
  else if (role == Qt::ForegroundRole)
  {
    switch (log.level_raw)
    {
      case DEBUG:
        return QBrush(QColor::fromRgb(50, 50, 50));  // black
      case INFO:
        return QBrush(QColor::fromRgb(0, 0, 255));  // blue
      case WARNINGS:
        return QBrush(QColor::fromRgb(240, 120, 0));  // orange
      case ERROR:
        return QBrush(QColor::fromRgb(255, 0, 0));  // red
    }
  }
  else if (role == Qt::UserRole)
  {
    switch (index.column())
    {
      case 0:
        return (int)log.count;
      case 1: {
        auto usec = (long long)log.time_usec_since_epoch;
        return QVariant(usec);
      }
      case 2:
        return log.level_raw;
      case 3:
        return *log.node;
      case 4:
        return log.message;
      case 5:
        return *log.source;
    }
  }
  else
  {
    return QVariant();
  }
  return QVariant();
}

LogsTableModel::LogItem LogsTableModel::convertRosout(const rosgraph_msgs::Log& log)
{
  _count++;

  LogItem item;
  switch (log.level)
  {
    case rosgraph_msgs::Log::DEBUG:
      item.level_raw = DEBUG;
      break;
    case rosgraph_msgs::Log::INFO:
      item.level_raw = INFO;
      break;
    case rosgraph_msgs::Log::WARN:
      item.level_raw = WARNINGS;
      break;
    case rosgraph_msgs::Log::ERROR:
      item.level_raw = ERROR;
      break;
  }

  item.count = _count;

  QString node_name = QString::fromStdString(log.name);
  auto node_it = _node_list.find(node_name);

  if (node_it == _node_list.end())
  {
    auto inserted_ret = _node_list.insert(node_name);
    node_it = inserted_ret.first;
  }
  item.node = &(*node_it);

  QString source_name(log.file.c_str());
  source_name += (" ");
  source_name += QString::fromStdString(log.function);
  source_name += (":");
  source_name += QString::number(log.line);

  auto source_it = _source_list.find(source_name);

  if (source_it == _source_list.end())
  {
    auto inserted_ret = _source_list.insert(source_name);
    source_it = inserted_ret.first;
  }
  item.source = &(*source_it);

  item.message = log.msg.c_str();

  item.time_usec_since_epoch = log.header.stamp.toNSec() / 1000;
  item.time_text = QDateTime::fromMSecsSinceEpoch(item.time_usec_since_epoch / 1000).toString("d/M/yy HH:mm::ss.zzz");
  return item;
}

void LogsTableModel::push_back(const rosgraph_msgs::Log::ConstPtr& pushed_log)
{
  bool to_shift = (_logs.size() == MAX_CAPACITY);

  _logs.push_back(convertRosout(*pushed_log));

  if (to_shift)
  {
    this->beginRemoveRows(QModelIndex(), 0, 0);
    this->endRemoveRows();
    emit dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
  }

  this->beginInsertRows(QModelIndex(), _logs.size() - 1, _logs.size() - 1);
  this->endInsertRows();
}

void LogsTableModel::push_back(const std::vector<rosgraph_msgs::Log::ConstPtr>& pushed_logs)
{
  size_t old_size = _logs.size();
  size_t new_size = old_size + pushed_logs.size();

  int to_add = pushed_logs.size();
  int to_shift = 0;

  if (new_size > MAX_CAPACITY)
  {
    to_add = (MAX_CAPACITY - old_size);
    to_shift = new_size - MAX_CAPACITY;
    new_size = MAX_CAPACITY;
  }

  const size_t last_row = new_size - 1;
  const size_t first_row = new_size - pushed_logs.size();

  for (int i = 0; i < pushed_logs.size(); i++)
  {
    _logs.push_back(convertRosout(*pushed_logs[i]));
  }

  std::sort(_logs.begin(), _logs.end(),
            [](const LogItem& a, const LogItem& b) { return a.time_usec_since_epoch < b.time_usec_since_epoch; });

  if (to_shift > 0)
  {
    this->beginRemoveRows(QModelIndex(), 0, 0);
    this->endRemoveRows();

    emit dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
  }

  this->beginInsertRows(QModelIndex(), first_row, last_row);
  this->endInsertRows();
}

const QString& LogsTableModel::message(int index) const
{
  return _logs[index].message;
}

const QString& LogsTableModel::nodeName(int index) const
{
  return *(_logs[index].node);
}

LogsTableModel::Severity LogsTableModel::severity(int index) const
{
  return _logs[index].level_raw;
}

TimePoint LogsTableModel::timestamp(int index) const
{
  std::chrono::microseconds since_epoch(_logs[index].time_usec_since_epoch);
  return TimePoint() + since_epoch;
}

void LogsTableModel::clear()
{
  this->beginRemoveRows(QModelIndex(), 0, _logs.size() - 1);
  this->endRemoveRows();
  _logs.clear();
  _count = 0;
}
