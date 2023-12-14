#ifndef LOGSTABLEMODEL_HPP
#define LOGSTABLEMODEL_HPP

#include <QAbstractTableModel>
#include <QString>
#include <QDateTime>
#include <rosgraph_msgs/Log.h>
#include <rosbag/view.h>
#include <boost/circular_buffer.hpp>
#include <unordered_map>
#include <chrono>

#ifdef _WIN32
#undef ERROR
#endif

typedef std::chrono::high_resolution_clock::time_point TimePoint;


class LogsTableModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  explicit LogsTableModel(QObject *parent = 0);

  typedef enum{
    DEBUG = 0,
    INFO = 1,
    WARNINGS = 2,
    ERROR = 3
  }Severity;

  // Header:
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  // Basic functionality:
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;

  int columnCount(const QModelIndex &parent = QModelIndex()) const override;

  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

  void push_back(const rosgraph_msgs::Log::ConstPtr& pushed_log);

  void push_back(const std::vector<rosgraph_msgs::Log::ConstPtr>& pushed_logs);

  const QString& message(int index) const;

  const QString &nodeName(int index) const;

  Severity severity(int index) const;

  TimePoint timestamp(int index) const;

  int size() const { return _logs.size(); }

  void clear();

private:

  std::set<QString> _source_list;
  std::set<QString> _node_list;

  typedef struct{
    size_t count;
    int64_t time_usec_since_epoch;
    QString  time_text;
    Severity level_raw;
    const QString* node;
    QString message;
    const QString* source;
  }LogItem;

  boost::circular_buffer<LogItem> _logs;

  size_t _count;

  enum{ MAX_CAPACITY = 20000 }; // max capacity of the circular buffer

  LogItem convertRosout(const rosgraph_msgs::Log &log);

#ifdef USE_ROSOUT2
  LogItem convertRosout(const rosout2_msg::LogMsg &log);
#endif

signals:

  void rowsShifted(int);

};


#endif // LOGSTABLEMODEL_HPP
