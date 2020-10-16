#ifndef MODELFILTER_HPP
#define MODELFILTER_HPP

#include <chrono>
#include <QSortFilterProxyModel>
#include <QDateTime>
#include <QString>
#include <QRegExpValidator>

typedef std::chrono::high_resolution_clock::time_point TimePoint;

class ModelFilter : public QSortFilterProxyModel
{
  Q_OBJECT
public:
  explicit ModelFilter(QObject *parent = 0);

  typedef enum{
    CONTAINS_ONE = 0,
    WILDCARDS = 1,
    REGEX = 2
  }FilterMode;

signals:

public slots:

  void setMessageFilterEnabled(bool enabled);
  void setNodeFilterEnabled(bool enabled);
  void setSourceFilterEnabled(bool enabled);
  void setTimeFilterEnabled(bool enabled);

  void messageFilterUpdated(FilterMode mode, const QString& text  );
  void nodeFilterUpdated(FilterMode mode, const QString& text  );
  void sourceFilterUpdated(FilterMode mode, const QString& text  );
  void timeMinMaxUpdated(TimePoint min, TimePoint max);

  void setSeverityInfoEnabled(bool enabled);
  void setSeverityDebugEnabled(bool enabled);
  void setSeverityErrorEnabled(bool enabled);
  void setSeverityWarningsEnabled(bool enabled);

private:
  bool applyFilter(const QString &filter, ModelFilter::FilterMode mode,
                   const QString& text_to_parse, const QRegExpValidator *validator) const;

protected:
  virtual bool filterAcceptsRow(int sourceRow,
                                const QModelIndex &sourceParent) const override;

  TimePoint _min;
  TimePoint _max;

  bool _node_filter_enabled;
  bool _source_filter_enabled;
  bool _msg_filter_enabled;
  bool _time_filter_enabled;

  bool _debug_filter_enabled;
  bool _info_filter_enabled;
  bool _error_filter_enabled;
  bool _warn_filter_enabled;

  FilterMode _node_mode;
  FilterMode _msg_mode;
  FilterMode _source_mode;

  QString _node_text;
  QString _msg_text;
  QString _source_text;

  QRegExpValidator _node_validator;
  QRegExpValidator _msg_validator;
  QRegExpValidator _source_validator;
};

#endif // MODELFILTER_HPP
