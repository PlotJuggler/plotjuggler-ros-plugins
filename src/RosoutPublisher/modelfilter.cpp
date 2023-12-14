#include "modelfilter.hpp"
#include "logs_table_model.hpp"

ModelFilter::ModelFilter(QObject* parent) : QSortFilterProxyModel(parent)
{
  _msg_filter_enabled = false;
  _node_filter_enabled = false;
  _source_filter_enabled = false;
  _time_filter_enabled = true;

  _info_filter_enabled = false;
  _error_filter_enabled = false;
  _warn_filter_enabled = false;
  _debug_filter_enabled = false;
}

void ModelFilter::setMessageFilterEnabled(bool enabled)
{
  _msg_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setNodeFilterEnabled(bool enabled)
{
  _node_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setSourceFilterEnabled(bool enabled)
{
  _source_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setTimeFilterEnabled(bool enabled)
{
  _time_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::messageFilterUpdated(ModelFilter::FilterMode mode, const QString& filter)
{
  _msg_mode = mode;
  _msg_text = filter;

  if (mode == WILDCARDS)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::Wildcard);
    _msg_validator.setRegExp(regexp);
  }
  else if (mode == REGEX)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::RegExp2);
    _msg_validator.setRegExp(regexp);
  }

  invalidateFilter();
}

void ModelFilter::nodeFilterUpdated(ModelFilter::FilterMode mode, const QString& filter)
{
  _node_mode = mode;
  _node_text = filter;

  if (mode == WILDCARDS)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::Wildcard);
    _node_validator.setRegExp(regexp);
  }
  else if (mode == REGEX)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::RegExp2);
    _node_validator.setRegExp(regexp);
  }
  invalidateFilter();
}

void ModelFilter::sourceFilterUpdated(ModelFilter::FilterMode mode, const QString& filter)
{
  _source_mode = mode;
  _source_text = filter;

  if (mode == WILDCARDS)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::Wildcard);
    _source_validator.setRegExp(regexp);
  }
  else if (mode == REGEX)
  {
    QRegExp regexp(filter, Qt::CaseSensitive, QRegExp::RegExp2);
    _source_validator.setRegExp(regexp);
  }
  invalidateFilter();
}

void ModelFilter::timeMinMaxUpdated(TimePoint min, TimePoint max)
{
  _min = min;
  _max = max;
  invalidateFilter();
}

void ModelFilter::setSeverityInfoEnabled(bool enabled)
{
  _info_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setSeverityDebugEnabled(bool enabled)
{
  _debug_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setSeverityErrorEnabled(bool enabled)
{
  _error_filter_enabled = enabled;
  invalidateFilter();
}

void ModelFilter::setSeverityWarningsEnabled(bool enabled)
{
  _warn_filter_enabled = enabled;
  invalidateFilter();
}

bool ModelFilter::filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const
{
  QModelIndex index_time = sourceModel()->index(sourceRow, 1, sourceParent);
  QModelIndex index_severity = sourceModel()->index(sourceRow, 2, sourceParent);
  QModelIndex index_node = sourceModel()->index(sourceRow, 3, sourceParent);
  QModelIndex index_message = sourceModel()->index(sourceRow, 4, sourceParent);
  QModelIndex index_source = sourceModel()->index(sourceRow, 5, sourceParent);

  int severity = sourceModel()->data(index_severity, Qt::UserRole).toInt();

  if (!_info_filter_enabled && severity == LogsTableModel::INFO)
    return false;
  if (!_error_filter_enabled && severity == LogsTableModel::ERROR)
    return false;
  if (!_warn_filter_enabled && severity == LogsTableModel::WARNINGS)
    return false;
  if (!_debug_filter_enabled && severity == LogsTableModel::DEBUG)
    return false;

  if (_time_filter_enabled)
  {
    int64_t usec = sourceModel()->data(index_time, Qt::UserRole).toLongLong();
    auto timestamp = TimePoint() + std::chrono::microseconds(usec);

    if (timestamp < _min || timestamp > _max)
    {
      return false;
    }
  }

  if (_msg_filter_enabled)
  {
    const QString& text = sourceModel()->data(index_message, Qt::UserRole).toString();
    bool ret = applyFilter(_msg_text, _msg_mode, text, &_msg_validator);
    if (!ret)
    {
      return false;
    }
  }

  if (_source_filter_enabled)
  {
    const QString& text = sourceModel()->data(index_source, Qt::UserRole).toString();
    bool ret = applyFilter(_source_text, _source_mode, text, &_source_validator);
    if (!ret)
    {
      return false;
    }
  }

  if (_node_filter_enabled)
  {
    const QString& text = sourceModel()->data(index_node, Qt::UserRole).toString();
    bool ret = applyFilter(_node_text, _node_mode, text, &_node_validator);
    if (!ret)
    {
      return false;
    }
  }

  return true;
}

bool ModelFilter::applyFilter(const QString& filter, ModelFilter::FilterMode mode, const QString& text_to_parse,
                              const QRegExpValidator* validator) const
{
  // accept if no filter
  if (filter.count() == 0)
  {
    return true;
  }

  assert(!(validator == nullptr && (mode == WILDCARDS || mode == REGEX)));

  if (mode == CONTAINS_ONE)
  {
    QStringList filter_words = filter.split(QRegExp("\\s"), QString::SkipEmptyParts);

    for (int i = 0; i < filter_words.size(); i++)
    {
      if (text_to_parse.contains(filter_words[i], Qt::CaseSensitive) == true)
      {
        return true;
      }
    }
    return false;
  }
  //-----------------------
  if (mode == WILDCARDS || mode == REGEX)
  {
    QString message = text_to_parse;
    int pos = 0;
    return validator->validate(message, pos) == QValidator::Acceptable;
  }
  return false;
}
