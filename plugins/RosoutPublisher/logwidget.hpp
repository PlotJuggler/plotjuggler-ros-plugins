/**
 * @file /include/rqt_console_plus/main_window.hpp
 *
 * @brief Qt based gui for rqt_console_plus.
 *
 * @date November 2010
 **/
#ifndef _LOG_WIDGET_H
#define _LOG_WIDGET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QWidget>
#include <QValidator>
#include "logs_table_model.hpp"
#include "ui_logwidget.h"
#include "modelfilter.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rqt_console_plus {


class LogWidget : public QWidget
{
  Q_OBJECT

public:
  explicit LogWidget(LogsTableModel& tablemodel, QWidget *parent = 0);
  ~LogWidget();


private slots:
  void on_lineEditMessageFilter_textEdited(const QString &filter);

  void on_checkBoxMessageFilter_toggled(bool checked);

  void on_checkBoxLoggerFilter_toggled(bool checked);

  void on_comboBoxMessageFilter_currentIndexChanged(int index);

  void on_buttonEnableDebug_toggled(bool checked);

  void on_rowsInserted(const QModelIndex & parent, int first_row, int last_row);

  void on_buttonEnableInfo_toggled(bool checked);

  void on_buttonEnableWarnings_toggled(bool checked);

  void on_buttonEnableError_toggled(bool checked);

  void on_lineEditLoggerFilter_textEdited(const QString &arg1);

  void on_comboBoxLoggerFilter_currentIndexChanged(int index);

public slots:

  void on_timeRangeChanged(TimePoint time_min, TimePoint time_max);

private:

  Ui::LogWidgetDesign ui;
  LogsTableModel& model;

  ModelFilter proxy_model;
};

}  // namespace rqt_console_plus

#endif // rqt_console_plus_MAIN_WINDOW_H
