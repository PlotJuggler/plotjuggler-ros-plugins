/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QDebug>
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QStringList>
#include <QHeaderView>
#include <iostream>
#include "logwidget.hpp"

namespace rqt_console_plus
{
using namespace Qt;

LogWidget::LogWidget(LogsTableModel& tablemodel, QWidget* parent)
  : QWidget(parent), model(tablemodel), proxy_model(this)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  proxy_model.setSourceModel(&model);
  ui.tableView->setModel(&proxy_model);
  // ui.tableView->setModel( &model );

  ui.tableView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Interactive);
  ui.tableView->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
  // ui.tableView->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);

  ui.tableView->verticalHeader()->setVisible(false);

  connect(&model, &LogsTableModel::rowsInserted, this, &LogWidget::on_rowsInserted);

  proxy_model.setSeverityDebugEnabled(ui.buttonEnableDebug->isChecked());
  proxy_model.setSeverityWarningsEnabled(ui.buttonEnableWarnings->isChecked());
  proxy_model.setSeverityErrorEnabled(ui.buttonEnableError->isChecked());
  proxy_model.setSeverityInfoEnabled(ui.buttonEnableInfo->isChecked());

  proxy_model.setMessageFilterEnabled(ui.checkBoxMessageFilter->isChecked());
  proxy_model.setNodeFilterEnabled(ui.checkBoxLoggerFilter->isChecked());
  proxy_model.setTimeFilterEnabled(true);
}

LogWidget::~LogWidget()
{
}

void LogWidget::on_lineEditMessageFilter_textEdited(const QString& filter)
{
  proxy_model.messageFilterUpdated(static_cast<ModelFilter::FilterMode>(ui.comboBoxMessageFilter->currentIndex()),
                                   filter);
  ui.tableView->resizeColumnToContents(3);
}

void LogWidget::on_comboBoxMessageFilter_currentIndexChanged(int mode)
{
  proxy_model.messageFilterUpdated(static_cast<ModelFilter::FilterMode>(mode), ui.lineEditMessageFilter->text());
  ui.tableView->resizeColumnToContents(4);
}

void LogWidget::on_checkBoxMessageFilter_toggled(bool checked)
{
  ui.labelMessageFilter->setEnabled(checked);
  ui.comboBoxMessageFilter->setEnabled(checked);
  ui.lineEditMessageFilter->setEnabled(checked);

  proxy_model.setMessageFilterEnabled(checked);
  ui.tableView->resizeColumnToContents(4);
}

void LogWidget::on_lineEditLoggerFilter_textEdited(const QString& filter)
{
  proxy_model.nodeFilterUpdated(static_cast<ModelFilter::FilterMode>(ui.comboBoxLoggerFilter->currentIndex()), filter);
  ui.tableView->resizeColumnToContents(3);
}

void LogWidget::on_comboBoxLoggerFilter_currentIndexChanged(int mode)
{
  proxy_model.nodeFilterUpdated(static_cast<ModelFilter::FilterMode>(mode), ui.lineEditLoggerFilter->text());
  ui.tableView->resizeColumnToContents(3);
}
void LogWidget::on_checkBoxLoggerFilter_toggled(bool checked)

{
  ui.labelLoggerFilter->setEnabled(checked);
  ui.comboBoxLoggerFilter->setEnabled(checked);
  ui.lineEditLoggerFilter->setEnabled(checked);

  proxy_model.setNodeFilterEnabled(checked);
  ui.tableView->resizeColumnToContents(3);
}

void LogWidget::on_rowsInserted(const QModelIndex&, int first_row, int last_row)
{
  ui.tableView->scrollToBottom();
}

void LogWidget::on_buttonEnableDebug_toggled(bool checked)
{
  proxy_model.setSeverityDebugEnabled(checked);
  ui.tableView->resizeColumnToContents(2);
}

void LogWidget::on_buttonEnableInfo_toggled(bool checked)
{
  proxy_model.setSeverityInfoEnabled(checked);
  ui.tableView->resizeColumnToContents(2);
}

void LogWidget::on_buttonEnableWarnings_toggled(bool checked)
{
  proxy_model.setSeverityWarningsEnabled(checked);
  ui.tableView->resizeColumnToContents(2);
}

void LogWidget::on_buttonEnableError_toggled(bool checked)
{
  proxy_model.setSeverityErrorEnabled(checked);
  ui.tableView->resizeColumnToContents(2);
}

void LogWidget::on_timeRangeChanged(TimePoint time_min, TimePoint time_max)
{
  using namespace std::chrono;
  {
    auto msec_since_epoch_A = duration_cast<milliseconds>(time_min.time_since_epoch());
    auto datetimeA = QDateTime::fromMSecsSinceEpoch(msec_since_epoch_A.count());
    ui.timeRangeMin->setDateTime(datetimeA);
  }

  {
    auto msec_since_epoch_B = duration_cast<milliseconds>(time_max.time_since_epoch());
    auto datetimeB = QDateTime::fromMSecsSinceEpoch(msec_since_epoch_B.count());
    ui.timeRangeMax->setDateTime(datetimeB);
  }

  // qDebug() << msec_since_epoch_A.count() << " " << msec_since_epoch_B.count();
  // qDebug() << datetimeA << " " << datetimeB;

  proxy_model.timeMinMaxUpdated(time_min, time_max);
  ui.tableView->scrollToBottom();
  ui.tableView->resizeColumnToContents(0);
  ui.tableView->resizeColumnToContents(1);
}

}  // namespace rqt_console_plus
