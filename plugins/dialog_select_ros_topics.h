#ifndef DIALOG_SELECT_ROS_TOPICS_H
#define DIALOG_SELECT_ROS_TOPICS_H

#include <QDialog>
#include <QString>
#include <QFile>
#include <QStringList>
#include <QCheckBox>
#include <QShortcut>
#include <QDomDocument>

#include "parser_configuration.h"

namespace Ui
{
class dialogSelectRosTopics;
}

class DialogSelectRosTopics : public QDialog
{
  Q_OBJECT

public:

  explicit DialogSelectRosTopics(const std::vector<std::pair<QString, QString>>& topic_list,
                                   const PJ::RosParserConfig& default_info, QWidget* parent = nullptr);

  ~DialogSelectRosTopics() override;

  PJ::RosParserConfig getResult() const;

public slots:

  void updateTopicList(std::vector<std::pair<QString, QString>> topic_list);

private slots:

  void on_buttonBox_accepted();

  void on_listRosTopics_itemSelectionChanged();

  void on_maximumSizeHelp_pressed();

  void on_lineEditFilter_textChanged(const QString& search_string);

  void on_spinBoxArraySize_valueChanged(int value);

private:
  QStringList _topic_list;
  QStringList _default_selected_topics;

  QShortcut _select_all;
  QShortcut _deselect_all;

  Ui::dialogSelectRosTopics* ui;
};

#endif  // DIALOG_SELECT_ROS_TOPICS_H
