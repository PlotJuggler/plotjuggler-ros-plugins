#pragma once
#include <QDialog>
#include <QStringList>

#include "ui_publisher_select_dialog.h"

namespace Ui
{
  class Ui_PublisherSelect;
}

class PublisherSelectDialog : public QDialog
{
  Q_OBJECT
private:
  Ui::PublisherSelect *_ui;

public:
  explicit PublisherSelectDialog(QWidget* parent = nullptr) : QDialog(parent), _ui(new Ui::PublisherSelect)
  {
    _ui->setupUi(this);
    _ui->listTopics->verticalHeader()->setVisible(false);
    QStringList labels;
    labels.push_back("Topic name");
    _ui->listTopics->setHorizontalHeaderLabels(labels);
  }

  Ui::PublisherSelect* ui()
  {
    return _ui;
  }

  ~PublisherSelectDialog()
  {
    delete _ui;
  }
};
