#ifndef DIALOG_WITH_ITEMLIST_H
#define DIALOG_WITH_ITEMLIST_H

#include <QDialog>
#include <QLabel>
#include <QListWidget>
#include <QAbstractButton>
#include <unordered_set>

#include "ui_dialog_with_itemlist.h"

namespace Ui
{
class DialogWithItemList;
}

class DialogWithItemList : public QDialog
{
  Q_OBJECT
private:
  Ui::DialogWithItemList* ui;

public:
  explicit DialogWithItemList(QWidget* parent, QString title, QString text, std::unordered_set<std::string> list)
    : QDialog(parent), ui(new Ui::DialogWithItemList)
  {
    QStringList name_list;
    for (auto& name : list)
    {
      name_list.push_back(name.c_str());
    }

    ui->setupUi(this);
    this->setWindowTitle(title);
    ui->label->setText(text);
    ui->listWidget->addItems(name_list);
    ui->listWidget->sortItems();
  }
  ~DialogWithItemList()
  {
    delete ui;
  }

  static void warning(const QString& message, std::unordered_set<std::string> list)
  {
    auto dialog = new DialogWithItemList(0, tr("Warning"), message, list);
    dialog->exec();
    dialog->deleteLater();
  }

private slots:
  void on_buttonBox_clicked(QAbstractButton* button)
  {
    this->close();
  }
};

#endif  // DIALOG_WITH_ITEMLIST_H
