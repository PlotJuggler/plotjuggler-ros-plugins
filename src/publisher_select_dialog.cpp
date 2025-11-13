#include "publisher_select_dialog.h"
#include <QStringList>
#include <QTableWidgetItem>
#include <QAbstractItemView>

void PublisherSelectDialog::on_lineEditFilter_textChanged(const QString &search_string)
{
  QStringList spaced_items = search_string.split(' ');

  for (int row = 0; row < _ui->listTopics->rowCount(); row++)
  {
    auto item = _ui->listTopics->item(row, 0);
    QString name = item->text();
    bool toHide = false;

    for (const auto &item : spaced_items)
    {
      if (!name.contains(item, Qt::CaseInsensitive))
      {
        toHide = true;
        break;
      }
    }
    _ui->listTopics->setRowHidden(row, toHide);
  }
}

void PublisherSelectDialog::on_listTopics_itemSelectionChanged()
{
  QModelIndexList indexes = _ui->listTopics->selectionModel()->selectedIndexes();
  _ui->buttonBox->setEnabled(indexes.size() > 0);
}
