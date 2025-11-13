#include "publisher_select_dialog.h"
#include <QStringList>

void PublisherSelectDialog::on_lineEditFilter_textChanged(const QString &search_string)
{
  QStringList spaced_items = search_string.split(' ');

  for (const auto &pair : _topic_widgets)
  {
    const std::string &topic_name = pair.first;
    QLabel *label = pair.second.first;
    QCheckBox *checkbox = pair.second.second;

    QString name = QString::fromStdString(topic_name);
    bool toHide = false;

    for (const auto &item : spaced_items)
    {
      if (!name.contains(item, Qt::CaseInsensitive))
      {
        toHide = true;
        break;
      }
    }

    label->setVisible(!toHide);
    checkbox->setVisible(!toHide);
  }
}
