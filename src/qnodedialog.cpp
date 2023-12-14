#include "qnodedialog.h"
#include "ui_qnodedialog.h"
#include <boost/make_shared.hpp>
#include <QSettings>
#include <QMessageBox>

std::string getDefaultMasterURI()
{
  if (qgetenv("ROS_MASTER_URI").isEmpty())
  {
    QMessageBox msgBox;
    msgBox.setText("WARNINGS: the ROS_MASTER_URI is not defined in your environment\n"
                   "Using the default value [http://localhost:11311]\n");
    msgBox.exec();
    return "http://localhost:11311";
  }
  else
  {
    auto master_uri = (qgetenv("ROS_MASTER_URI"));
    return std::string(master_uri.data());
  }
}

QNodeDialog::QNodeDialog(QWidget* parent) : QDialog(parent), ui(new Ui::QNodeDialog)
{
  ui->setupUi(this);

  QSettings settings;

  auto master_ui = settings.value("QNode.master_uri", tr("http://localhost:11311")).toString();
  auto host_ip = settings.value("QNode.host_ip", tr("localhost")).toString();

  ui->lineEditMaster->setText(master_ui);
  ui->lineEditHost->setText(host_ip);
}

namespace ros
{
namespace master
{
void init(const M_string& remappings);
}
}  // namespace ros

bool QNodeDialog::Connect(const std::string& ros_master_uri, const std::string& hostname)
{
  std::map<std::string, std::string> remappings;
  remappings["__master"] = ros_master_uri;
  remappings["__hostname"] = hostname;

  static bool first_time = true;
  if (first_time || !ros::ok())
  {
    ros::init(remappings, "PlotJugglerListener", ros::init_options::AnonymousName);
    first_time = false;
  }
  else
  {
    ros::master::init(remappings);
  }
  ros::start();

  bool connected = ros::master::check();
  if (!connected)
  {
    QMessageBox msgBox;
    msgBox.setText(QString("Could not connect to the ros master [%1]").arg(QString::fromStdString(ros_master_uri)));
    msgBox.exec();
  }

  return connected;
}

QNodeDialog::~QNodeDialog()
{
  QSettings settings;
  settings.setValue("QNode.master_uri", ui->lineEditMaster->text());
  settings.setValue("QNode.host_ip", ui->lineEditHost->text());
  delete ui;
}

void QNodeDialog::on_pushButtonConnect_pressed()
{
  bool connected = false;
  if (ui->checkBoxUseEnvironment->isChecked())
  {
    const std::string master_uri = getDefaultMasterURI();
    connected = QNodeDialog::Connect(master_uri, "localhost");
  }
  else
  {
    std::string ros_master_uri = ui->lineEditMaster->text().toStdString();
    std::string hostname = ui->lineEditHost->text().toStdString();
    connected = QNodeDialog::Connect(ros_master_uri, hostname);
  }
  if (connected)
  {
    this->close();
  }
  else
  {
  }
}

void QNodeDialog::on_checkBoxUseEnvironment_toggled(bool checked)
{
  ui->lineEditMaster->setEnabled(!checked);
  ui->lineEditHost->setEnabled(!checked);
}

void QNodeDialog::on_pushButtonCancel_pressed()
{
  this->close();
}

RosManager& RosManager::get()
{
  static RosManager manager;
  return manager;
}

void RosManager::stopROS()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();;
    ros::waitForShutdown();
  }
}

RosManager::~RosManager()
{
}

ros::NodeHandlePtr RosManager::getNode()
{
  RosManager& manager = RosManager::get();

  if (!ros::isInitialized() || !ros::master::check() || !ros::ok())
  {
    bool connected = QNodeDialog::Connect(getDefaultMasterURI(), "localhost");
    if (!connected)
    {
      // as a fallback strategy, launch the QNodeDialog
      QNodeDialog dialog;
      dialog.exec();
    }
  }
  if (ros::master::check() && ros::isInitialized())
  {
    if (!manager._node)
    {
      manager._node.reset(new ros::NodeHandle, [](ros::NodeHandle* node) {
            delete node;
            RosManager::get().stopROS();
          });
    }
  }
  else{
    return nullptr;
  }
  return manager._node;
}
