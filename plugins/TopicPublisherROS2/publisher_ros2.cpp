#include "publisher_ros2.h"

#include <QDebug>
#include <QDialog>
#include <QFormLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <QRadioButton>
#include <unordered_map>
#include <QMessageBox>
#include <tf2_ros/qos.hpp>
#include <rosbag2_cpp/types.hpp>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include "publisher_select_dialog.h"

TopicPublisherROS2::TopicPublisherROS2() :  _node(nullptr), _enabled(false)
{
  _context = std::make_shared<rclcpp::Context>();
  _context->init(0, nullptr);

  auto exec_args = rclcpp::executor::ExecutorArgs();
  exec_args.context = _context;
  _executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(exec_args, 2);

  _select_topics_to_publish = new QAction(QString("Select topics to be published"));
  connect(_select_topics_to_publish, &QAction::triggered,
          this, &TopicPublisherROS2::filterDialog);

  _available_actions.push_back(_select_topics_to_publish);
}

TopicPublisherROS2::~TopicPublisherROS2()
{
  _enabled = false;
}

const std::vector<QAction*> &TopicPublisherROS2::availableActions()
{
  return _available_actions;
}

void TopicPublisherROS2::updatePublishers()
{
  if( !_node )
  {
    return;
  }
  for (const auto& info : _topics_info)
  {
    auto to_publish = _topics_to_publish.find(info.name);
    if( to_publish == _topics_to_publish.end() || to_publish->second == false  )
    {
      continue; // no publish
    }

    auto publisher_it = _publishers.find(info.name);
    if (publisher_it == _publishers.end())
    {
      _publishers.insert({ info.name, GenericPublisher::create(*_node, info.name, info.type) });
    }
  }

  // remove already created publishers if not needed anymore
  for (auto it = _publishers.begin(); it != _publishers.end(); /* no increment */)
  {
    auto to_publish = _topics_to_publish.find(it->first);
    if( to_publish == _topics_to_publish.end() || to_publish->second == false  )
    {
      it = _publishers.erase(it);
    }
    else{
      it++;
    }
  }
}

void TopicPublisherROS2::setEnabled(bool to_enable)
{
  if (!_node && to_enable)
  {
    auto node_opts = rclcpp::NodeOptions();
    node_opts.context(_context);
    _node = std::make_shared<rclcpp::Node>("plotjuggler", node_opts);
    _executor->add_node(_node);
  }
  _enabled = (to_enable && _node);

  if (_enabled)
  {
    auto metadata_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::topics_metadata");
    if (metadata_it == _datamap->user_defined.end())
    {
      return;
    }
    // I stored it in a one point timeseries... shoot me
    const auto any_metadata = metadata_it->second[0].y;
    _topics_info = std::any_cast<std::vector<TopicInfo>>(any_metadata);

    // select all the topics by default
    for(const auto& info: _topics_info)
    {
      _topics_to_publish.insert( {info.name, true} );
    }

    updatePublishers();

    if (!_tf_broadcaster)
    {
      _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*_node);
    }
    if (!_tf_static_broadcaster)
    {
      _tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*_node);
    }

    _previous_play_index = std::numeric_limits<int>::max();
  }
  else
  {
    _node.reset();
    _publishers.clear();
    _tf_broadcaster.reset();
    _tf_static_broadcaster.reset();
  }
}

void TopicPublisherROS2::filterDialog()
{
  auto metadata_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::topics_metadata");
  if (metadata_it != _datamap->user_defined.end())
  {
    // I stored it in a one point timeseries... shoot me
    const auto any_metadata = metadata_it->second[0].y;
    _topics_info = std::any_cast<std::vector<TopicInfo>>(any_metadata);
  }

  PublisherSelectDialog *dialog =  new PublisherSelectDialog();
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->ui()->radioButtonClock->setHidden(true);
  dialog->ui()->radioButtonHeaderStamp->setHidden(true);

  std::map<std::string, QCheckBox*> checkbox;

  for (const TopicInfo& info : _topics_info)
  {
    const std::string topic_name = info.name;
    auto cb = new QCheckBox(dialog);
    auto filter_it = _topics_to_publish.find(topic_name);
    if (filter_it == _topics_to_publish.end())
    {
      cb->setChecked(true);
    }
    else
    {
      cb->setChecked(filter_it->second);
    }
    cb->setFocusPolicy(Qt::NoFocus);
    dialog->ui()->formLayout->addRow(new QLabel(QString::fromStdString(topic_name)), cb);
    checkbox.insert(std::make_pair(topic_name, cb));
    connect(dialog->ui()->pushButtonSelect, &QPushButton::pressed, [cb]() { cb->setChecked(true); });
    connect(dialog->ui()->pushButtonDeselect, &QPushButton::pressed, [cb]() { cb->setChecked(false); });
  }

  dialog->exec();

  if (dialog->result() == QDialog::Accepted)
  {
    _topics_to_publish.clear();
    for (const auto& it : checkbox)
    {
      _topics_to_publish.insert({ it.first, it.second->isChecked() });
    }

    updatePublishers();
  }
}

constexpr  long NSEC_PER_SEC = 1000000000;

rcutils_time_point_value_t Convert(const builtin_interfaces::msg::Time& stamp)
{
  return stamp.nanosec + NSEC_PER_SEC*stamp.sec;
}

builtin_interfaces::msg::Time Convert(const rcutils_time_point_value_t& time_stamp)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(time_stamp / NSEC_PER_SEC);
  stamp.nanosec = static_cast<int32_t>(time_stamp - (NSEC_PER_SEC * stamp.sec));
  return stamp;
}

void TopicPublisherROS2::broadcastTF(double current_time)
{
   using StringPair = std::pair<std::string, std::string>;

   std::map<StringPair, geometry_msgs::msg::TransformStamped> transforms;
   std::map<StringPair, geometry_msgs::msg::TransformStamped> static_transforms;

   for (const auto& data_it : _datamap->user_defined)
   {
     const std::string& topic_name = data_it.first;
     const PJ::PlotDataAny& plot_any = data_it.second;

     if (topic_name != "/tf_static" && topic_name != "/tf")
     {
       continue;
     }

     const PJ::PlotDataAny* tf_data = &plot_any;
     int last_index = tf_data->getIndexFromX(current_time);
     if (last_index < 0)
     {
       continue;
     }

     auto transforms_ptr = (topic_name == "/tf_static") ? & static_transforms : &transforms;

     std::vector<uint8_t> raw_buffer;
     // 2 seconds in the past (to be configurable in the future)
     int initial_index = tf_data->getIndexFromX(current_time - 2.0);

     if (_previous_play_index < last_index && _previous_play_index > initial_index)
     {
       initial_index = _previous_play_index;
     }

     for (size_t index = std::max(0, initial_index); index <= last_index; index++)
     {
       const std::any& any_value = tf_data->at(index).y;

       const bool isRosbagMessage = (any_value.type() == typeid(MessageRefPtr));

       if (!isRosbagMessage)
       {
         continue;
       }

       const auto& msg_instance = std::any_cast<MessageRefPtr>(any_value);

       auto tf_type_support = rosidl_typesupport_cpp::get_message_type_support_handle<tf2_msgs::msg::TFMessage>();
       tf2_msgs::msg::TFMessage tf_msg;
       if ( RMW_RET_OK != rmw_deserialize(msg_instance->serialized_data.get(), tf_type_support, &tf_msg) )
       {
         throw std::runtime_error("failed to deserialize TF message");
       }

       for (const auto& stamped_transform : tf_msg.transforms)
       {
         const auto& parent_id = stamped_transform.header.frame_id;
         const auto& child_id = stamped_transform.child_frame_id;
         StringPair trans_id = std::make_pair(parent_id, child_id);
         auto it = transforms_ptr->find(trans_id);
         if (it == transforms_ptr->end())
         {
           transforms_ptr->insert({ trans_id, stamped_transform });
         }
         else if (Convert(it->second.header.stamp) <=
                  Convert(stamped_transform.header.stamp))
         {
           it->second = stamped_transform;
         }
       }
     }

     // ready to broadacast
     std::vector<geometry_msgs::msg::TransformStamped> transforms_vector;
     transforms_vector.reserve(transforms_ptr->size());

     rcutils_time_point_value_t time_stamp;
     int error = rcutils_system_time_now(&time_stamp);

     if (error != RCUTILS_RET_OK)
     {
       qDebug() << "Error getting current time. Error:" << rcutils_get_error_string().str;
     }

     for (auto& trans : *transforms_ptr)
     {
       trans.second.header.stamp = Convert( time_stamp );
       transforms_vector.emplace_back(std::move(trans.second));
     }
     if( transforms_ptr == &transforms ){
       _tf_broadcaster->sendTransform(transforms_vector);
     }
     else{
      _tf_static_broadcaster->sendTransform(transforms_vector);
     }
   }

}


void TopicPublisherROS2::updateState(double current_time)
{
  if (!_enabled || !_node)
  {
    return;
  }

  //-----------------------------------------------
  broadcastTF(current_time);
  //-----------------------------------------------

  auto data_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::consecutive_messages");
  if (data_it != _datamap->user_defined.end())
  {
    const PJ::PlotDataAny& continuous_msgs = data_it->second;
    _previous_play_index = continuous_msgs.getIndexFromX(current_time);
  }

  for (const auto& data_it : _datamap->user_defined)
  {
    const std::string& topic_name = data_it.first;
    const PJ::PlotDataAny& plot_any = data_it.second;

    if (topic_name == "/tf" || topic_name == "tf_static")
    {
      continue;
    }

    auto publisher_it = _publishers.find(topic_name);
    if (publisher_it == _publishers.end())
    {
      continue;
    }

    int last_index = plot_any.getIndexFromX(current_time);
    if (last_index < 0)
    {
      continue;
    }

    const auto& any_value = plot_any.at(last_index).y;

    if (any_value.type() == typeid(MessageRefPtr))
    {
      const auto& msg_instance = std::any_cast<MessageRefPtr>(any_value);
      publisher_it->second->publish(msg_instance->serialized_data);
    }
  }
}

void TopicPublisherROS2::play(double current_time)
{
  if (!_enabled || !_node){
    return;
  }

  auto data_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::consecutive_messages");
  if (data_it == _datamap->user_defined.end())
  {
    return;
  }
  const PJ::PlotDataAny& continuous_msgs = data_it->second;
  int current_index = continuous_msgs.getIndexFromX(current_time);

  if (_previous_play_index > current_index)
  {
    _previous_play_index = current_index;
    updateState(current_time);
    return;
  }
  else
  {
    const PJ::PlotDataAny& consecutive_msg = data_it->second;
    for (int index = _previous_play_index + 1; index <= current_index; index++)
    {
      const auto& any_value = consecutive_msg.at(index).y;
      if (any_value.type() == typeid(MessageRefPtr))
      {
        const auto& msg_instance = std::any_cast<MessageRefPtr>(any_value);

        auto publisher_it = _publishers.find(msg_instance->topic_name);
        if (publisher_it == _publishers.end())
        {
          continue;
        }

        publisher_it->second->publish(msg_instance->serialized_data);
      }
    }
    _previous_play_index = current_index;
  }
}

