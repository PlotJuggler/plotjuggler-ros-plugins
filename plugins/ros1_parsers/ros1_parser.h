#pragma once

#include "ros_type_introspection/ros_introspection.hpp"
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/messageparser_base.h"

//----------------------------------

enum LargeArrayPolicy : bool
{
  DISCARD_LARGE_ARRAYS = true,
  KEEP_LARGE_ARRAYS = false
};

class RosMessageParser: public MessageParser
{
public:
  RosMessageParser(const std::string& topic_name, PlotDataMapRef& plot_data)
  {
    init(topic_name, &plot_data);
  }

  virtual void setMaxArrayPolicy(LargeArrayPolicy, size_t)
  { }

};

template <typename T>
class BuiltinMessageParser : public RosMessageParser
{
public:
  BuiltinMessageParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : RosMessageParser(topic_name, plot_data)
  {
  }

  virtual const char* formatName() const override
  {
    return ros::message_traits::DataType<T>::value();
  }

  virtual bool parseMessage(MessageRef serialized_msg, double timestamp) override
  {
    T msg;
    ros::serialization::IStream is(const_cast<uint8_t*>(serialized_msg.data()), serialized_msg.size());
    ros::serialization::deserialize(is, msg);
    parseMessageImpl(msg, timestamp);
    return true;
  }

  virtual void parseMessageImpl(const T& msg, double timestamp) = 0;

protected:
};

class IntrospectionParser : public RosMessageParser
{
public:
  IntrospectionParser(const std::string& topic_name, const std::string& topic_type, const std::string& definition,
                      PlotDataMapRef& plot_data)
    : RosMessageParser(topic_name, plot_data), _max_size(999)
  {
    auto type = RosIntrospection::ROSType(topic_type);
    _parser.registerMessageDefinition(topic_name, type, definition);
  }

  virtual const char* formatName() const override
  {
    return "IntrospectionParser";
  }

  void setMaxArrayPolicy(LargeArrayPolicy policy, size_t max_size) override;

  virtual bool parseMessage(MessageRef serialized_msg, double timestamp) override;

private:
  RosIntrospection::Parser _parser;
  RosIntrospection::FlatMessage _flat_msg;
  RosIntrospection::RenamedValues _renamed;
  size_t _max_size;
};

class CompositeParser
{
public:
  CompositeParser(PlotDataMapRef& plot_data);

  virtual void setUseHeaderStamp(bool use);

  virtual void setMaxArrayPolicy(LargeArrayPolicy policy, size_t max_size);

  void registerMessageType(const std::string& topic_name, const std::string& topic_type, const std::string& definition);

  bool parseMessage(const std::string& topic_name, MessageRef serialized_msg, double timestamp);

private:

  std::map<std::string, std::shared_ptr<RosMessageParser>> _parsers;

  LargeArrayPolicy _discard_policy;

  size_t _max_array_size;

  bool _use_header_stamp;

  PlotDataMapRef& _plot_data;
};
