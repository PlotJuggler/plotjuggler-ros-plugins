#pragma once

#include <PlotJuggler/plotdata.h>
#include <PlotJuggler/messageparser_base.h>
#include "ros_type_introspection/ros_introspection.hpp"
#include "parser_configuration.h"
//----------------------------------

using namespace PJ;


template <typename T>
class BuiltinMessageParser : public RosMessageParser
{
  public:
  BuiltinMessageParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
      : RosMessageParser(topic_name, plot_data)
  {
  }

  bool parseMessage(MessageRef serialized_msg, double& timestamp) override
  {
    T msg;
    ros::serialization::IStream is(const_cast<uint8_t*>(serialized_msg.data()), serialized_msg.size());
    ros::serialization::deserialize(is, msg);
    parseMessageImpl(msg, timestamp);
    return true;
  }

  virtual void parseMessageImpl(const T& msg, double& timestamp) = 0;

  protected:
};

class IntrospectionParser : public RosMessageParser
{
  public:
  IntrospectionParser(const std::string& topic_name,
                      const std::string& topic_type,
                      const std::string& definition,
                      PJ::PlotDataMapRef& plot_data)
      : RosMessageParser(topic_name, plot_data)
  {
    auto type = RosIntrospection::ROSType(topic_type);
    _parser.registerMessageDefinition(topic_name, type, definition);
  }

  virtual bool parseMessage(MessageRef serialized_msg, double& timestamp) override;

  private:
  RosIntrospection::Parser _parser;
  RosIntrospection::FlatMessage _flat_msg;
  RosIntrospection::RenamedValues _renamed;

};

class RosCompositeParser: public CompositeParser
{
  public:

  RosCompositeParser(PlotDataMapRef& plot_data): CompositeParser(plot_data) {}

  void registerMessageType(const std::string& topic_name,
                           const std::string& topic_type,
                           const std::string& definition);
};



