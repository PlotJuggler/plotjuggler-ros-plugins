/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016-2017 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
* *******************************************************************/


#ifndef ROS_INTROPSECTION_SHAPE_SHIFTER2_H
#define ROS_INTROPSECTION_SHAPE_SHIFTER2_H

#include "ros/ros.h"
#include "ros/console.h"
#include "ros/assert.h"
#include <vector>
#include <boost/flyweight.hpp>
#include <ros/message_traits.h>
#include "ros_type_introspection/ros_introspection.hpp"

namespace RosIntrospection
{

/**
 * @brief The ShapeShifter class is a type erased container for ROS Messages.
 * It can be used also to create generic publishers and subscribers.
 */
class ShapeShifter
{
public:
  typedef boost::shared_ptr<ShapeShifter> Ptr;
  typedef boost::shared_ptr<ShapeShifter const> ConstPtr;

  static bool uses_old_API_;

  // Constructor and destructor
  ShapeShifter();
  virtual ~ShapeShifter();

  // Helpers for inspecting ShapeShifter
  std::string const& getDataType()          const;
  std::string const& getMD5Sum()            const;
  std::string const& getMessageDefinition() const;

  // Helper for advertising
  ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                           bool latch=false,
                           const ros::SubscriberStatusCallback &connect_cb=ros::SubscriberStatusCallback()) const;

  //! Call to try instantiating as a particular type
  template<class M>
  void instantiate(M& destination) const;

  //! Write serialized message contents out to a stream
  template<typename Stream>
  void write(Stream& stream) const;

  const uint8_t* raw_data() const;

  template<typename Stream>
  void read(Stream& stream);

  ///! Directly serialize the contentof a message into this ShapeShifter.
  template<typename Message>
  void direct_read(const Message& msg,bool morph);

  //! Return the size of the serialized message
  uint32_t size() const;

  void morph(const std::string& md5sum, const std::string& datatype_, const std::string& msg_def_);

private:

  std::string md5_;
  std::string datatype_;
  std::string msg_def_;
  bool typed_;

  mutable std::vector<uint8_t> msgBuf_;

};

}


// Message traits allow shape shifter to work with the new serialization API
namespace ros {
namespace message_traits {

template <> struct IsMessage<RosIntrospection::ShapeShifter> : TrueType { };
template <> struct IsMessage<const RosIntrospection::ShapeShifter> : TrueType { };

template<>
struct MD5Sum<RosIntrospection::ShapeShifter>
{
  static const char* value(const RosIntrospection::ShapeShifter& m) { return m.getMD5Sum().data(); }

  // Used statically, a ShapeShifter2 appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct DataType<RosIntrospection::ShapeShifter>
{
  static const char* value(const RosIntrospection::ShapeShifter& m) { return m.getDataType().data(); }

  // Used statically, a ShapeShifter2 appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct Definition<RosIntrospection::ShapeShifter>
{
  static const char* value(const RosIntrospection::ShapeShifter& m) { return m.getMessageDefinition().data(); }
};

} // namespace message_traits


namespace serialization
{

template<>
struct Serializer<RosIntrospection::ShapeShifter>
{
  template<typename Stream>
  inline static void write(Stream& stream, const RosIntrospection::ShapeShifter& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, RosIntrospection::ShapeShifter& m)
  {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const RosIntrospection::ShapeShifter& m) {
    return m.size();
  }
};


template<>
struct PreDeserialize<RosIntrospection::ShapeShifter>
{
  static void notify(const PreDeserializeParams<RosIntrospection::ShapeShifter>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];

    params.message->morph(md5, datatype, msg_def);
  }
};

} // namespace serialization

} //namespace ros



// Template implementations:

namespace RosIntrospection
{

template<class M> inline 
void ShapeShifter::instantiate(M& destination) const
{
  if (!typed_)
    throw std::runtime_error("Tried to instantiate message from an untyped ShapeShifter2.");

  if (ros::message_traits::datatype<M>() != getDataType())
    throw std::runtime_error("Tried to instantiate message without matching datatype.");

  if (ros::message_traits::md5sum<M>() != getMD5Sum())
    throw std::runtime_error("Tried to instantiate message without matching md5sum.");

  ros::serialization::IStream s(msgBuf_.data(), msgBuf_.size() );
  ros::serialization::deserialize(s, destination);

}

template<typename Stream> inline 
void ShapeShifter::write(Stream& stream) const {
  if (msgBuf_.size() > 0)
    memcpy(stream.advance(msgBuf_.size()), msgBuf_.data(), msgBuf_.size());
}

inline const uint8_t* ShapeShifter::raw_data() const {
  return msgBuf_.data();
}

inline uint32_t ShapeShifter::size() const
{
  return msgBuf_.size();
}

template<typename Stream> inline 
void ShapeShifter::read(Stream& stream)
{
  //allocate enough space
  msgBuf_.resize( stream.getLength() );
  //copy
  memcpy(msgBuf_.data(), stream.getData(), stream.getLength());
}

template<typename Message> inline 
void ShapeShifter::direct_read(const Message& msg, bool do_morph)
{
  if(do_morph)
  {
    this->morph(
          ros::message_traits::MD5Sum<Message>::value(),
          ros::message_traits::DataType<Message>::value(),
          ros::message_traits::Definition<Message>::value());
  }

  auto length = ros::serialization::serializationLength(msg);

  //allocate enough space
  msgBuf_.resize( length );
  //copy
  ros::serialization::OStream o_stream(msgBuf_.data(), length);
  ros::serialization::serialize(o_stream, msg);
}

inline ShapeShifter::ShapeShifter()
  :  typed_(false),
     msgBuf_()
{
}


inline ShapeShifter::~ShapeShifter()
{

}


inline std::string const& ShapeShifter::getDataType()          const { return datatype_; }


inline std::string const& ShapeShifter::getMD5Sum()            const { return md5_;   }


inline std::string const& ShapeShifter::getMessageDefinition() const { return msg_def_;  }


inline void ShapeShifter::morph(const std::string& _md5sum, const std::string& _datatype, const std::string& _msg_def)
{
  md5_ = _md5sum;
  datatype_ = _datatype;
  msg_def_ = _msg_def;
  typed_ = (md5_ != "*");
}


inline ros::Publisher ShapeShifter::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch,
                                       const ros::SubscriberStatusCallback &connect_cb) const
{
  ros::AdvertiseOptions opts(topic, queue_size, getMD5Sum(), getDataType(), getMessageDefinition(), connect_cb);
  opts.latch = latch;
  return nh.advertise(opts);
}

} // namespace


#endif

