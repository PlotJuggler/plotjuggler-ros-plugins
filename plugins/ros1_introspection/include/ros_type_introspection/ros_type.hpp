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

#ifndef ROS_INTROSPECTION_ROSTYPE_H
#define ROS_INTROSPECTION_ROSTYPE_H

#include <vector>
#include <map>
#include <iostream>
#include <functional>
#include "ros_type_introspection/utils/variant.hpp"

namespace RosIntrospection{


/**
 * @brief ROSType
 */
class ROSType {
public:

  ROSType(){}

  ROSType(boost::string_ref name);

  ROSType(const ROSType& other) {  *this = other; }

  ROSType(ROSType&& other) {  *this = other; }

  ROSType& operator= (const ROSType& other);

  ROSType& operator= (ROSType&& other);

  /// Concatenation of msg_name and pkg_name.
  /// ex.: geometry_msgs/Pose"
  const std::string& baseName() const;

  /// ex.: geometry_msgs/Pose -> "Pose"
  const boost::string_ref& msgName()  const;

  /// ex.: geometry_msgs/Pose -> "geometry_msgs"
  const boost::string_ref& pkgName()  const;

  void setPkgName(boost::string_ref new_pkg);

  /// True if the type is ROS builtin
  bool isBuiltin() const;

  /// If builtin, size of builtin, -1 means variable or undefined
  int typeSize() const;

  /// If type is builtin, returns the id.  BuiltinType::OTHER otherwise.
  BuiltinType typeID() const;

  bool operator==(const ROSType& other) const  {
    return _hash == other._hash;
  }

  bool operator!=(const ROSType& other) const  {
    return (_hash != other._hash);
  }

  bool operator<(const ROSType& other) const {
    return this->baseName() < other.baseName();
  }

  size_t hash() const { return _hash; }

protected:

  BuiltinType _id;
  std::string _base_name;
  boost::string_ref _msg_name;
  boost::string_ref _pkg_name;
  size_t _hash;

};

//----------- definitions -------------

inline const std::string &ROSType::baseName() const
{
  return _base_name;
}

inline const boost::string_ref& ROSType::msgName() const
{
  return _msg_name;
}

inline const boost::string_ref &ROSType::pkgName() const
{
  return _pkg_name;
}

inline bool ROSType::isBuiltin() const
{
  return _id != RosIntrospection::OTHER;
}

inline int ROSType::typeSize() const
{
  return builtinSize( _id );
}

inline BuiltinType ROSType::typeID() const
{
  return _id;
}

//--------- helper functions --------------

inline std::ostream& operator<<(std::ostream &os, const ROSType& t )
{
  os << t.baseName();
  return os;
}

inline BuiltinType toBuiltinType(const boost::string_ref& s) {
  static std::map<boost::string_ref, BuiltinType> string_to_builtin_map {
    { "bool", BOOL },
    { "byte", BYTE },
    { "char", CHAR },
    { "uint8", UINT8 },
    { "uint16", UINT16 },
    { "uint32", UINT32 },
    { "uint64", UINT64 },
    { "int8", INT8 },
    { "int16", INT16 },
    { "int32", INT32 },
    { "int64", INT64 },
    { "float32", FLOAT32 },
    { "float64", FLOAT64 },
    { "time", TIME },
    { "duration", DURATION },
    { "string", STRING },
  };
  const auto it = string_to_builtin_map.find(s);
  return (it != string_to_builtin_map.cend()) ? it->second : OTHER;
}

}

namespace std {
  template <> struct hash<RosIntrospection::ROSType>
  {

    typedef RosIntrospection::ROSType argument_type;
    typedef std::size_t               result_type;

    result_type operator()(RosIntrospection::ROSType const& type) const
    {
      return type.hash();
    }
  };
}


#endif // ROSTYPE_H
