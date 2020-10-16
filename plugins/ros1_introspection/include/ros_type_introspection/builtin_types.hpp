
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

#ifndef ROS_BUILTIN_TYPES_HPP
#define ROS_BUILTIN_TYPES_HPP

#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include <unordered_map>

namespace RosIntrospection{


enum BuiltinType {
  BOOL , BYTE, CHAR,
  UINT8, UINT16, UINT32, UINT64,
  INT8, INT16, INT32, INT64,
  FLOAT32, FLOAT64,
  TIME, DURATION,
  STRING, OTHER
};

//---------------------------------------------------------

inline int builtinSize(const BuiltinType c) {
  switch (c) {
  case BOOL:
  case BYTE:
  case INT8:
  case CHAR:
  case UINT8:    return 1;
  case UINT16:
  case INT16:    return 2;
  case UINT32:
  case INT32:
  case FLOAT32:  return 4;
  case UINT64:
  case INT64:
  case FLOAT64:
  case TIME:
  case DURATION: return 8;
  case STRING:
  case OTHER: return -1;
  }
  throw std::runtime_error( "unsupported builtin type value");
}

inline const char* toStr(const BuiltinType& c)
{
  switch (c) {
  case BOOL:     return "BOOL";
  case BYTE:     return "BYTE";
  case INT8:     return "INT8";
  case CHAR:     return "CHAR";
  case UINT8:    return "UINT8";
  case UINT16:   return "UINT16";
  case UINT32:   return "UINT32";
  case UINT64:   return "UINT64";
  case INT16:    return "INT16";
  case INT32:    return "INT32";
  case INT64:    return "INT64";
  case FLOAT32:  return "FLOAT32";
  case FLOAT64:  return "FLOAT64";
  case TIME:     return "TIME";
  case DURATION: return "DURATION";
  case STRING:   return "STRING";
  case OTHER:    return "OTHER";
  }
  throw std::runtime_error( "unsupported builtin type value");
}

inline std::ostream& operator<<(std::ostream& os, const BuiltinType& c)
{
  os << toStr(c);
  return os;
}

template <typename T> BuiltinType getType()
{
    return OTHER;
}

template <> inline BuiltinType getType<bool>()  {  return BOOL; }

template <> inline BuiltinType getType<char>()           {  return CHAR; }

template <> inline BuiltinType getType<int8_t>()  {  return INT8; }
template <> inline BuiltinType getType<int16_t>() {  return INT16; }
template <> inline BuiltinType getType<int32_t>() {  return INT32; }
template <> inline BuiltinType getType<int64_t>() {  return INT64; }

template <> inline BuiltinType getType<uint8_t>()  {  return UINT8; }
template <> inline BuiltinType getType<uint16_t>() {  return UINT16; }
template <> inline BuiltinType getType<uint32_t>() {  return UINT32; }
template <> inline BuiltinType getType<uint64_t>() {  return UINT64; }

template <> inline BuiltinType getType<float>()  {  return FLOAT32; }
template <> inline BuiltinType getType<double>() {  return FLOAT64; }

template <> inline BuiltinType getType<std::string>() {  return STRING; }

template <> inline BuiltinType getType<ros::Time>()     {  return TIME; }
template <> inline BuiltinType getType<ros::Duration>() {  return DURATION; }

}

#endif // ROS_BUILTIN_TYPES_HPP
