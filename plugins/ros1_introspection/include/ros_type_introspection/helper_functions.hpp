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


#ifndef ROS_INTROSPECTION_HELPER_H
#define ROS_INTROSPECTION_HELPER_H

#include <functional>
#include <ros_type_introspection/utils/variant.hpp>
#include <ros_type_introspection/utils/span.hpp>

namespace RosIntrospection{

template< class T>
using Span = nonstd::span<T>;

// Brutally faster for numbers below 100
inline int print_number(char* buffer, uint16_t value)
{
  const char DIGITS[] =
      "00010203040506070809"
      "10111213141516171819"
      "20212223242526272829"
      "30313233343536373839"
      "40414243444546474849"
      "50515253545556575859"
      "60616263646566676869"
      "70717273747576777879"
      "80818283848586878889"
      "90919293949596979899";
  if (value < 10)
  {
    buffer[0] = static_cast<char>('0' + value);
    return 1;
  }
  else if (value < 100) {
    value *= 2;
    buffer[0] = DIGITS[ value ];
    buffer[1] = DIGITS[ value+1 ];
    return 2;
  }
  else{
    return sprintf( buffer,"%d", value );
  }
}


// helper function to deserialize raw memory
template <typename T> inline void ReadFromBuffer( const Span<uint8_t>& buffer, size_t& offset, T& destination)
{
  if ( offset + sizeof(T) > static_cast<std::size_t>(buffer.size()) )
  {
    throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
  }
  destination =  (*( reinterpret_cast<const T*>( &(buffer.data()[offset]) ) ) );
  offset += sizeof(T);
}

template <> inline void ReadFromBuffer( const Span<uint8_t>& buffer, size_t& offset, std::string& destination)
{
  uint32_t string_size = 0;
  ReadFromBuffer( buffer, offset, string_size );

  if( offset + string_size > static_cast<std::size_t>(buffer.size()) )
  {
    throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
  }

  const char* buffer_ptr = reinterpret_cast<const char*>( &buffer[offset] );
  offset += string_size;

  destination.assign( buffer_ptr, string_size );
}

template <typename T> inline
Variant ReadFromBufferToVariant( const Span<uint8_t>& buffer, size_t& offset)
{
  T destination;
  ReadFromBuffer(buffer, offset, destination);
  return Variant(destination);
}

inline Variant ReadFromBufferToVariant(BuiltinType id, const Span<uint8_t>& buffer, size_t& offset)
{
  switch(id)
  {
  case BOOL: return ReadFromBufferToVariant<bool>(buffer,offset);
  case CHAR: return ReadFromBufferToVariant<char>(buffer,offset);
  case BYTE:
  case UINT8:  return ReadFromBufferToVariant<uint8_t>(buffer,offset);
  case UINT16: return ReadFromBufferToVariant<uint16_t>(buffer,offset);
  case UINT32: return ReadFromBufferToVariant<uint32_t>(buffer,offset);
  case UINT64: return ReadFromBufferToVariant<uint64_t>(buffer,offset);

  case INT8:   return ReadFromBufferToVariant<int8_t>(buffer,offset);
  case INT16:  return ReadFromBufferToVariant<int16_t>(buffer,offset);
  case INT32:  return ReadFromBufferToVariant<int32_t>(buffer,offset);
  case INT64:  return ReadFromBufferToVariant<int64_t>(buffer,offset);

  case FLOAT32:  return ReadFromBufferToVariant<float>(buffer,offset);
  case FLOAT64:  return ReadFromBufferToVariant<double>(buffer,offset);

  case TIME: {
    ros::Time tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }
  case DURATION: {
    ros::Duration tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }

  case STRING: {
    uint32_t string_size = 0;
    ReadFromBuffer( buffer, offset, string_size );
    if( offset + string_size > static_cast<std::size_t>(buffer.size()) ) {
      throw std::runtime_error("Buffer overrun");
    }
    Variant var_string(reinterpret_cast<const char*>( &buffer[offset] ), string_size  );
    offset += string_size;
    return var_string;
  }
  case OTHER: return -1;
  default: break;
  }
  throw std::runtime_error( "unsupported builtin type value");
}


} // end namespace


#endif
