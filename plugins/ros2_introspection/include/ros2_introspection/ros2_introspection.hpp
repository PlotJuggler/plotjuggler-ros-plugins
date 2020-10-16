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
#pragma once

#include <unordered_map>
#include <ros2_introspection/stringtree.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

namespace Ros2Introspection{

struct FlatMessage {

  /// Tree that the StringTreeLeaf(s) refer to.
  const StringTree* tree;

  /// List of all those parsed fields that can be represented by a builtin value different from "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, double> > values;

  /// List of all those parsed fields that can be represented by a builtin value equal to "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, std::string> > strings;

  /// Store "blobs", i.e all those fields which are vectors of BYTES (AKA uint8_t),
  /// where the vector size is greater than the argument [max_array_size]
  /// passed  to the function deserializeIntoFlatContainer
  std::vector< std::pair<StringTreeLeaf, BufferView> > blobs;
};

typedef std::vector< std::pair<std::string, double> > RenamedValues;


struct TopicInfo{

  TopicInfo(const std::string& type);

  std::string topic_type;
  bool has_header_stamp;
  const rosidl_message_type_support_t *introspection_support;
  const rosidl_message_type_support_t *type_support;

  static rcutils_allocator_t allocator;
};

enum MaxArrayPolicy: bool {
  DISCARD_LARGE_ARRAYS = true,
  KEEP_LARGE_ARRAYS = false
};

class Parser{

public:
  Parser(const std::string &topic_name, const std::string& type_name);

  enum{ MAX_ARRAY_SIZE = 9999 };

  void setMaxArrayPolicy( MaxArrayPolicy discard_policy, size_t max_size );

  MaxArrayPolicy maxArrayPolicy() const;

  size_t maxArraySize() const;

  bool deserializeIntoFlatMessage(const rcutils_uint8_array_t *msg,
                                  FlatMessage* flat_container_output) const;

  const TopicInfo& topicInfo() const;

private:

  MaxArrayPolicy _discard_policy;

  size_t _max_array_size;

  StringTree _field_tree;

  TopicInfo _topic_info;
};

void ConvertFlatMessageToRenamedValues(const FlatMessage& flat, RenamedValues& renamed );

bool TypeHasHeader(const rosidl_message_type_support_t* type_support);

}
