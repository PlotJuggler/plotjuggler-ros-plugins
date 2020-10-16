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

#ifndef ROS_INTROSPECTION_ROSMESSAGE_H
#define ROS_INTROSPECTION_ROSMESSAGE_H

#include "ros_type_introspection/utils/tree.hpp"
#include "ros_type_introspection/ros_field.hpp"

namespace RosIntrospection{


class ROSMessage{
public:

  /// This constructor does most of the work in terms of parsing.
  /// It uses the message definition to extract fields and types.
  ROSMessage(const std::string& msg_def );

  /**
   * @brief Get field by index.
   */
  const ROSField& field(size_t index) const { return _fields[index]; }

  /**
   * @brief Vector of fields.
   * @return
   */
  const std::vector<ROSField>& fields() const { return _fields; }

  const ROSType& type() const { return _type; }

  void mutateType(const ROSType& new_type ) { _type = new_type; }

  void updateMissingPkgNames(const std::vector<const ROSType *> &all_types);

private:

  ROSType _type;
  std::vector<ROSField> _fields;
};

typedef details::TreeNode<std::string> StringTreeNode;
typedef details::Tree<std::string> StringTree;

typedef details::TreeNode<const ROSMessage*> MessageTreeNode;
typedef details::Tree<const ROSMessage*> MessageTree;

struct ROSMessageInfo
{
  StringTree  string_tree;
  MessageTree message_tree;
  std::vector<ROSMessage> type_list;
};

//------------------------------------------------

inline std::ostream& operator<<(std::ostream &os, const ROSMessage& msg )
{
  os << msg.type();
  return os;
}

inline std::ostream& operator<<(std::ostream &os, const ROSMessage* msg )
{
  os << msg->type();
  return os;
}


}

#endif
