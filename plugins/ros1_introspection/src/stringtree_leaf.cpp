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

#include "ros_type_introspection/stringtree_leaf.hpp"
#include "ros_type_introspection/helper_functions.hpp"

namespace RosIntrospection
{
int StringTreeLeaf::toStr(char* buffer) const
{
  const StringTreeNode* leaf_node = this->node_ptr;
  if (!leaf_node)
  {
    return -1;
  }

  boost::container::static_vector<const std::string*, 16> strings_chain;

  while (leaf_node)
  {
    const auto& str = leaf_node->value();
    strings_chain.push_back(&str);
    leaf_node = leaf_node->parent();
  };

  std::reverse(strings_chain.begin(), strings_chain.end());

  size_t array_count = 0;
  size_t offset = 0;

  for (const auto& str : strings_chain)
  {
    const size_t S = str->size();
    if (S == 1 && (*str)[0] == '#')
    {
      buffer[offset++] = '.';
      offset += print_number(&buffer[offset], this->index_array[array_count++]);
    }
    else
    {
      if (str != strings_chain.front())
      {
        buffer[offset++] = '/';
      }
      std::memcpy(&buffer[offset], str->data(), S);
      offset += S;
    }
  }
  buffer[offset] = '\0';
  return offset;
}

void CreateStringFromTreeLeaf(const StringTreeLeaf& leaf, bool skip_root, std::string& out)
{
  const StringTreeNode* leaf_node = leaf.node_ptr;
  if (!leaf_node)
  {
    out.clear();
    return;
  }

  boost::container::static_vector<const std::string*, 16> strings_chain;

  size_t total_size = 0;

  while (leaf_node)
  {
    const auto& str = leaf_node->value();
    leaf_node = leaf_node->parent();
    if (!(leaf_node == nullptr && skip_root))
    {
      strings_chain.emplace_back(&str);
      const size_t S = str.size();
      if (S == 1 && str[0] == '#')
      {
        total_size += 5;  // super conservative
      }
      else
      {
        total_size += S + 1;
      }
    }
  };

  out.resize(total_size);
  char* buffer = &out[0];

  std::reverse(strings_chain.begin(), strings_chain.end());

  size_t array_count = 0;
  size_t offset = 0;

  for (const auto& str : strings_chain)
  {
    const size_t S = str->size();
    if (S == 1 && (*str)[0] == '#')
    {
      buffer[offset++] = '.';
      offset += print_number(&buffer[offset], leaf.index_array[array_count++]);
    }
    else
    {
      if (str != strings_chain.front())
      {
        buffer[offset++] = '/';
      }
      std::memcpy(&buffer[offset], str->data(), S);
      offset += S;
    }
  }
  out.resize(offset);
}

}  // namespace RosIntrospection
