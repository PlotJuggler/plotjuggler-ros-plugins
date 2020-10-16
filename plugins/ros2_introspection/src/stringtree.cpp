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

#include <ros2_introspection/stringtree.hpp>

namespace Ros2Introspection
{
namespace details
{
// Brutally faster for numbers below 100
inline int printNumber(char* buffer, uint16_t value)
{
  const char DIGITS[] = "00010203040506070809"
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
  else if (value < 100)
  {
    value *= 2;
    buffer[0] = DIGITS[value];
    buffer[1] = DIGITS[value + 1];
    return 2;
  }
  else
  {
    return sprintf(buffer, "%d", value);
  }
}
}  // end namespace details

int StringTreeLeaf::toStr(std::string& buffer_str) const
{
  const StringTreeNode* leaf_node = this->node_ptr;
  if (!leaf_node)
  {
    return -1;
  }

  boost::container::static_vector<const std::string*, 16> strings_chain;
  size_t num_bytes = 2;

  while (leaf_node)
  {
    const auto& str = leaf_node->value();
    const size_t S = str.size();
    if (S == 1 && str[0] == NUM_PLACEHOLDER)
    {
      num_bytes += 5;  // space for up to 9999
    }
    else
    {
      num_bytes += S + 1;
    }
    strings_chain.push_back(&str);
    leaf_node = leaf_node->parent();
  };

  std::reverse(strings_chain.begin(), strings_chain.end());

  size_t array_count = 0;
  size_t offset = 0;

  buffer_str.resize(num_bytes);  // expand the buffer
  char* buffer = &buffer_str.front();

  for (const auto& str : strings_chain)
  {
    const size_t S = str->size();
    if (S == 1 && (*str)[0] == NUM_PLACEHOLDER)
    {
      buffer[offset++] = '.';
      offset += details::printNumber(&buffer[offset], this->index_array[array_count++]);
    }
    else
    {
      if (offset != 0)
      {
        buffer[offset++] = SEPARATOR;
      }
      std::memcpy(&buffer[offset], str->data(), S);
      offset += S;
    }
  }

  buffer_str.resize(offset);
  return int(offset);
}

}  // namespace Ros2Introspection
