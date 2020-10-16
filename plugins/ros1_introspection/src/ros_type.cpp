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

#include "ros_type_introspection/ros_type.hpp"
#include "ros_type_introspection/helper_functions.hpp"

namespace RosIntrospection
{
ROSType::ROSType(boost::string_ref name) : _base_name(name)
{
  int pos = -1;
  for (size_t i = 0; i < name.size(); i++)
  {
    if (name[i] == '/')
    {
      pos = i;
      break;
    }
  }

  if (pos == -1)
  {
    _msg_name = _base_name;
  }
  else
  {
    _pkg_name = boost::string_ref(_base_name.data(), pos);
    pos++;
    _msg_name = boost::string_ref(_base_name.data() + pos, _base_name.size() - pos);
  }

  _id = toBuiltinType(_msg_name);
  _hash = std::hash<std::string>{}(_base_name);
}

ROSType& ROSType::operator=(const ROSType& other)
{
  int pos = other._pkg_name.size();
  _base_name = other._base_name;
  _pkg_name = boost::string_ref(_base_name.data(), pos);
  if (pos > 0)
    pos++;
  _msg_name = boost::string_ref(_base_name.data() + pos, _base_name.size() - pos);
  _id = other._id;
  _hash = other._hash;
  return *this;
}

ROSType& ROSType::operator=(ROSType&& other)
{
  int pos = other._pkg_name.size();
  _base_name = std::move(other._base_name);
  _pkg_name = boost::string_ref(_base_name.data(), pos);
  if (pos > 0)
    pos++;
  _msg_name = boost::string_ref(_base_name.data() + pos, _base_name.size() - pos);
  _id = other._id;
  _hash = other._hash;
  return *this;
}

void ROSType::setPkgName(boost::string_ref new_pkg)
{
  assert(_pkg_name.size() == 0);

  int pos = new_pkg.size();
  _base_name = new_pkg.to_string() + "/" + _base_name;

  _pkg_name = boost::string_ref(_base_name.data(), pos++);
  _msg_name = boost::string_ref(_base_name.data() + pos, _base_name.size() - pos);

  _hash = std::hash<std::string>{}(_base_name);
}

}  // namespace RosIntrospection
