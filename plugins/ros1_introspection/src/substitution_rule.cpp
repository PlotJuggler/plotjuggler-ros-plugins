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

#include "ros_type_introspection/substitution_rule.hpp"
#include <boost/algorithm/string/split.hpp>

namespace RosIntrospection
{
std::vector<boost::string_ref> StrSplit(boost::string_ref str, boost::string_ref delim)
{
  std::vector<boost::string_ref> out;
  size_t p;
  while (true)
  {
    p = str.find_first_of(delim);
    out.push_back(str.substr(0, p));
    if (p == boost::string_ref::npos)
    {
      break;
    }
    str.remove_prefix(p + 1);
  }
  return out;
}

SubstitutionRule::SubstitutionRule(const char* pattern, const char* alias, const char* substitution)
  : _full_pattern(pattern), _full_alias(alias), _full_substitution(substitution)
{
  _pattern = StrSplit(_full_pattern, "./");
  _alias = StrSplit(_full_alias, "./");
  _substitution = StrSplit(_full_substitution, "./");

  size_t h1 = std::hash<std::string>{}(_full_pattern);
  size_t h2 = std::hash<std::string>{}(_full_alias);
  size_t h3 = std::hash<std::string>{}(_full_substitution);
  _hash = (h1 ^ (h2 << 1)) ^ (h3 << 1);
}

SubstitutionRule& SubstitutionRule::operator=(const SubstitutionRule& other)
{
  _full_pattern = (other._full_pattern);
  _full_alias = (other._full_alias);
  _full_substitution = (other._full_substitution);
  // this could be optimized...
  _pattern = StrSplit(_full_pattern, "./");
  _alias = StrSplit(_full_alias, "./");
  _substitution = StrSplit(_full_substitution, "./");

  _hash = other._hash;
  return *this;
}

}  // namespace RosIntrospection
