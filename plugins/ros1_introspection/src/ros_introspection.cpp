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

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <functional>
#include "ros_type_introspection/ros_introspection.hpp"
#include "ros_type_introspection/helper_functions.hpp"

namespace RosIntrospection
{
void Parser::createTrees(ROSMessageInfo& info, const std::string& type_name) const
{
  std::function<void(const ROSMessage*, StringTreeNode*, MessageTreeNode*)> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSMessage* msg_definition, StringTreeNode* string_node, MessageTreeNode* msg_node) {
    // note: should use reserve here, NOT resize
    const size_t NUM_FIELDS = msg_definition->fields().size();

    string_node->children().reserve(NUM_FIELDS);
    msg_node->children().reserve(NUM_FIELDS);

    for (const ROSField& field : msg_definition->fields())
    {
      if (field.isConstant() == false)
      {
        // Let's add first a child to string_node
        string_node->addChild(field.name());
        StringTreeNode* new_string_node = &(string_node->children().back());
        if (field.isArray())
        {
          new_string_node->children().reserve(1);
          new_string_node = new_string_node->addChild("#");
        }

        const ROSMessage* next_msg = nullptr;
        // builtin types will not trigger a recursion
        if (field.type().isBuiltin() == false)
        {
          next_msg = getMessageByType(field.type(), info);
          if (next_msg == nullptr)
          {
            throw std::runtime_error("This type was not registered ");
          }
          msg_node->addChild(next_msg);
          MessageTreeNode* new_msg_node = &(msg_node->children().back());
          recursiveTreeCreator(next_msg, new_string_node, new_msg_node);
        }
      }  // end of field.isConstant()
    }    // end of for fields
  };     // end of lambda

  info.string_tree.root()->setValue(type_name);
  info.message_tree.root()->setValue(&info.type_list.front());
  // TODO info.type_tree.root()->value() =
  // start recursion
  recursiveTreeCreator(&info.type_list.front(), info.string_tree.root(), info.message_tree.root());
}

inline bool operator==(const std::string& a, const boost::string_ref& b)
{
  return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
}

inline bool FindPattern(const std::vector<boost::string_ref>& pattern, size_t index, const StringTreeNode* tail,
                        const StringTreeNode** head)
{
  if (tail->value() == pattern[index])
  {
    index++;
  }
  else
  {  // mismatch
    if (index > 0)
    {
      // reset counter;
      FindPattern(pattern, 0, tail, head);
      return false;
    }
    index = 0;
  }

  if (index == pattern.size())
  {
    *head = (tail);
    return true;
  }

  bool found = false;

  for (auto& child : tail->children())
  {
    found = FindPattern(pattern, index, &child, head);
    if (found)
      break;
  }
  return found;
}

void Parser::registerRenamingRules(const ROSType& type, const std::vector<SubstitutionRule>& given_rules)
{
  std::unordered_set<SubstitutionRule>& rule_set = _registered_rules[type];
  for (const auto& rule : given_rules)
  {
    if (rule_set.find(rule) == rule_set.end())
    {
      rule_set.insert(rule);
      _rule_cache_dirty = true;
    }
  }
}

void Parser::updateRuleCache()
{
  if (_rule_cache_dirty == false)
  {
    return;
  }
  else
  {
    _rule_cache_dirty = false;
  }
  for (const auto& rule_it : _registered_rules)
  {
    const ROSType& type = rule_it.first;
    const std::unordered_set<SubstitutionRule>& rule_set = rule_it.second;

    for (const auto& msg_it : _registered_messages)
    {
      const std::string& msg_identifier = msg_it.first;
      const ROSMessageInfo& msg_info = msg_it.second;

      if (getMessageByType(type, msg_info))
      {
        std::vector<RulesCache>& cache_vector = _rule_caches[msg_identifier];
        for (const auto& rule : rule_set)
        {
          RulesCache cache(rule);
          FindPattern(cache.rule->pattern(), 0, msg_info.string_tree.croot(), &cache.pattern_head);
          FindPattern(cache.rule->alias(), 0, msg_info.string_tree.croot(), &cache.alias_head);
          if (cache.pattern_head && cache.alias_head &&
              std::find(cache_vector.begin(), cache_vector.end(), cache) == cache_vector.end())
          {
            cache_vector.push_back(std::move(cache));
          }
        }
      }
    }
  }
}

void Parser::registerMessageDefinition(const std::string& msg_definition, const ROSType& main_type,
                                       const std::string& definition)
{
  if (_registered_messages.count(msg_definition) > 0)
  {
    return;  // already registered
  }
  _rule_cache_dirty = true;

  static const boost::regex msg_separation_regex("^\\s*=+\\n+");

  std::vector<std::string> split;
  std::vector<const ROSType*> all_types;

  boost::split_regex(split, definition, msg_separation_regex);

  ROSMessageInfo info;
  info.type_list.reserve(split.size());

  for (size_t i = 0; i < split.size(); ++i)
  {
    ROSMessage msg(split[i]);
    if (i == 0)
    {
      msg.mutateType(main_type);
    }

    info.type_list.push_back(std::move(msg));
    all_types.push_back(&(info.type_list.back().type()));
  }

  for (ROSMessage& msg : info.type_list)
  {
    msg.updateMissingPkgNames(all_types);
  }
  //------------------------------

  createTrees(info, msg_definition);

  //  std::cout << info.string_tree << std::endl;
  //  std::cout << info.message_tree << std::endl;
  _registered_messages.insert(std::make_pair(msg_definition, std::move(info)));
}

const ROSMessageInfo* Parser::getMessageInfo(const std::string& msg_identifier) const
{
  auto it = _registered_messages.find(msg_identifier);
  if (it != _registered_messages.end())
  {
    return &(it->second);
  }
  return nullptr;
}

const ROSMessage* Parser::getMessageByType(const ROSType& type, const ROSMessageInfo& info) const
{
  for (const ROSMessage& msg : info.type_list)  // find in the list
  {
    if (msg.type() == type)
    {
      return &msg;
    }
  }
  return nullptr;
}

void Parser::applyVisitorToBuffer(const std::string& msg_identifier, const ROSType& monitored_type,
                                  Span<uint8_t>& buffer, Parser::VisitingCallback callback) const
{
  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  if (msg_info == nullptr)
  {
    throw std::runtime_error("extractField: msg_identifier not registered. Use registerMessageDefinition");
  }
  if (getMessageByType(monitored_type, *msg_info) == nullptr)
  {
    // you will not find it. Skip it;
    return;
  }

  std::function<void(const MessageTreeNode*)> recursiveImpl;
  size_t buffer_offset = 0;

  recursiveImpl = [&](const MessageTreeNode* msg_node) {
    const ROSMessage* msg_definition = msg_node->value();
    const ROSType& msg_type = msg_definition->type();

    const bool matching = (msg_type == monitored_type);

    uint8_t* prev_buffer_ptr = buffer.data() + buffer_offset;
    size_t prev_offset = buffer_offset;

    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields())
    {
      if (field.isConstant())
        continue;

      const ROSType& field_type = field.type();

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }

      //------------------------------------

      if (field_type.isBuiltin())
      {
        for (int i = 0; i < array_size; i++)
        {
          // Skip
          ReadFromBufferToVariant(field_type.typeID(), buffer, buffer_offset);
        }
      }
      else
      {
        // field_type.typeID() == OTHER
        for (int i = 0; i < array_size; i++)
        {
          recursiveImpl(msg_node->child(index_m));
        }
        index_m++;
      }
    }  // end for fields
    if (matching)
    {
      Span<uint8_t> view(prev_buffer_ptr, buffer_offset - prev_offset);
      callback(monitored_type, view);
    }
  };  // end lambda

  // start recursion
  recursiveImpl(msg_info->message_tree.croot());
}

template <typename Container>
inline void ExpandVectorIfNecessary(Container& container, size_t new_size)
{
  if (container.size() <= new_size)
  {
    const size_t increased_size = std::max(size_t(32), container.size() * 2);
    container.resize(increased_size);
  }
}

bool Parser::deserializeIntoFlatContainer(const std::string& msg_identifier, Span<uint8_t> buffer,
                                          FlatMessage* flat_container, const uint32_t max_array_size) const
{
  bool entire_message_parse = true;
  const ROSMessageInfo* msg_info = getMessageInfo(msg_identifier);

  size_t value_index = 0;
  size_t name_index = 0;
  size_t blob_index = 0;
  size_t blob_storage_index = 0;

  if (msg_info == nullptr)
  {
    throw std::runtime_error("deserializeIntoFlatContainer: msg_identifier not registerd. Use "
                             "registerMessageDefinition");
  }
  size_t buffer_offset = 0;

  std::function<void(const MessageTreeNode*, StringTreeLeaf, bool)> deserializeImpl;

  deserializeImpl = [&](const MessageTreeNode* msg_node, const StringTreeLeaf& tree_leaf, bool store) {
    const ROSMessage* msg_definition = msg_node->value();
    size_t index_s = 0;
    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields())
    {
      bool DO_STORE = store;
      if (field.isConstant())
        continue;

      const ROSType& field_type = field.type();

      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index_s);

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }
      if (field.isArray())
      {
        new_tree_leaf.index_array.push_back(0);
        new_tree_leaf.node_ptr = new_tree_leaf.node_ptr->child(0);
      }

      bool IS_BLOB = false;

      // Stop storing it if is NOT a blob and a very large array.
      if (array_size > static_cast<int32_t>(max_array_size))
      {
        if (builtinSize(field_type.typeID()) == 1)
        {
          IS_BLOB = true;
        }
        else
        {
          if (_discard_large_array)
          {
            DO_STORE = false;
          }
          entire_message_parse = false;
        }
      }

      if (IS_BLOB)  // special case. This is a "blob", typically an image, a map, pointcloud, etc.
      {
        ExpandVectorIfNecessary(flat_container->blob, blob_index);

        if (buffer_offset + array_size > static_cast<std::size_t>(buffer.size()))
        {
          throw std::runtime_error("Buffer overrun in deserializeIntoFlatContainer (blob)");
        }
        if (DO_STORE)
        {
          flat_container->blob[blob_index].first = new_tree_leaf;
          auto& blob = flat_container->blob[blob_index].second;
          blob_index++;

          if (_blob_policy == STORE_BLOB_AS_COPY)
          {
            ExpandVectorIfNecessary(flat_container->blob_storage, blob_storage_index);

            auto& storage = flat_container->blob_storage[blob_storage_index];
            storage.resize(array_size);
            std::memcpy(storage.data(), &buffer[buffer_offset], array_size);
            blob_storage_index++;

            blob = Span<uint8_t>(storage.data(), storage.size());
          }
          else
          {
            blob = Span<uint8_t>(&buffer[buffer_offset], array_size);
          }
        }
        buffer_offset += array_size;
      }
      else  // NOT a BLOB
      {
        bool DO_STORE_ARRAY = DO_STORE;
        for (int i = 0; i < array_size; i++)
        {
          if (DO_STORE_ARRAY && i >= static_cast<int32_t>(max_array_size))
          {
            DO_STORE_ARRAY = false;
          }

          if (field.isArray() && DO_STORE_ARRAY)
          {
            new_tree_leaf.index_array.back() = i;
          }

          if (field_type.typeID() == STRING)
          {
            ExpandVectorIfNecessary(flat_container->name, name_index);

            uint32_t string_size = 0;
            ReadFromBuffer(buffer, buffer_offset, string_size);

            if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
            {
              throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
            }

            if (DO_STORE_ARRAY)
            {
              if (string_size == 0)
              {
                // corner case, when there is an empty string at the end of the message
                flat_container->name[name_index].second.clear();
              }
              else
              {
                const char* buffer_ptr = reinterpret_cast<const char*>(buffer.data() + buffer_offset);
                flat_container->name[name_index].second.assign(buffer_ptr, string_size);
              }
              flat_container->name[name_index].first = new_tree_leaf;
              name_index++;
            }
            buffer_offset += string_size;
          }
          else if (field_type.isBuiltin())
          {
            ExpandVectorIfNecessary(flat_container->value, value_index);

            Variant var = ReadFromBufferToVariant(field_type.typeID(), buffer, buffer_offset);
            if (DO_STORE_ARRAY)
            {
              flat_container->value[value_index] = std::make_pair(new_tree_leaf, std::move(var));
              value_index++;
            }
          }
          else
          {  // field_type.typeID() == OTHER

            deserializeImpl(msg_node->child(index_m), new_tree_leaf, DO_STORE_ARRAY);
          }
        }  // end for array_size
      }

      if (field_type.typeID() == OTHER)
      {
        index_m++;
      }
      index_s++;
    }  // end for fields
  };   // end of lambda

  flat_container->tree = &msg_info->string_tree;

  StringTreeLeaf rootnode;
  rootnode.node_ptr = msg_info->string_tree.croot();

  deserializeImpl(msg_info->message_tree.croot(), rootnode, true);

  flat_container->name.resize(name_index);
  flat_container->value.resize(value_index);
  flat_container->blob.resize(blob_index);
  flat_container->blob_storage.resize(blob_storage_index);

  // WORKAROUNG: messages created with ROS serial might have 1 extra byte :(
  if (buffer.size() - buffer_offset > 1)
  {
    char msg_buff[1000];
    sprintf(msg_buff,
            "buildRosFlatType: There was an error parsing the buffer.\n"
            "Size %d != %d, while parsing [%s]",
            (int)buffer_offset, (int)buffer.size(), msg_identifier.c_str());

    throw std::runtime_error(msg_buff);
  }
  return entire_message_parse;
}

inline bool isNumberPlaceholder(const boost::string_ref& s)
{
  return s.size() == 1 && s[0] == '#';
}

inline bool isSubstitutionPlaceholder(const boost::string_ref& s)
{
  return s.size() == 1 && s[0] == '@';
}

// given a leaf of the tree, that can have multiple index_array,
// find the only index which corresponds to the # in the pattern
inline int PatternMatchAndIndexPosition(const StringTreeLeaf& leaf, const StringTreeNode* pattern_head)
{
  const StringTreeNode* node_ptr = leaf.node_ptr;

  int pos = leaf.index_array.size() - 1;

  while (node_ptr)
  {
    if (node_ptr != pattern_head)
    {
      if (isNumberPlaceholder(node_ptr->value()))
      {
        pos--;
      }
    }
    else
    {
      return pos;
    }
    node_ptr = node_ptr->parent();
  }  // end while
  return -1;
}

template <typename VectorType>
inline void JoinStrings(const VectorType& vect, const char separator, std::string& destination)
{
  size_t count = 0;
  for (const auto& v : vect)
    count += v.size();

  // the following approach seems to be faster
  // https://github.com/facontidavide/InterestingBenchmarks/blob/master/StringAppend_vs_Memcpy.md

  destination.resize(count + vect.size() - 1);

  char* buffer = &destination[0];
  size_t buff_pos = 0;

  for (size_t c = 0; c < vect.size() - 1; c++)
  {
    const size_t S = vect[c].size();
    std::memcpy(&buffer[buff_pos], vect[c].data(), S);
    buff_pos += S;
    buffer[buff_pos++] = separator;
  }
  std::memcpy(&buffer[buff_pos], vect.back().data(), vect.back().size());
}

void Parser::applyNameTransform(const std::string& msg_identifier, const FlatMessage& container,
                                RenamedValues* renamed_value, bool skip_topicname)
{
  if (_rule_cache_dirty)
  {
    updateRuleCache();
  }
  auto rule_found = _rule_caches.find(msg_identifier);

  const size_t num_values = container.value.size();
  const size_t num_names = container.name.size();

  renamed_value->resize(container.value.size());
  // DO NOT clear() renamed_value

  _alias_array_pos.resize(num_names);
  _formatted_string.reserve(num_values);
  _formatted_string.clear();

  _substituted.resize(num_values);
  for (size_t i = 0; i < num_values; i++)
  {
    _substituted[i] = false;
  }

  // size_t renamed_index = 0;

  if (rule_found != _rule_caches.end())
  {
    const std::vector<RulesCache>& rules_cache = rule_found->second;

    for (const RulesCache& cache : rules_cache)
    {
      const SubstitutionRule* rule = cache.rule;
      const StringTreeNode* pattern_head = cache.pattern_head;
      const StringTreeNode* alias_head = cache.alias_head;

      if (!pattern_head || !alias_head)
        continue;

      for (size_t n = 0; n < num_names; n++)
      {
        const StringTreeLeaf& name_leaf = container.name[n].first;
        _alias_array_pos[n] = PatternMatchAndIndexPosition(name_leaf, alias_head);
      }

      for (size_t value_index = 0; value_index < num_values; value_index++)
      {
        if (_substituted[value_index])
          continue;

        const auto& value_leaf = container.value[value_index];

        const StringTreeLeaf& leaf = value_leaf.first;

        int pattern_array_pos = PatternMatchAndIndexPosition(leaf, pattern_head);

        if (pattern_array_pos >= 0)  // -1 if pattern doesn't match
        {
          boost::string_ref new_name;

          for (size_t n = 0; n < num_names; n++)
          {
            const auto& it = container.name[n];
            const StringTreeLeaf& alias_leaf = it.first;

            if (_alias_array_pos[n] >= 0)  // -1 if pattern doesn't match
            {
              if (alias_leaf.index_array[_alias_array_pos[n]] == leaf.index_array[pattern_array_pos])
              {
                new_name = it.second;
                break;
              }
            }
          }

          //--------------------------
          if (!new_name.empty())
          {
            boost::container::static_vector<boost::string_ref, 12> concatenated_name;

            const StringTreeNode* node_ptr = leaf.node_ptr;

            int position = leaf.index_array.size() - 1;

            while (node_ptr != pattern_head)
            {
              const boost::string_ref& str_val = node_ptr->value();

              if (isNumberPlaceholder(str_val))
              {
                char buffer[16];
                const int number = leaf.index_array[position--];
                int str_size = print_number(buffer, number);
                _formatted_string.push_back(std::string(buffer, str_size));
                concatenated_name.push_back(_formatted_string.back());
              }
              else
              {
                concatenated_name.push_back(str_val);
              }
              node_ptr = node_ptr->parent();
            }

            for (int s = rule->substitution().size() - 1; s >= 0; s--)
            {
              const boost::string_ref& str_val = rule->substitution()[s];

              if (isSubstitutionPlaceholder(str_val))
              {
                concatenated_name.push_back(new_name);
                position--;
              }
              else
              {
                concatenated_name.push_back(str_val);
              }
            }

            for (size_t p = 0; p < rule->pattern().size() && node_ptr; p++)
            {
              node_ptr = node_ptr->parent();
            }

            while (node_ptr)
            {
              boost::string_ref str_val = node_ptr->value();

              if (isNumberPlaceholder(str_val))
              {
                char buffer[16];
                const int number = leaf.index_array[position--];
                int str_size = print_number(buffer, number);
                _formatted_string.push_back(std::string(buffer, str_size));
                concatenated_name.push_back(_formatted_string.back());
              }
              else
              {
                concatenated_name.push_back(str_val);
              }
              node_ptr = node_ptr->parent();
            }

            //------------------------
            auto& renamed_pair = (*renamed_value)[value_index];

            if (skip_topicname)
            {
              concatenated_name.pop_back();
            }

            std::reverse(concatenated_name.begin(), concatenated_name.end());
            JoinStrings(concatenated_name, '/', renamed_pair.first);
            renamed_pair.second = value_leaf.second;

            _substituted[value_index] = true;

          }  // end if( new_name )
        }    // end if( PatternMatching )
      }      // end for values
    }        // end for rules
  }          // end rule found

  for (size_t value_index = 0; value_index < container.value.size(); value_index++)
  {
    if (!_substituted[value_index])
    {
      const std::pair<StringTreeLeaf, Variant>& value_leaf = container.value[value_index];

      std::string& destination = (*renamed_value)[value_index].first;
      CreateStringFromTreeLeaf(value_leaf.first, skip_topicname, destination);
      (*renamed_value)[value_index].second = value_leaf.second;
    }
  }
}

}  // namespace RosIntrospection
