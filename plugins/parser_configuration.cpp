#include "parser_configuration.h"
#include <boost/spirit/include/qi.hpp>
#include <boost/algorithm/string.hpp>

namespace PJ
{

void RosParserConfig::xmlSaveState(QDomDocument &doc, QDomElement &parent_elem) const
{
  QDomElement stamp_elem = doc.createElement("use_header_stamp");
  stamp_elem.setAttribute("value", use_header_stamp ? "true" : "false");
  parent_elem.appendChild(stamp_elem);

  QDomElement discard_elem = doc.createElement("discard_large_arrays");
  discard_elem.setAttribute("value", discard_large_arrays ? "true" : "false");
  parent_elem.appendChild(discard_elem);

  QDomElement max_elem = doc.createElement("max_array_size");
  max_elem.setAttribute("value", QString::number(max_array_size));
  parent_elem.appendChild(max_elem);

  QDomElement bool_elem = doc.createElement("boolean_strings_to_number");
  bool_elem.setAttribute("value", boolean_strings_to_number ? "true" : "false");
  parent_elem.appendChild(bool_elem);

  QDomElement suffix_elem = doc.createElement("remove_suffix_from_strings");
  suffix_elem.setAttribute("value", remove_suffix_from_strings ? "true" : "false");
  parent_elem.appendChild(suffix_elem);
}

void RosParserConfig::xmlLoadState(const QDomElement &parent_element)
{
  QDomElement stamp_elem = parent_element.firstChildElement("use_header_stamp");
  use_header_stamp = (stamp_elem.attribute("value") == "true");

  QDomElement discard_elem = parent_element.firstChildElement("discard_large_arrays");
  discard_large_arrays = (discard_elem.attribute("value") == "true");

  QDomElement max_elem = parent_element.firstChildElement("max_array_size");
  max_array_size = max_elem.attribute("value").toInt();

  QDomElement bool_elem = parent_element.firstChildElement("boolean_strings_to_number");
  boolean_strings_to_number = (bool_elem.attribute("value") == "true");

  QDomElement suffix_elem = parent_element.firstChildElement("remove_suffix_from_strings");
  remove_suffix_from_strings = (suffix_elem.attribute("value")== "true");
}

void RosParserConfig::saveToSettings(QSettings &settings, QString prefix) const
{
  settings.setValue(prefix + "/default_topics", topics);
  settings.setValue(prefix + "/use_header_stamp", use_header_stamp);
  settings.setValue(prefix + "/max_array_size", (int)max_array_size);
  settings.setValue(prefix + "/discard_large_arrays", discard_large_arrays);
  settings.setValue(prefix + "/boolean_strings_to_number", boolean_strings_to_number);
  settings.setValue(prefix + "/remove_suffix_from_strings", remove_suffix_from_strings);
}

void RosParserConfig::loadFromSettings(const QSettings &settings, QString prefix)
{
  topics = settings.value(prefix + "/default_topics", false).toStringList();
  use_header_stamp = settings.value(prefix + "/use_header_stamp", false).toBool();
  max_array_size = settings.value(prefix + "/max_array_size", 100).toInt();
  discard_large_arrays = settings.value(prefix + "/discard_large_arrays", true).toBool();
  boolean_strings_to_number = settings.value(prefix + "/boolean_strings_to_number", true).toBool();
  remove_suffix_from_strings = settings.value(prefix + "/remove_suffix_from_strings", true).toBool();
}

CompositeParser::CompositeParser(PlotDataMapRef &plot_data): _plot_data(plot_data)
{
}

const RosParserConfig &CompositeParser::getConfig()
{
  return _config;
}

void CompositeParser::setConfig(const RosParserConfig &config)
{
  _config = config;
  // we don't need this information.
  _config.topics.clear();

  for(auto& [name, parser]: _parsers)
  {
    parser->setConfig(_config);
  }
}

bool CompositeParser::parseMessage(const std::string& topic_name, MessageRef serialized_msg, double& timestamp)
{
  auto it = _parsers.find(topic_name);
  if (it == _parsers.end())
  {
    return false;
  }
  auto&parser = it->second;
  parser->parseMessage(serialized_msg, timestamp);
  return true;
}

RosMessageParser::RosMessageParser(const std::string &topic_name, PlotDataMapRef &plot_data):
  MessageParser(topic_name, plot_data)
{
}

const RosParserConfig &RosMessageParser::getConfig() const
{
  return _config;
}

void RosMessageParser::setConfig(const RosParserConfig &config)
{
  _config = config;
}

PlotData &RosMessageParser::getSeries(const std::string& key)
{
  return _plot_data.getOrCreateNumeric(key);
}

StringSeries &RosMessageParser::getStringSeries(const std::string& key)
{
  return _plot_data.getOrCreateStringSeries(key);
}

bool ParseDouble(const std::string &str,
                 double &value,
                 bool remover_suffix,
                 bool parse_boolean)
{
  bool parsed = boost::spirit::qi::parse(str.data(), str.data() + str.size(),
                                         boost::spirit::qi::double_, value);

  if(!parsed && remover_suffix)
  {
    int pos = 0;
    while(pos < str.size())
    {
      char c = str[pos];
      if( !std::isdigit(c) && c!= '-' && c!= '+' && c!='.')
      {
        break;
      }
      pos++;
    }
    if( pos < str.size() )
    {
      parsed = boost::spirit::qi::parse(str.data(), str.data() + pos,
                                        boost::spirit::qi::double_, value);
    }
  }

  if(!parsed && parse_boolean && str.size() >=4 && str.size() <=5)
  {
    std::string temp = str;
    boost::algorithm::to_lower(temp);
    if( temp == "true" )
    {
      parsed = true;
      value = 1;
    }
    else if( temp == "false" )
    {
      parsed = true;
      value = 0;
    }
  }

  return parsed;
}

}
