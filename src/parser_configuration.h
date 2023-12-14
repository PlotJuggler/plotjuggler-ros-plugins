#ifndef PJ_PARSER_CONFIGURATION_H
#define PJ_PARSER_CONFIGURATION_H

#include <QStringList>
#include <QSettings>
#include <QDomDocument>

#include <PlotJuggler/plotdata.h>
#include <PlotJuggler/messageparser_base.h>

namespace PJ
{

struct RosParserConfig
{
  QStringList topics;
  unsigned max_array_size = 999;
  bool use_header_stamp = false;
  bool discard_large_arrays = false;
  bool boolean_strings_to_number = false;
  bool remove_suffix_from_strings = false;

  void xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const;
  void xmlLoadState(const QDomElement& parent_element);

  void saveToSettings(QSettings& setting, QString prefix) const;
  void loadFromSettings(const QSettings& settings, QString prefix);
};


// aggregator front-end to many parsers
class CompositeParser
{
  public:

  void addParser(const std::string& topic_name, std::shared_ptr<PJ::MessageParser> parser);

  const RosParserConfig& getConfig();

  void setConfig(const RosParserConfig& config);

  bool parseMessage(const std::string& topic_name, MessageRef serialized_msg, double& timestamp);

  void clear() { _parsers.clear(); }

  bool hasParser(const std::string& topic_name) { return _parsers.count(topic_name) != 0; }

  protected:

  std::unordered_map<std::string, std::shared_ptr<PJ::MessageParser>> _parsers;
  RosParserConfig _config;
};

bool ParseDouble(const std::string& str, double& value, bool remover_suffix, bool parse_boolean);


}

#endif // PARSER_CONFIGURATION_H
