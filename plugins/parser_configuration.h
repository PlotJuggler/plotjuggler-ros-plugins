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

// base class for the parser
class RosMessageParser: public MessageParser
{
  public:
  RosMessageParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data);

  const RosParserConfig& getConfig() const;

  void setConfig(const RosParserConfig& config);

  PJ::PlotData& getSeries(const std::string& key);

  PJ::StringSeries& getStringSeries(const std::string &key);

  protected:

  RosParserConfig _config;
};

// aggregator front-end to many parsers
class CompositeParser
{
  public:
  CompositeParser(PlotDataMapRef& plot_data);

  const RosParserConfig& getConfig();

  void setConfig(const RosParserConfig& config);

  bool parseMessage(const std::string& topic_name, MessageRef serialized_msg, double& timestamp);

  protected:

  std::map<std::string, std::shared_ptr<RosMessageParser>> _parsers;
  RosParserConfig _config;
  PJ::PlotDataMapRef& _plot_data;
};

bool ParseDouble(const std::string& str, double& value, bool remover_suffix, bool parse_boolean);


}

#endif // PARSER_CONFIGURATION_H
