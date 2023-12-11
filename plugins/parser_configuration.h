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


class RosMessageParser
{
  public:
  RosMessageParser(PlotDataMapRef &plot_data);

  PJ::PlotData& getSeries(const std::string& key);

  PJ::StringSeries& getStringSeries(const std::string &key);

  virtual bool parseMessage(MessageRef serialized_msg, double& timestamp) = 0;

  protected:

  PJ::PlotDataMapRef& _plot_data;
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

  std::unordered_map<std::string, std::shared_ptr<RosMessageParser>> _parsers;
  RosParserConfig _config;
  PJ::PlotDataMapRef& _plot_data;
};

bool ParseDouble(const std::string& str, double& value, bool remover_suffix, bool parse_boolean);


}

#endif // PARSER_CONFIGURATION_H
