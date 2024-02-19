#pragma once

#include <PlotJuggler/messageparser_base.h>
#include <iostream>

inline std::shared_ptr<PJ::MessageParser>
CreateParserROS(const PJ::ParserFactories &factories,
                const std::string& topic_name,
                const std::string& type_name,
                const std::string& definition,
                PJ::PlotDataMapRef& data)
{
  return factories.at("ros1msg")->createParser(topic_name, type_name, definition, data);
}

