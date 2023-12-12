#pragma once

#include <PlotJuggler/plotdata.h>
#include <PlotJuggler/messageparser_base.h>
#include "ros_type_introspection/ros_introspection.hpp"
#include "parser_configuration.h"

std::shared_ptr<PJ::MessageParser>
CreateParserROS(const PJ::ParserFactories &factories,
                const std::string &topic_name,
                const std::string &type_name,
                PJ::PlotDataMapRef &data);
