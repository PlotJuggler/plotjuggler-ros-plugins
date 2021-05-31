#pragma once

#include "fmt/format.h"
#include "ros2_parser.h"

using namespace Ros2Introspection;

template <size_t N>
class CovarianceParser
{
public:
  CovarianceParser(const std::string& prefix, PJ::PlotDataMapRef& plot_data)
  {
    int index = 0;
    for (int i = 0; i < N; i++)
    {
      for (int j = i; j < N; j++)
      {
        auto str = fmt::format("{}[{};{}]", prefix, i, j);
        _data.push_back(&RosMessageParser::getSeries(plot_data, str));
      }
    }
  }

  void parse(const std::array<double, N * N>& covariance, double& timestamp)
  {
    size_t index = 0;
    for (int i = 0; i < N; i++)
    {
      for (int j = i; j < N; j++)
      {
        _data[index++]->pushBack({ timestamp, covariance[i * N + j] });
      }
    }
  }

private:
  std::vector<PJ::PlotData*> _data;
};
