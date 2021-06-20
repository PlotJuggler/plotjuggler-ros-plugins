#pragma once

#include "fmt/format.h"
#include "ros2_parser.h"

using namespace Ros2Introspection;

template <size_t N>
class CovarianceParser
{
public:
  CovarianceParser(const std::string& prefix, PJ::PlotDataMapRef& plot_data)
     : _prefix( prefix )
       , _plot_data( plot_data )
  {
  }

  void parse(const std::array<double, N * N>& covariance, double& timestamp)
  {
    if( !_initialized )
    {
      _initialized = true;

      int index = 0;
      for (int i = 0; i < N; i++)
      {
        for (int j = i; j < N; j++)
        {
          auto key = fmt::format("{}[{};{}]", _prefix, i, j);
          _data.push_back(  &_plot_data.getOrCreateNumeric(key) );
        }
      }
    }

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

  const std::string _prefix;
  PJ::PlotDataMapRef& _plot_data;
  bool _initialized = false;
};
