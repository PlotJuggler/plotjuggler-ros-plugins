#pragma once

#include "fmt/format.h"
#include "ros1_parser.h"

template <size_t N>
class CovarianceParser
{
public:
  CovarianceParser(const std::string& prefix, PJ::PlotDataMapRef& plot_data)
  {
    auto plot_data_ptr = &plot_data;
    _lazy_init = [=]()
    {
      for (int i = 0; i < N; i++)
      {
        for (int j = i; j < N; j++)
        {
          auto key = fmt::format("{}[{};{}]", prefix, i, j);
          _data.push_back( &plot_data_ptr->getOrCreateNumeric(key) );
        }
      }
    };
  }

  void parse(const boost::array<double, N * N>& covariance, double& timestamp)
  {
    if( !_initialized )
    {
      _initialized = true;
      _lazy_init();
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
  std::function<void()> _lazy_init;
  bool _initialized = false;
};
