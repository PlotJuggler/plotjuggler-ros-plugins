#pragma once

#include "fmt/format.h"
#include "ros1_parser.h"

template <size_t N>
class CovarianceParser
{
public:
  CovarianceParser(const std::string& prefix, PJ::PlotDataMapRef& plot_data)
  {
    for (int i = 0; i < N; i++)
    {
      for (int j = i; j < N; j++)
      {

        auto key = fmt::format("{}[{};{}]", prefix, i, j);

        auto plot_pair = plot_data.numeric.find(key);
        if (plot_pair == plot_data.numeric.end())
        {
          plot_pair = plot_data.addNumeric(key);
        }
        auto plot_data = &(plot_pair->second);
        _data.push_back(plot_data);
      }
    }
  }

  void parse(const boost::array<double, N * N>& covariance, double& timestamp)
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
