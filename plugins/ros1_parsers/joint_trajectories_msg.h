#pragma once

#include <trajectory_msgs/JointTrajectory.h>
#include "ros1_parser.h"
#include "header_msg.h"

class JointTrajectoryParser : public BuiltinMessageParser<trajectory_msgs::JointTrajectory>
{
public:
  JointTrajectoryParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<trajectory_msgs::JointTrajectory>(topic_name, plot_data)
    , _header_parser(topic_name + "/header", plot_data)
  {
  }

  void parseMessageImpl(const trajectory_msgs::JointTrajectory& msg, double& timestamp) override
  {
    _header_parser.parse(msg.header, timestamp, _use_header_stamp);

    _positions.clear();
    _velocities.clear();
    _accelerations.clear();
    _effort.clear();

    for ( int j = 0; j < msg.joint_names.size(); j++ )
    {
      const std::string prefix = _topic_name + "/" + msg.joint_names[j];
      _positions.push_back( &getSeries(prefix + "/positions") );
      _velocities.push_back( &getSeries(prefix + "/velocities") );
      _accelerations.push_back( &getSeries(prefix + "/accelerations") );
      _effort.push_back( &getSeries(prefix + "/effort") );
    }

    double first_ts = timestamp;

    for (const auto point: msg.points)
    {
      timestamp = first_ts + point.time_from_start.toSec();

      for( int j = 0; j < point.positions.size(); j++ )
      {
        _positions[j]->pushBack( {timestamp, point.positions[j]} );
      }

      for( int j = 0; j < point.velocities.size(); j++ )
      {
        _velocities[j]->pushBack( {timestamp, point.velocities[j]} );
      }

      for( int j = 0; j < point.accelerations.size(); j++ )
      {
        _accelerations[j]->pushBack( {timestamp, point.accelerations[j]} );
      }

      for( int j = 0; j < point.effort.size(); j++ )
      {
        _effort[j]->pushBack( {timestamp, point.effort[j]} );
      }
    }
  }

private:
  HeaderMsgParser _header_parser;

  std::vector<PlotData*> _positions;
  std::vector<PlotData*> _velocities;
  std::vector<PlotData*> _accelerations;
  std::vector<PlotData*> _effort;
};
