#pragma once

#include <moveit_msgs/DisplayTrajectory.h>
#include "ros1_parser.h"
#include "joint_trajectories_msg.h"
#include "header_msg.h"

class Moveit_DisplayTrajectoryParser : public BuiltinMessageParser<moveit_msgs::DisplayTrajectory>
{
public:
  Moveit_DisplayTrajectoryParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data)
    : BuiltinMessageParser<moveit_msgs::DisplayTrajectory>(topic_name, plot_data)
    ,_joint_trajectory( topic_name + "/trajectory", plot_data )
  {
  }

  void parseMessageImpl(const moveit_msgs::DisplayTrajectory& msg, double& timestamp) override
  {
    for(const auto& trajectory: msg.trajectory)
    {
      _joint_trajectory.parseMessageImpl( trajectory.joint_trajectory, timestamp );
    }
  }

private:

  JointTrajectoryParser _joint_trajectory;
};
