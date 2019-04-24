#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>

#include <string>
#include <vector>

namespace search_planners {

using geometry_msgs::PoseStamped;
using std::vector;
using std::string;
using costmap_2d::Costmap2DROS;

class DepthFirstSearch : public nav_core::BaseGlobalPlanner {

 public:
  virtual ~DepthFirstSearch() = default;

  virtual bool makePlan(
      const PoseStamped& start,
      const PoseStamped& goal,
      vector<PoseStamped>& plan) override;

  virtual void initialize(
      string name,
      Costmap2DROS* costmap_ros) override;

};  // end DepthFirstSearch class


}  // end namespace