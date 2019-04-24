#include "search_planners/depth_first_search.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(search_planners::DepthFirstSearch, nav_core::BaseGlobalPlanner)

namespace search_planners {

bool DepthFirstSearch::makePlan(
    const PoseStamped& start,
    const PoseStamped& goal,
    vector<PoseStamped>& plan) {

  return false;

}

void DepthFirstSearch::initialize(
    string name,
    Costmap2DROS* costmap_ros) {}


}  // end namespace