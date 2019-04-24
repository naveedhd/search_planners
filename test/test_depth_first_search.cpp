#include <catch_ros/catch.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.hpp>
#include <search_planners/depth_first_search.h>
#include <vector>


TEST_CASE("depth first search planner is constructable from base class", "[dfs]") {

  nav_core::BaseGlobalPlanner* planner;
  search_planners::DepthFirstSearch dfs;
  planner = &dfs;

  geometry_msgs::PoseStamped start, goal;
  std::vector<geometry_msgs::PoseStamped> plan;
  REQUIRE(!planner->makePlan(start, goal, plan));

}

TEST_CASE("depth first search planner is loadable from class loader", "[dfs]") {

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> loader("nav_core", "nav_core::BaseGlobalPlanner");

  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner = loader.createInstance("search_planners/DepthFirstSearch");

  geometry_msgs::PoseStamped start, goal;
  std::vector<geometry_msgs::PoseStamped> plan;
  REQUIRE(!planner->makePlan(start, goal, plan));

}