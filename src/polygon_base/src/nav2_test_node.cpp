#include <cstdio>
#include <pluginlib/class_loader.hpp>
#include "nav2_core/global_planner.hpp"

int main(int argc, char ** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> p_loader("nav2_core", "nav2_core::GlobalPlanner");

  try
  {
    std::shared_ptr<nav2_core::GlobalPlanner> planner = p_loader.createSharedInstance("full_coverage_path_planner/SpiralSTC");
    // printf("Triangle area: %.2f\n", triangle->area());
    // printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
