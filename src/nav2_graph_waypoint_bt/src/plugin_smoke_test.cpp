#include <iostream>
#include <string>
#include "behaviortree_cpp/bt_factory.h"

int main(int argc, char ** argv) {
  if (argc != 2) {
    std::cerr << "Usage: plugin_smoke_test /absolute/path/libnav2_graph_waypoint_bt.so\n";
    return 2;
  }
  try {
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(argv[1]);
    const auto & types = factory.manifests();
    if (types.find("PruneGraphGoals") == types.end() ||
        types.find("TurnSegmentControl") == types.end()) {
      std::cerr << "Plugin loaded but expected BT node registrations were missing\n";
      return 1;
    }
    std::cout << "PASS: PruneGraphGoals and TurnSegmentControl are registered\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Plugin load failed: " << e.what() << '\n';
    return 1;
  }
}
