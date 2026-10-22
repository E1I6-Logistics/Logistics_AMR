#include <cassert>
#include <cmath>
#include <iostream>
#include "nav2_graph_waypoint_bt/geometry.hpp"
using namespace graph_bt;
int main() {
  assert(std::abs(turnAngleDegrees({0,0},{1,0},{2,0})) < 1e-9);
  assert(std::abs(turnAngleDegrees({0,0},{1,0},{1,1}) - 90.0) < 1e-9);
  assert(std::abs(turnAngleDegrees({0,0},{1,0},{0,0}) - 180.0) < 1e-9);
  auto early = measurePassage({0,0},{1,0},{0.96, 0.02});
  assert(!shouldPrune(early, .04, .06, .12, 0));
  auto good = measurePassage({0,0},{1,0},{1.02, 0.03});
  assert(shouldPrune(good, .04, .06, .12, 0));
  auto after = measurePassage({0,0},{1,0},{1.07, 0.02});
  assert(shouldPrune(after, .04, .06, .12, 0));
  auto far = measurePassage({0,0},{1,0},{1.2, 0.0});
  assert(!shouldPrune(far, .04, .06, .12, 0));
  auto side = measurePassage({0,0},{1,0},{1.01, 0.1});
  assert(!shouldPrune(side, .04, .06, .12, 0));
  auto degenerate = measurePassage({1,0},{1,0},{1,0});
  assert(!degenerate.valid);
  std::vector<Point2> line_turn = {{0,0}, {0.5,0}, {1,0}, {1,0.5}, {1,1}};
  auto segments = splitAtTurns(line_turn, 45.0, 0.07);
  assert(segments.size() == 2);
  assert(segments[0].first == 0 && segments[0].second == 2);
  assert(segments[1].first == 3 && segments[1].second == 4);
  std::vector<Point2> first_corner = {{1,0}, {1,1}};
  Point2 start_pose{0,0};
  auto first_segments = splitAtTurns(first_corner, 45.0, 0.07, &start_pose);
  assert(first_segments.size() == 2);
  assert(first_segments[0].first == 0 && first_segments[0].second == 0);
  assert(first_segments[1].first == 1 && first_segments[1].second == 1);
  std::vector<Point2> one = {{1,2}};
  assert(splitAtTurns(one, 45.0, 0.07).size() == 1);
  std::cout << "geometry tests passed\n";
}
