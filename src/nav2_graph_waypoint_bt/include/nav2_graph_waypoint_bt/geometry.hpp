#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace graph_bt {

struct Point2 { double x; double y; };
constexpr double kPi = 3.14159265358979323846;

inline double distance(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline double heading(Point2 a, Point2 b) {
  return std::atan2(b.y - a.y, b.x - a.x);
}
inline double turnAngleDegrees(Point2 a, Point2 b, Point2 c) {
  const double in = heading(a, b);
  const double out = heading(b, c);
  return std::abs(std::atan2(std::sin(out - in), std::cos(out - in))) * 180.0 / kPi;
}

struct Passage {
  double progress_m;  // >= 0 means at / beyond plane through waypoint
  double lateral_m;
  double distance_m;
  bool valid;
};

inline Passage measurePassage(Point2 anchor, Point2 goal, Point2 robot) {
  const double dx = goal.x - anchor.x;
  const double dy = goal.y - anchor.y;
  const double length = std::hypot(dx, dy);
  const double d = distance(robot, goal);
  if (length < 0.025) { return {0.0, d, d, false}; }
  const double ux = dx / length, uy = dy / length;
  const double rx = robot.x - goal.x, ry = robot.y - goal.y;
  return {rx * ux + ry * uy, std::abs(rx * uy - ry * ux), d, true};
}

inline bool shouldPrune(Passage p, double near_radius, double lateral_tol,
                        double max_pass_distance, double pass_margin) {
  if (!p.valid || p.lateral_m > lateral_tol) { return false; }
  const bool at_goal = p.distance_m <= near_radius && p.progress_m >= -0.005;
  const bool passed = p.progress_m >= pass_margin &&
    p.distance_m <= max_pass_distance;
  return at_goal || passed;
}

// Groups waypoint indexes into non-overlapping segments, ending at turn nodes.
// An optional incoming position detects a first-waypoint turn.
inline std::vector<std::pair<size_t, size_t>> splitAtTurns(
    const std::vector<Point2> & goals, double min_turn_deg, double min_leg_m,
    const Point2 * incoming = nullptr) {
  std::vector<std::pair<size_t, size_t>> segments;
  if (goals.empty()) {return segments;}
  size_t start = 0;
  if (incoming != nullptr && goals.size() >= 2 &&
      distance(*incoming, goals[0]) >= min_leg_m &&
      distance(goals[0], goals[1]) >= min_leg_m &&
      turnAngleDegrees(*incoming, goals[0], goals[1]) >= min_turn_deg) {
    segments.emplace_back(0, 0);
    start = 1;
  }
  for (size_t i = 1; i + 1 < goals.size(); ++i) {
    if (distance(goals[i - 1], goals[i]) >= min_leg_m &&
        distance(goals[i], goals[i + 1]) >= min_leg_m &&
        turnAngleDegrees(goals[i - 1], goals[i], goals[i + 1]) >= min_turn_deg) {
      segments.emplace_back(start, i);
      start = i + 1;
    }
  }
  segments.emplace_back(start, goals.size() - 1);
  return segments;
}

}  // namespace graph_bt
