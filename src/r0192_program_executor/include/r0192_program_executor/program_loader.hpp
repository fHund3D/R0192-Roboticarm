// ============================================================================
// program_loader.hpp
//
// Parsing + validation of the R0192 program/point YAML files. This is the
// ONLY place the files are parsed — the schema documented in
// doku/program_ide_plan.md (and mirrored as JSON Schema in doku/schemas/ for
// VS Code) is enforced here at runtime.
//
//   points.yaml     named targets, type joint (6 values, rad) or pose
//                   (base_link, m / quaternion)
//   program_*.yaml  ordered steps: move_j / move_l / wait. velocity and
//                   acceleration are MoveIt scaling factors in (0, 1].
// ============================================================================

#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <map>
#include <string>
#include <vector>

namespace r0192_program_executor
{

// A named target from points.yaml. Programs reference points by name only —
// inline poses in program steps are forbidden by design.
struct Point
{
  enum class Type { kJoint, kPose };

  Type type{Type::kJoint};
  std::vector<double> joints;     // kJoint: rad, order joint_1..joint_6
  geometry_msgs::msg::Pose pose;  // kPose: expressed in `frame`
  std::string frame;              // kPose: v1 accepts only "base_link"
};

using PointMap = std::map<std::string, Point>;

// One program step. For move steps, velocity/acceleration are MoveIt
// scaling factors in (0, 1] (defaults applied by the loader).
struct Step
{
  std::string type;    // "move_j" / "move_l" / "wait"
  std::string name;    // optional display name (empty if not set)
  std::string target;  // move steps: point name
  double velocity{0.0};
  double acceleration{0.0};
  double duration{0.0};  // wait: seconds

  // Display string for action feedback: name if set, otherwise a summary.
  std::string label() const;
};

struct Program
{
  std::string name;
  std::string description;
  std::vector<Step> steps;
};

// Both loaders throw std::runtime_error with a precise human-readable message
// ("<file>: <point/step>: <problem>") on any schema violation.
PointMap loadPoints(const std::string & path);
Program loadProgram(const std::string & path);

// Checks that every move step references an existing point and that the
// step/point combination is executable (move_l requires a pose point).
// Throws std::runtime_error on violation.
void crossValidate(const Program & program, const PointMap & points);

}  // namespace r0192_program_executor
