#include "r0192_program_executor/program_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace r0192_program_executor
{
namespace
{

constexpr int kSchemaVersion = 1;
constexpr std::size_t kNumArmJoints = 6;
// Applied when neither the step nor the program `defaults` set a value.
// Conservative, consistent with the 0.1 scaling used in the MoveIt config.
constexpr double kDefaultScaling = 0.1;

[[noreturn]] void fail(const std::string & file, const std::string & where,
                       const std::string & what)
{
  throw std::runtime_error(file + ": " + where + ": " + what);
}

bool validName(const std::string & name)
{
  static const std::regex re("^[A-Za-z_][A-Za-z0-9_]*$");
  return std::regex_match(name, re);
}

YAML::Node loadFile(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::BadFile &) {
    throw std::runtime_error(path + ": file not found or not readable");
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(path + ": YAML parse error: " + e.what());
  }
  if (!root.IsMap()) fail(path, "root", "expected a YAML mapping");
  return root;
}

void checkVersion(const YAML::Node & root, const std::string & file)
{
  if (!root["version"] || root["version"].as<int>(-1) != kSchemaVersion) {
    fail(file, "version", "missing or unsupported (expected 'version: 1')");
  }
}

// Typo protection: unknown keys are rejected (additionalProperties: false).
void rejectUnknownKeys(const YAML::Node & map, const std::set<std::string> & allowed,
                       const std::string & file, const std::string & where)
{
  for (const auto & kv : map) {
    const auto key = kv.first.as<std::string>();
    if (!allowed.count(key)) fail(file, where, "unknown key '" + key + "'");
  }
}

double asNumber(const YAML::Node & node, const std::string & file,
                const std::string & where, const std::string & key)
{
  if (!node || !node.IsScalar()) fail(file, where, "'" + key + "' missing or not a number");
  try {
    return node.as<double>();
  } catch (const YAML::Exception &) {
    fail(file, where, "'" + key + "' is not a number");
  }
}

// Scaling factors must be in (0, 1]; `fallback` is used when the key is absent.
double scalingOrDefault(const YAML::Node & node, double fallback, const std::string & file,
                        const std::string & where, const std::string & key)
{
  if (!node) return fallback;
  const double v = asNumber(node, file, where, key);
  if (v <= 0.0 || v > 1.0) {
    fail(file, where, "'" + key + "' must be a scaling factor in (0, 1]");
  }
  return v;
}

// Blend radius c_dis (m): non-negative, 0 (the default) means "stop at point".
double blendRadiusOrZero(const YAML::Node & node, const std::string & file,
                         const std::string & where)
{
  if (!node) return 0.0;
  const double v = asNumber(node, file, where, "c_dis");
  if (v < 0.0) fail(file, where, "'c_dis' (blend radius, m) must be >= 0");
  return v;
}

Point parseJointPoint(const YAML::Node & node, const std::string & file,
                      const std::string & where)
{
  rejectUnknownKeys(node, {"type", "joints"}, file, where);
  const auto joints = node["joints"];
  if (!joints || !joints.IsSequence()) {
    fail(file, where, "'joints' missing or not a list");
  }
  if (joints.size() != kNumArmJoints) {
    fail(file, where, "'joints' must have exactly 6 values (joint_1..joint_6, rad)");
  }
  Point p;
  p.type = Point::Type::kJoint;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    p.joints.push_back(asNumber(joints[i], file, where, "joints[" + std::to_string(i) + "]"));
  }
  return p;
}

Point parsePosePoint(const YAML::Node & node, const std::string & file,
                     const std::string & where)
{
  rejectUnknownKeys(node, {"type", "frame", "position", "orientation"}, file, where);

  Point p;
  p.type = Point::Type::kPose;
  p.frame = node["frame"] ? node["frame"].as<std::string>() : "base_link";
  if (p.frame != "base_link") {
    fail(file, where, "only frame 'base_link' is supported (got '" + p.frame + "')");
  }

  const auto pos = node["position"];
  if (!pos || !pos.IsMap()) fail(file, where, "'position' missing or not a mapping");
  rejectUnknownKeys(pos, {"x", "y", "z"}, file, where + " position");
  p.pose.position.x = asNumber(pos["x"], file, where, "position.x");
  p.pose.position.y = asNumber(pos["y"], file, where, "position.y");
  p.pose.position.z = asNumber(pos["z"], file, where, "position.z");

  const auto ori = node["orientation"];
  if (!ori || !ori.IsMap()) fail(file, where, "'orientation' missing or not a mapping");
  rejectUnknownKeys(ori, {"x", "y", "z", "w"}, file, where + " orientation");
  p.pose.orientation.x = asNumber(ori["x"], file, where, "orientation.x");
  p.pose.orientation.y = asNumber(ori["y"], file, where, "orientation.y");
  p.pose.orientation.z = asNumber(ori["z"], file, where, "orientation.z");
  p.pose.orientation.w = asNumber(ori["w"], file, where, "orientation.w");

  const double norm = std::sqrt(
    p.pose.orientation.x * p.pose.orientation.x +
    p.pose.orientation.y * p.pose.orientation.y +
    p.pose.orientation.z * p.pose.orientation.z +
    p.pose.orientation.w * p.pose.orientation.w);
  if (std::abs(norm - 1.0) > 0.01) {
    fail(file, where, "orientation quaternion is not normalized (|q| = " +
                        std::to_string(norm) + ")");
  }
  return p;
}

}  // namespace

std::string Step::label() const
{
  if (!name.empty()) return name;
  if (type == "wait") {
    std::ostringstream os;
    os << "wait " << duration << " s";
    return os.str();
  }
  return type + " -> " + target;
}

PointMap loadPoints(const std::string & path)
{
  const YAML::Node root = loadFile(path);
  checkVersion(root, path);
  rejectUnknownKeys(root, {"version", "points"}, path, "root");

  const auto pts = root["points"];
  if (!pts || !pts.IsMap()) fail(path, "points", "missing or not a mapping");

  PointMap out;
  for (const auto & kv : pts) {
    const auto name = kv.first.as<std::string>();
    const std::string where = "point '" + name + "'";
    if (!validName(name)) {
      fail(path, where, "invalid name (allowed: ^[A-Za-z_][A-Za-z0-9_]*$)");
    }
    const auto node = kv.second;
    if (!node.IsMap()) fail(path, where, "expected a mapping");
    const auto type = node["type"] ? node["type"].as<std::string>() : "";
    if (type == "joint") {
      out.emplace(name, parseJointPoint(node, path, where));
    } else if (type == "pose") {
      out.emplace(name, parsePosePoint(node, path, where));
    } else {
      fail(path, where, "'type' must be 'joint' or 'pose'");
    }
  }
  return out;
}

Program loadProgram(const std::string & path)
{
  const YAML::Node root = loadFile(path);
  checkVersion(root, path);
  rejectUnknownKeys(root, {"version", "name", "description", "defaults", "steps"}, path, "root");

  Program prog;
  if (!root["name"] || root["name"].as<std::string>("").empty()) {
    fail(path, "name", "missing or empty");
  }
  prog.name = root["name"].as<std::string>();
  prog.description = root["description"] ? root["description"].as<std::string>() : "";

  double def_vel = kDefaultScaling;
  double def_acc = kDefaultScaling;
  if (const auto defaults = root["defaults"]) {
    if (!defaults.IsMap()) fail(path, "defaults", "expected a mapping");
    rejectUnknownKeys(defaults, {"velocity", "acceleration"}, path, "defaults");
    def_vel = scalingOrDefault(defaults["velocity"], def_vel, path, "defaults", "velocity");
    def_acc = scalingOrDefault(defaults["acceleration"], def_acc, path, "defaults", "acceleration");
  }

  const auto steps = root["steps"];
  if (!steps || !steps.IsSequence() || steps.size() == 0) {
    fail(path, "steps", "missing or empty (expected a non-empty list)");
  }

  for (std::size_t i = 0; i < steps.size(); ++i) {
    const auto node = steps[i];
    const std::string where = "step " + std::to_string(i + 1);
    if (!node.IsMap()) fail(path, where, "expected a mapping");

    Step s;
    s.type = node["type"] ? node["type"].as<std::string>() : "";
    s.name = node["name"] ? node["name"].as<std::string>() : "";

    if (s.type == "move_j" || s.type == "move_l") {
      // Legacy vocabulary (phase 1/6): velocity/acceleration keys, no blending.
      rejectUnknownKeys(node, {"type", "name", "target", "velocity", "acceleration"},
                        path, where);
      if (!node["target"] || node["target"].as<std::string>("").empty()) {
        fail(path, where, "'target' (point name) missing or empty");
      }
      s.target = node["target"].as<std::string>();
      if (!validName(s.target)) fail(path, where, "invalid target name '" + s.target + "'");
      s.velocity = scalingOrDefault(node["velocity"], def_vel, path, where, "velocity");
      s.acceleration = scalingOrDefault(node["acceleration"], def_acc, path, where, "acceleration");
    } else if (s.type == "ptp" || s.type == "lin" || s.type == "circ") {
      // KRL/Pilz vocabulary (phase 7): vel/acc scaling + optional c_dis blend
      // radius. circ also takes a via (auxiliary) point.
      std::set<std::string> allowed{"type", "name", "target", "vel", "acc", "c_dis"};
      if (s.type == "circ") allowed.insert("via");
      rejectUnknownKeys(node, allowed, path, where);
      if (!node["target"] || node["target"].as<std::string>("").empty()) {
        fail(path, where, "'target' (point name) missing or empty");
      }
      s.target = node["target"].as<std::string>();
      if (!validName(s.target)) fail(path, where, "invalid target name '" + s.target + "'");
      if (s.type == "circ") {
        if (!node["via"] || node["via"].as<std::string>("").empty()) {
          fail(path, where, "circ requires a 'via' (auxiliary point) name");
        }
        s.via = node["via"].as<std::string>();
        if (!validName(s.via)) fail(path, where, "invalid via name '" + s.via + "'");
      }
      s.velocity = scalingOrDefault(node["vel"], def_vel, path, where, "vel");
      s.acceleration = scalingOrDefault(node["acc"], def_acc, path, where, "acc");
      s.c_dis = blendRadiusOrZero(node["c_dis"], path, where);
    } else if (s.type == "wait") {
      rejectUnknownKeys(node, {"type", "name", "duration"}, path, where);
      s.duration = asNumber(node["duration"], path, where, "duration");
      if (s.duration <= 0.0) fail(path, where, "'duration' must be > 0 (seconds)");
    } else {
      fail(path, where, "'type' must be one of: move_j, move_l, ptp, lin, circ, wait");
    }
    prog.steps.push_back(std::move(s));
  }
  return prog;
}

bool isValidPointName(const std::string & name)
{
  return validName(name);
}

void savePoints(const std::string & path, const PointMap & points)
{
  YAML::Emitter out;
  out.SetDoublePrecision(9);
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << kSchemaVersion;
  out << YAML::Key << "points" << YAML::Value << YAML::BeginMap;
  for (const auto & [name, p] : points) {
    out << YAML::Key << name << YAML::Value << YAML::BeginMap;
    if (p.type == Point::Type::kJoint) {
      out << YAML::Key << "type" << YAML::Value << "joint";
      out << YAML::Key << "joints" << YAML::Value << YAML::Flow << p.joints;
    } else {
      out << YAML::Key << "type" << YAML::Value << "pose";
      out << YAML::Key << "frame" << YAML::Value << p.frame;
      out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginMap
          << YAML::Key << "x" << YAML::Value << p.pose.position.x
          << YAML::Key << "y" << YAML::Value << p.pose.position.y
          << YAML::Key << "z" << YAML::Value << p.pose.position.z
          << YAML::EndMap;
      out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginMap
          << YAML::Key << "x" << YAML::Value << p.pose.orientation.x
          << YAML::Key << "y" << YAML::Value << p.pose.orientation.y
          << YAML::Key << "z" << YAML::Value << p.pose.orientation.z
          << YAML::Key << "w" << YAML::Value << p.pose.orientation.w
          << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap << YAML::EndMap;

  const std::string tmp = path + ".tmp";
  {
    std::ofstream file(tmp);
    if (!file) throw std::runtime_error("cannot write " + tmp);
    file << "# R0192 point database — named targets referenced by programs (by name only).\n"
            "# Edited in VS Code (schema: doku/schemas/points.schema.json) or rewritten by\n"
            "# the /teach_point and /delete_point services (comments are not preserved).\n"
         << out.c_str() << "\n";
    if (!file.good()) throw std::runtime_error("write to " + tmp + " failed");
  }
  std::filesystem::rename(tmp, path);
}

void crossValidate(const Program & program, const PointMap & points)
{
  // Cartesian moves (line/arc) need pose-type points; joint-space moves accept
  // either. move_l ~ lin, move_j ~ ptp.
  auto requiresPose = [](const std::string & type) {
    return type == "move_l" || type == "lin" || type == "circ";
  };
  auto isMove = [](const std::string & type) {
    return type == "move_j" || type == "move_l" ||
           type == "ptp" || type == "lin" || type == "circ";
  };

  for (std::size_t i = 0; i < program.steps.size(); ++i) {
    const auto & s = program.steps[i];
    const std::string where = "step " + std::to_string(i + 1) + " (" + s.type + ")";
    if (!isMove(s.type)) continue;

    const auto it = points.find(s.target);
    if (it == points.end()) {
      throw std::runtime_error(where + ": references unknown point '" + s.target + "'");
    }
    if (requiresPose(s.type) && it->second.type != Point::Type::kPose) {
      throw std::runtime_error(where + ": " + s.type + " requires a pose-type point ('" +
                               s.target + "' is joint-type)");
    }
    if (s.type == "circ") {
      const auto via = points.find(s.via);
      if (via == points.end()) {
        throw std::runtime_error(where + ": references unknown via point '" + s.via + "'");
      }
      if (via->second.type != Point::Type::kPose) {
        throw std::runtime_error(where + ": circ via point '" + s.via +
                                 "' must be pose-type (is joint-type)");
      }
    }
  }
}

}  // namespace r0192_program_executor
