// Unit tests for r0192_program_executor's YAML loader/validator — the single
// place programs and points are parsed. Covers both step vocabularies (legacy
// move_j/move_l/wait and KRL/Pilz ptp/lin/circ), the point schema, cross
// validation, the atomic writer round-trip and the small helpers.

#include <gtest/gtest.h>

#include "r0192_program_executor/program_loader.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

using namespace r0192_program_executor;

namespace
{
// Writes `content` to a unique temp file and returns its path. Files land in a
// per-process temp dir that the fixture removes on teardown.
std::filesystem::path writeTmp(const std::string & content)
{
  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path() / "r0192_loader_test";
  std::filesystem::create_directories(dir);
  const auto path = dir / ("f" + std::to_string(counter++) + ".yaml");
  std::ofstream(path) << content;
  return path;
}

std::string loadProgramErr(const std::string & yaml)
{
  try {
    loadProgram(writeTmp(yaml).string());
  } catch (const std::exception & e) {
    return e.what();
  }
  return "";
}
}  // namespace

class LoaderTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "r0192_loader_test", ec);
  }
};

// ---------------------------------------------------------------------------
// Points
// ---------------------------------------------------------------------------
TEST_F(LoaderTest, JointPointParsed)
{
  const auto p = writeTmp(R"(version: 1
points:
  home:
    type: joint
    joints: [0.0, -0.5, 1.2, 0.0, 1.0, 0.0]
)");
  const auto pts = loadPoints(p.string());
  ASSERT_EQ(pts.size(), 1u);
  const auto & home = pts.at("home");
  EXPECT_EQ(home.type, Point::Type::kJoint);
  ASSERT_EQ(home.joints.size(), 6u);
  EXPECT_DOUBLE_EQ(home.joints[1], -0.5);
  EXPECT_DOUBLE_EQ(home.joints[2], 1.2);
}

TEST_F(LoaderTest, PosePointParsed)
{
  const auto p = writeTmp(R"(version: 1
points:
  drop:
    type: pose
    frame: base_link
    position: {x: 0.3, y: 0.2, z: 0.4}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
)");
  const auto pts = loadPoints(p.string());
  const auto & drop = pts.at("drop");
  EXPECT_EQ(drop.type, Point::Type::kPose);
  EXPECT_EQ(drop.frame, "base_link");
  EXPECT_DOUBLE_EQ(drop.pose.position.x, 0.3);
  EXPECT_DOUBLE_EQ(drop.pose.orientation.w, 1.0);
}

TEST_F(LoaderTest, JointPointWrongCountRejected)
{
  const auto p = writeTmp(R"(version: 1
points:
  bad:
    type: joint
    joints: [0.0, 0.0, 0.0]
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

TEST_F(LoaderTest, PoseUnnormalizedQuaternionRejected)
{
  const auto p = writeTmp(R"(version: 1
points:
  bad:
    type: pose
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.5}
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

TEST_F(LoaderTest, PoseUnsupportedFrameRejected)
{
  const auto p = writeTmp(R"(version: 1
points:
  bad:
    type: pose
    frame: tool0
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

TEST_F(LoaderTest, PointUnknownKeyRejected)
{
  const auto p = writeTmp(R"(version: 1
points:
  home:
    type: joint
    joints: [0, 0, 0, 0, 0, 0]
    speed: 1.0
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

TEST_F(LoaderTest, PointBadNameRejected)
{
  const auto p = writeTmp(R"(version: 1
points:
  "1bad":
    type: joint
    joints: [0, 0, 0, 0, 0, 0]
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

TEST_F(LoaderTest, PointsMissingVersionRejected)
{
  const auto p = writeTmp(R"(points:
  home:
    type: joint
    joints: [0, 0, 0, 0, 0, 0]
)");
  EXPECT_THROW(loadPoints(p.string()), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Programs — legacy + KRL vocabularies
// ---------------------------------------------------------------------------
TEST_F(LoaderTest, ProgramAllStepTypesParsed)
{
  const auto p = writeTmp(R"(version: 1
name: "All steps"
defaults:
  velocity: 0.2
  acceleration: 0.2
steps:
  - type: move_j
    target: a
    velocity: 0.3
  - type: move_l
    target: b
  - type: ptp
    target: a
    vel: 0.5
    acc: 0.4
  - type: lin
    target: b
    c_dis: 0.01
  - type: circ
    via: c
    target: b
  - type: wait
    duration: 2.0
)");
  const auto prog = loadProgram(p.string());
  ASSERT_EQ(prog.steps.size(), 6u);
  EXPECT_EQ(prog.name, "All steps");

  EXPECT_EQ(prog.steps[0].type, "move_j");
  EXPECT_DOUBLE_EQ(prog.steps[0].velocity, 0.3);       // explicit
  EXPECT_DOUBLE_EQ(prog.steps[0].acceleration, 0.2);   // from defaults

  EXPECT_EQ(prog.steps[2].type, "ptp");
  EXPECT_DOUBLE_EQ(prog.steps[2].velocity, 0.5);       // vel key
  EXPECT_DOUBLE_EQ(prog.steps[2].acceleration, 0.4);   // acc key
  EXPECT_DOUBLE_EQ(prog.steps[2].c_dis, 0.0);          // default

  EXPECT_EQ(prog.steps[3].type, "lin");
  EXPECT_DOUBLE_EQ(prog.steps[3].c_dis, 0.01);

  EXPECT_EQ(prog.steps[4].type, "circ");
  EXPECT_EQ(prog.steps[4].via, "c");
  EXPECT_EQ(prog.steps[4].target, "b");

  EXPECT_EQ(prog.steps[5].type, "wait");
  EXPECT_DOUBLE_EQ(prog.steps[5].duration, 2.0);
}

TEST_F(LoaderTest, DefaultScalingFallback)
{
  const auto p = writeTmp(R"(version: 1
name: "x"
steps:
  - type: ptp
    target: a
)");
  const auto prog = loadProgram(p.string());
  EXPECT_DOUBLE_EQ(prog.steps[0].velocity, 0.1);       // global fallback
  EXPECT_DOUBLE_EQ(prog.steps[0].acceleration, 0.1);
}

TEST_F(LoaderTest, CircWithoutViaRejected)
{
  EXPECT_NE(loadProgramErr(R"(version: 1
name: "x"
steps:
  - type: circ
    target: b
)").find("via"), std::string::npos);
}

TEST_F(LoaderTest, NegativeBlendRadiusRejected)
{
  EXPECT_NE(loadProgramErr(R"(version: 1
name: "x"
steps:
  - type: lin
    target: b
    c_dis: -0.1
)").find("c_dis"), std::string::npos);
}

TEST_F(LoaderTest, ScalingOutOfRangeRejected)
{
  EXPECT_THROW(loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: ptp
    target: a
    vel: 1.5
)").string()), std::runtime_error);
}

TEST_F(LoaderTest, UnknownStepTypeRejected)
{
  EXPECT_THROW(loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: move_p
    target: a
)").string()), std::runtime_error);
}

TEST_F(LoaderTest, WrongKeyOnPtpRejected)
{
  // ptp uses vel/acc, not velocity/acceleration.
  EXPECT_NE(loadProgramErr(R"(version: 1
name: "x"
steps:
  - type: ptp
    target: a
    velocity: 0.2
)").find("velocity"), std::string::npos);
}

TEST_F(LoaderTest, EmptyStepsRejected)
{
  EXPECT_THROW(loadProgram(writeTmp(R"(version: 1
name: "x"
steps: []
)").string()), std::runtime_error);
}

TEST_F(LoaderTest, MissingNameRejected)
{
  EXPECT_THROW(loadProgram(writeTmp(R"(version: 1
steps:
  - type: wait
    duration: 1.0
)").string()), std::runtime_error);
}

TEST_F(LoaderTest, NonPositiveWaitRejected)
{
  EXPECT_THROW(loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: wait
    duration: 0
)").string()), std::runtime_error);
}

// ---------------------------------------------------------------------------
// crossValidate
// ---------------------------------------------------------------------------
TEST_F(LoaderTest, CrossValidatePasses)
{
  const auto pts = loadPoints(writeTmp(R"(version: 1
points:
  ja: {type: joint, joints: [0,0,0,0,0,0]}
  pa: {type: pose, position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}
  pb: {type: pose, position: {x: 0.1, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}
)").string());
  const auto prog = loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: ptp
    target: ja
  - type: lin
    target: pa
  - type: circ
    via: pa
    target: pb
)").string());
  EXPECT_NO_THROW(crossValidate(prog, pts));
}

TEST_F(LoaderTest, CrossValidateUnknownPoint)
{
  const auto pts = loadPoints(writeTmp(R"(version: 1
points:
  ja: {type: joint, joints: [0,0,0,0,0,0]}
)").string());
  const auto prog = loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: ptp
    target: nope
)").string());
  EXPECT_THROW(crossValidate(prog, pts), std::runtime_error);
}

TEST_F(LoaderTest, CrossValidateLinNeedsPose)
{
  const auto pts = loadPoints(writeTmp(R"(version: 1
points:
  ja: {type: joint, joints: [0,0,0,0,0,0]}
)").string());
  const auto prog = loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: lin
    target: ja
)").string());
  EXPECT_THROW(crossValidate(prog, pts), std::runtime_error);
}

TEST_F(LoaderTest, CrossValidateCircViaMustBePose)
{
  const auto pts = loadPoints(writeTmp(R"(version: 1
points:
  ja: {type: joint, joints: [0,0,0,0,0,0]}
  pa: {type: pose, position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}
)").string());
  const auto prog = loadProgram(writeTmp(R"(version: 1
name: "x"
steps:
  - type: circ
    via: ja
    target: pa
)").string());
  EXPECT_THROW(crossValidate(prog, pts), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Writer round-trip + helpers
// ---------------------------------------------------------------------------
TEST_F(LoaderTest, SavePointsRoundTrip)
{
  PointMap in;
  Point j;
  j.type = Point::Type::kJoint;
  j.joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  in["jp"] = j;
  Point p;
  p.type = Point::Type::kPose;
  p.frame = "base_link";
  p.pose.position.x = 0.25;
  p.pose.orientation.w = 1.0;
  in["pp"] = p;

  const auto path = std::filesystem::temp_directory_path() / "r0192_loader_test" / "rt.yaml";
  std::filesystem::create_directories(path.parent_path());
  savePoints(path.string(), in);
  const auto out = loadPoints(path.string());

  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out.at("jp").type, Point::Type::kJoint);
  EXPECT_DOUBLE_EQ(out.at("jp").joints[5], 0.6);
  EXPECT_EQ(out.at("pp").type, Point::Type::kPose);
  EXPECT_DOUBLE_EQ(out.at("pp").pose.position.x, 0.25);
}

TEST_F(LoaderTest, IsValidPointName)
{
  EXPECT_TRUE(isValidPointName("home"));
  EXPECT_TRUE(isValidPointName("_p1"));
  EXPECT_TRUE(isValidPointName("Pick_2"));
  EXPECT_FALSE(isValidPointName("1bad"));
  EXPECT_FALSE(isValidPointName("has space"));
  EXPECT_FALSE(isValidPointName(""));
}

TEST_F(LoaderTest, StepLabel)
{
  Step named;
  named.name = "Approach";
  named.type = "ptp";
  EXPECT_EQ(named.label(), "Approach");

  Step move;
  move.type = "lin";
  move.target = "drop";
  EXPECT_EQ(move.label(), "lin -> drop");

  Step wait;
  wait.type = "wait";
  wait.duration = 2.0;
  EXPECT_NE(wait.label().find("wait"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
