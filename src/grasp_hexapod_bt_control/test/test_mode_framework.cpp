// 模式框架单测：YAML 加载器、注册表、ModeLifecycle 状态机（run 代次/等待/
// 暂停/抢占 superseded）。不依赖 ROS master（不构造真实节点）。
#include "grasp_hexapod_bt_control/mode_base.h"
#include "grasp_hexapod_bt_control/mode_registry.h"
#include "grasp_hexapod_bt_control/mode_topic_config.h"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

namespace grasp_hexapod_bt_control {
namespace {

// ---------------------------------------------------------------------------
// 测试辅助。
// ---------------------------------------------------------------------------

class TestLifecycle : public ModeLifecycle {
 public:
  using ModeLifecycle::setPaused;
};

// 注册表测试用的最小模式（工厂不会被真正调用，但必须可构造）。
class DummyMode : public ModeBase {
 public:
  DummyMode(ModeContext& ctx, const ModeTopics& topics)
      : ModeBase("__dummy__", ctx, topics) {}
  bool start(const JointAngles&, std::string&) override { return true; }
  std::optional<JointAngles> step(const ModeStepInput&) override {
    return std::nullopt;
  }
  std::optional<ModeOutcome> checkTerminal() override { return std::nullopt; }
};

REGISTER_MODE("__dummy_test__", DummyMode)

class StubMode : public ModeBase {
 public:
  StubMode(ModeContext& ctx, const ModeTopics& topics)
      : ModeBase("__stub__", ctx, topics) {}
  bool start(const JointAngles&, std::string&) override { return false; }
  std::optional<JointAngles> step(const ModeStepInput&) override {
    return std::nullopt;
  }
  std::optional<ModeOutcome> checkTerminal() override { return std::nullopt; }
};

REGISTER_MODE_STUB("__stub_test__", StubMode)

std::string writeTempYaml(const std::string& name, const std::string& content) {
  const std::string path = "/tmp/" + name;
  std::ofstream out(path);
  out << content;
  return path;
}

// ---------------------------------------------------------------------------
// ModeTopicConfig。
// ---------------------------------------------------------------------------

TEST(ModeTopicConfig, LoadsSectionsAndKeys) {
  const std::string path = writeTempYaml(
      "mode_framework_ok.yaml",
      "shared:\n"
      "  imu: /grasp_hexapod/imu\n"
      "  joint_feedback: [/lf_pos, /lm_pos, /lb_pos, /rf_pos, /rm_pos, /rb_pos]\n"
      "walk:\n"
      "  cmd_vel: /cmd_vel\n");
  const ModeTopicConfig config = ModeTopicConfig::load(path);
  EXPECT_EQ(config.section("walk").get("cmd_vel"), "/cmd_vel");
  EXPECT_EQ(config.section("shared").get("imu"), "/grasp_hexapod/imu");
  EXPECT_EQ(config.section("shared").get("missing", "/fallback"), "/fallback");
  EXPECT_TRUE(config.section("missing_mode").empty());
  const std::vector<std::string> feedback =
      config.sectionList("shared", "joint_feedback");
  EXPECT_EQ(feedback.size(), 6u);
  EXPECT_EQ(feedback.front(), "/lf_pos");
  EXPECT_EQ(feedback.back(), "/rb_pos");
  EXPECT_FALSE(config.sectionNames().empty());
}

TEST(ModeTopicConfig, MissingFileThrows) {
  EXPECT_THROW(ModeTopicConfig::load("/tmp/definitely_missing_9x.yaml"),
               TopicConfigError);
}

TEST(ModeTopicConfig, NonMapRootThrows) {
  const std::string path =
      writeTempYaml("mode_framework_list.yaml", "- a\n- b\n");
  EXPECT_THROW(ModeTopicConfig::load(path), TopicConfigError);
}

TEST(ModeTopicConfig, MissingKeyThrowsWithName) {
  const std::string path = writeTempYaml(
      "mode_framework_missing_key.yaml", "walk:\n  other: /x\n");
  const ModeTopicConfig config = ModeTopicConfig::load(path);
  try {
    config.section("walk").get("cmd_vel");
    FAIL() << "expected TopicConfigError";
  } catch (const TopicConfigError& error) {
    EXPECT_NE(std::string(error.what()).find("walk.cmd_vel"),
              std::string::npos);
  }
}

TEST(ModeTopicConfig, MissingListThrowsWithName) {
  const std::string path =
      writeTempYaml("mode_framework_missing_list.yaml", "shared:\n  imu: /i\n");
  const ModeTopicConfig config = ModeTopicConfig::load(path);
  EXPECT_THROW(config.sectionList("shared", "joint_feedback"),
               TopicConfigError);
}

// ---------------------------------------------------------------------------
// ModeRegistry。
// ---------------------------------------------------------------------------

TEST(ModeRegistry, MacroRegistrationVisible) {
  EXPECT_TRUE(ModeRegistry::instance().has("__dummy_test__"));
  EXPECT_TRUE(ModeRegistry::instance().implemented("__dummy_test__"));
  EXPECT_TRUE(ModeRegistry::instance().has("__stub_test__"));
  EXPECT_FALSE(ModeRegistry::instance().implemented("__stub_test__"));
  EXPECT_FALSE(ModeRegistry::instance().has("__never_registered__"));
}

// ---------------------------------------------------------------------------
// ModeLifecycle：run 代次 / wait / finish / 抢占 / 暂停标志。
// ---------------------------------------------------------------------------

TEST(ModeLifecycle, FreshModeIsRunning) {
  TestLifecycle mode;
  EXPECT_EQ(mode.status().result, ModeResult::Running);
  EXPECT_EQ(mode.currentRun(), 0u);
  EXPECT_FALSE(mode.isPaused());
}

TEST(ModeLifecycle, WaitForRunTimesOutAsRunning) {
  TestLifecycle mode;
  const uint64_t run = mode.allocateRun();
  const ModeOutcome outcome = mode.waitForRun(run, 0.0);
  EXPECT_EQ(outcome.result, ModeResult::Running);
}

TEST(ModeLifecycle, FinishWakesWaiter) {
  TestLifecycle mode;
  const uint64_t run = mode.allocateRun();

  std::thread waiter([&mode, run] {
    const ModeOutcome outcome = mode.waitForRun(run, 2.0);
    EXPECT_EQ(outcome.result, ModeResult::Success);
    EXPECT_EQ(outcome.message, "done");
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  mode.finish(ModeResult::Success, "done");
  waiter.join();
  EXPECT_EQ(mode.status().result, ModeResult::Success);
}

TEST(ModeLifecycle, FinishIsIdempotentFirstWins) {
  TestLifecycle mode;
  mode.allocateRun();
  mode.finish(ModeResult::Success, "first");
  mode.finish(ModeResult::Failure, "second");
  EXPECT_EQ(mode.status().message, "first");
}

TEST(ModeLifecycle, NewRunSupersedesOldWaiter) {
  TestLifecycle mode;
  const uint64_t run1 = mode.allocateRun();
  mode.finish(ModeResult::Success, "first done");
  const uint64_t run2 = mode.allocateRun();  // 新调用：清 outcome、代次 +1
  EXPECT_NE(run1, run2);
  // 旧 run 的迟到等待者读到 superseded，而不是新 run 的结果。
  const ModeOutcome stale = mode.waitForRun(run1, 0.05);
  EXPECT_EQ(stale.result, ModeResult::Preempted);
  EXPECT_EQ(stale.message, "superseded");
  EXPECT_EQ(mode.waitForRun(run2, 0.0).result, ModeResult::Running);
}

TEST(ModeLifecycle, MarkPreemptedOnlyWhenRunning) {
  TestLifecycle mode;
  mode.allocateRun();
  mode.markPreempted("preempted by dock");
  EXPECT_EQ(mode.status().result, ModeResult::Preempted);
  EXPECT_EQ(mode.status().message, "preempted by dock");
  // 已终态后再次抢占不覆盖原结果。
  mode.markPreempted("preempted by home");
  EXPECT_EQ(mode.status().message, "preempted by dock");
}

TEST(ModeLifecycle, PauseFlagRoundTrip) {
  TestLifecycle mode;
  mode.setPaused(true);
  EXPECT_TRUE(mode.isPaused());
  mode.setPaused(false);
  EXPECT_FALSE(mode.isPaused());
}

}  // namespace
}  // namespace grasp_hexapod_bt_control
