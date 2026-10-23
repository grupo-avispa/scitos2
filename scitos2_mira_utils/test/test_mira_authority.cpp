// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fw/Framework.h>
#include <fw/MicroUnit.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "scitos2_mira_utils/mira_authority.hpp"

TEST(MiraAuthorityTest, checkinStartCheckoutDoesNotThrow) {
  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority");
  authority.start();
  authority.checkout();
  SUCCEED();
}

TEST(MiraAuthorityTest, serviceCallFailsWithoutARealResource) {
  // No '/robot/Robot' resource is registered in this test process, so every call must
  // report failure instead of throwing or silently succeeding
  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority_service");
  authority.start();

  EXPECT_FALSE(authority.callService("someService"));
  EXPECT_FALSE(authority.setParam("Some.Param", "1"));
  EXPECT_EQ(authority.getParam("Some.Param"), "");

  authority.checkout();
}

TEST(MiraAuthorityTest, customResourceIsHonored) {
  scitos2_mira_utils::MiraAuthority authority;
  authority.setResource("/robot/DoesNotExist");
  authority.checkin("test_mira_authority_resource");
  authority.start();

  // Still fails (the resource doesn't exist either), but exercises the configured
  // resource path instead of the default
  EXPECT_FALSE(authority.callService("someService"));

  authority.checkout();
}

// A fake robot resource exposing the services MiraAuthority talks to
class FakeRobot : public mira::MicroUnit
{
public:
  FakeRobot() {}

  template<typename Reflector>
  void reflect(Reflector & r)
  {
    MIRA_REFLECT_BASE(r, mira::MicroUnit);
    r.method("noArgs", &FakeRobot::noArgs, this, "No-argument service");
    r.method("withArg", &FakeRobot::withArg, this, "Service with an argument", "x", "An int");
    r.method("fails", &FakeRobot::fails, this, "Service that always throws");
    r.method("slow", &FakeRobot::slow, this, "Service that takes a while to answer");
    r.method(
      "slowWithArg", &FakeRobot::slowWithArg, this, "Slow service with an argument", "x",
      "An int");
  }

  void noArgs() {no_args_calls_++;}
  void withArg(int x) {last_arg_ = x;}
  void fails() {throw std::runtime_error("expected failure");}
  void slow() {std::this_thread::sleep_for(std::chrono::milliseconds(500));}
  void slowWithArg(int x)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    last_arg_ = x;
  }

  std::atomic<int> no_args_calls_{0};
  std::atomic<int> last_arg_{-1};
};

TEST(MiraAuthorityTest, servicesAndParamsAgainstARealResource) {
  FakeRobot robot;
  robot.checkin("/robot", "Robot");
  robot.publishService(robot);
  robot.start();

  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority_real");
  authority.start();

  EXPECT_TRUE(authority.callService("noArgs"));
  EXPECT_EQ(robot.no_args_calls_, 1);
  EXPECT_TRUE(authority.callService<int>("withArg", 42));
  EXPECT_EQ(robot.last_arg_, 42);
  EXPECT_TRUE(authority.callService<int>("noArgs"));

  // Failures reported by the remote side (including a property that does not exist) are
  // turned into a `false` / empty result
  EXPECT_FALSE(authority.callService("fails"));
  EXPECT_FALSE(authority.callService("doesNotExist"));
  EXPECT_FALSE(authority.callService<int>("fails", 1));
  EXPECT_FALSE(authority.setParam("NoSuchParam", "1"));
  EXPECT_EQ(authority.getParam("NoSuchParam"), "");

  authority.checkout();
}

TEST(MiraAuthorityTest, slowServiceTimesOutInsteadOfBlocking) {
  FakeRobot robot;
  robot.checkin("/robot", "Robot");
  robot.publishService(robot);
  robot.start();

  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority_timeout");
  authority.start();
  authority.setTimeout(std::chrono::milliseconds(50));

  // The service needs 500 ms, so the call must give up after ~50 ms instead of waiting for it
  auto begin = std::chrono::steady_clock::now();
  EXPECT_FALSE(authority.callService("slow"));
  EXPECT_LT(std::chrono::steady_clock::now() - begin, std::chrono::milliseconds(400));

  // Let the robot finish the slow call before the next one
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  EXPECT_TRUE(authority.callService("noArgs"));

  authority.checkout();
}

TEST(MiraAuthorityTest, waitForResource) {
  scitos2_mira_utils::MiraAuthority authority;
  authority.setResource("/robot/Waited");
  authority.checkin("test_mira_authority_wait");
  authority.start();

  EXPECT_FALSE(authority.waitForResource(std::chrono::milliseconds(100)));

  FakeRobot robot;
  robot.checkin("/robot", "Waited");
  robot.publishService(robot);
  robot.start();

  EXPECT_TRUE(authority.waitForResource(std::chrono::milliseconds(2000)));

  authority.checkout();
}

TEST(MiraAuthorityTest, startAndCheckoutAreIdempotent) {
  scitos2_mira_utils::MiraAuthority authority;
  // Nothing to check out before checking in
  EXPECT_NO_THROW(authority.checkout());

  authority.checkin("test_mira_authority_idempotent");
  EXPECT_NO_THROW(authority.start());
  EXPECT_NO_THROW(authority.start());
  EXPECT_NO_THROW(authority.checkout());
  EXPECT_NO_THROW(authority.checkout());
}

TEST(MiraAuthorityTest, asyncCallDoesNotBlockTheCaller) {
  FakeRobot robot;
  robot.checkin("/robot", "Robot");
  robot.publishService(robot);
  robot.start();

  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority_async");
  authority.start();

  // The service needs 300 ms, but the caller must be back right away, also on the next call
  // that finds the previous one still unanswered
  auto begin = std::chrono::steady_clock::now();
  EXPECT_TRUE(authority.callServiceAsync<int>("slowWithArg", 1));
  EXPECT_TRUE(authority.callServiceAsync<int>("withArg", 7));
  EXPECT_LT(std::chrono::steady_clock::now() - begin, std::chrono::milliseconds(200));

  // The calls are still delivered
  for (int i = 0; i < 100 && robot.last_arg_ != 1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_NE(robot.last_arg_, -1);

  authority.checkout();
}

TEST(MiraAuthorityTest, asyncCallFailsWithoutARealResource) {
  scitos2_mira_utils::MiraAuthority authority;
  authority.checkin("test_mira_authority_async_missing");
  authority.start();

  EXPECT_FALSE(authority.callServiceAsync<int>("someService", 1));

  authority.checkout();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
