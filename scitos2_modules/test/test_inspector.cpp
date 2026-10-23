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

// The serialization code of MIRA triggers a false positive of this warning
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <fw/Framework.h>
#include <fw/MicroUnit.h>

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "scitos2_modules/inspector.hpp"

// A fake robot resource with one property and one method with arguments
class FakeRobot : public mira::MicroUnit
{
  MIRA_OBJECT(FakeRobot)

public:
  FakeRobot() {}

  template<typename Reflector>
  void reflect(Reflector & r)
  {
    MIRA_REFLECT_BASE(r, mira::MicroUnit);
    r.property("Gain", gain_, "A property to read and write", 1.0f);
    r.method("add", &FakeRobot::add, this, "Add two integers", "a", "First term", "b",
      "Second term");
  }

  int add(int a, int b)
  {
    return a + b;
  }

  float gain_{1.0f};
};

MIRA_CLASS_SERIALIZATION(FakeRobot, mira::MicroUnit);

class InspectorFixture : public scitos2_modules::Inspector
{
public:
  using scitos2_modules::Inspector::getProperty;
  using scitos2_modules::Inspector::setProperty;
  using scitos2_modules::Inspector::callJson;
};

TEST(ScitosInspectorTest, servicesAnswerWhileActive) {
  rclcpp::init(0, nullptr);
  FakeRobot robot;
  robot.checkin("/robot", "Robot");
  robot.publishService(robot);
  robot.start();

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testInspector");
  node->declare_parameter("test.enabled", true);
  auto module = std::make_shared<InspectorFixture>();
  module->configure(node, "test");

  // Nothing is answered before the module is active
  auto get_request = std::make_shared<scitos2_msgs::srv::GetProperty::Request>();
  auto get_response = std::make_shared<scitos2_msgs::srv::GetProperty::Response>();
  get_request->name = "Gain";
  module->getProperty(get_request, get_response);
  EXPECT_FALSE(get_response->success);

  ASSERT_TRUE(module->activate());

  // Read and write a property
  module->getProperty(get_request, get_response);
  EXPECT_TRUE(get_response->success) << get_response->message;
  EXPECT_FLOAT_EQ(std::stof(get_response->value), 1.0f);

  auto set_request = std::make_shared<scitos2_msgs::srv::SetProperty::Request>();
  auto set_response = std::make_shared<scitos2_msgs::srv::SetProperty::Response>();
  set_request->name = "Gain";
  set_request->value = "2.5";
  module->setProperty(set_request, set_response);
  EXPECT_TRUE(set_response->success) << set_response->message;
  EXPECT_FLOAT_EQ(robot.gain_, 2.5f);

  // A property that does not exist is reported
  get_request->name = "NoSuchProperty";
  module->getProperty(get_request, get_response);
  EXPECT_FALSE(get_response->success);
  EXPECT_FALSE(get_response->message.empty());

  // A method with arguments, called with JSON
  auto call_request = std::make_shared<scitos2_msgs::srv::CallJson::Request>();
  auto call_response = std::make_shared<scitos2_msgs::srv::CallJson::Response>();
  call_request->service = "/robot/Robot";
  call_request->method = "add";
  call_request->params = "[40, 2]";
  module->callJson(call_request, call_response);
  EXPECT_TRUE(call_response->success) << call_response->message;
  EXPECT_EQ(call_response->result, "42");

  // Malformed JSON, an unknown method and an unknown service are reported
  call_request->params = "[40, ";
  module->callJson(call_request, call_response);
  EXPECT_FALSE(call_response->success);
  call_request->params = "[40, 2]";
  call_request->method = "doesNotExist";
  module->callJson(call_request, call_response);
  EXPECT_FALSE(call_response->success);
  call_request->service = "/robot/DoesNotExist";
  module->callJson(call_request, call_response);
  EXPECT_FALSE(call_response->success);

  // And nothing is answered again once it is deactivated
  EXPECT_TRUE(module->deactivate());
  module->getProperty(get_request, get_response);
  EXPECT_FALSE(get_response->success);

  module->cleanup();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
