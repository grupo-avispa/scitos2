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

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "scitos2_mira/mira_framework.hpp"
#include "scitos2_core/module.hpp"

class MiraFrameworkFixture : public scitos2_mira::MiraFramework
{
public:
  MiraFrameworkFixture()
  : scitos2_mira::MiraFramework(rclcpp::NodeOptions())
  {}

  diagnostic_msgs::msg::DiagnosticArray createDiagnostics()
  {
    return scitos2_mira::MiraFramework::createDiagnostics();
  }
};

class DummyModule : public scitos2_core::Module
{
public:
  enum class Behavior { OK, FAIL_ACTIVATE, THROW_ACTIVATE, FAIL_DEACTIVATE };

  explicit DummyModule(Behavior behavior = Behavior::OK)
  : behavior_(behavior) {}

  ~DummyModule() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &/*parent*/, std::string/*name*/) {}

  virtual void cleanup() {}

  virtual bool activate()
  {
    if (behavior_ == Behavior::THROW_ACTIVATE) {
      throw std::runtime_error("activation exception");
    }
    return behavior_ != Behavior::FAIL_ACTIVATE;
  }

  virtual bool deactivate() {return behavior_ != Behavior::FAIL_DEACTIVATE;}

private:
  Behavior behavior_;
};

// Mocked class loader
void onPluginDeletion(scitos2_core::Module * obj)
{
  if (nullptr != obj) {
    delete (obj);
  }
}

template<>
pluginlib::UniquePtr<scitos2_core::Module> pluginlib::ClassLoader<scitos2_core::Module>::
createUniqueInstance(const std::string & lookup_name)
{
  using Behavior = DummyModule::Behavior;
  const std::map<std::string, Behavior> mocked = {
    {"drive", Behavior::OK},
    {"fail_activate", Behavior::FAIL_ACTIVATE},
    {"throw_activate", Behavior::THROW_ACTIVATE},
    {"fail_deactivate", Behavior::FAIL_DEACTIVATE}};
  auto behavior = mocked.find(lookup_name);
  if (behavior == mocked.end()) {
    // original method body
    if (!isClassLoaded(lookup_name)) {
      loadLibraryForClass(lookup_name);
    }
    try {
      std::string class_type = getClassType(lookup_name);
      pluginlib::UniquePtr<scitos2_core::Module> obj =
        lowlevel_class_loader_.createUniqueInstance<scitos2_core::Module>(class_type);
      return obj;
    } catch (const class_loader::CreateClassException & ex) {
      throw pluginlib::CreateClassException(ex.what());
    }
  }

  // mocked plugin creation
  return std::unique_ptr<scitos2_core::Module,
           class_loader::ClassLoader::DeleterType<scitos2_core::Module>>(
    new DummyModule(behavior->second), onPluginDeletion);
}

namespace
{
using lifecycle_msgs::msg::State;

// MIRA only allows one framework per process, so every test shares the same node. The tests
// are order dependent: those that need a framework without a loaded configuration come first
// and the one that leaves the node finalized comes last.
std::shared_ptr<MiraFrameworkFixture> sharedNode()
{
  static auto node = std::make_shared<MiraFrameworkFixture>();
  return node;
}

std::string scitosConfig()
{
  return ament_index_cpp::get_package_share_directory("scitos2_mira") +
         "/test/scitos_config.xml";
}

void setParameter(const std::string & name, const rclcpp::ParameterValue & value)
{
  auto node = sharedNode();
  nav2_util::declare_parameter_if_not_declared(node, name, value);
  node->set_parameter(rclcpp::Parameter(name, value));
}

// Select the modules the node loads, each one mocked by the class loader through its name
void setModules(const std::vector<std::string> & modules)
{
  setParameter("module_plugins", rclcpp::ParameterValue(modules));
  for (const auto & module : modules) {
    setParameter(module + ".plugin", rclcpp::ParameterValue(module));
  }
}
}  // namespace

TEST(ScitosMiraFrameworkTest, diagnosticsReportAMissingConfiguration) {
  auto msg = sharedNode()->createDiagnostics();
  ASSERT_EQ(msg.status.size(), 1u);
  EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(ScitosMiraFrameworkTest, configureFailsWithMissingConfigFile) {
  auto node = sharedNode();
  setParameter("scitos_config", rclcpp::ParameterValue("/does/not/exist.xml"));
  setModules({});

  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ScitosMiraFrameworkTest, configure) {
  auto node = sharedNode();

  // Set an empty scitos config parameter
  setParameter("scitos_config", rclcpp::ParameterValue(""));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the unconfigured state as scitos_config plugins is empty
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // Now, set the scitos config parameter. In the the robot this should be a XML file
  setParameter("scitos_config", rclcpp::ParameterValue(scitosConfig()));
  setModules({"drive"});

  // Configure the node
  node->configure();
  node->activate();

  // Just call the function
  node->createDiagnostics();

  // Check results: the node should be in the active state
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  // Cleaning up
  node->deactivate();
  node->cleanup();

  // Configure the node again to warn that the scitos config is already loaded
  node->configure();
  node->activate();

  // Cleaning up
  node->deactivate();
  node->cleanup();
}

TEST(ScitosMiraFrameworkTest, diagnosticsWarnWithoutModules) {
  auto node = sharedNode();
  setModules({});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);

  auto msg = node->createDiagnostics();
  EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  node->cleanup();
}

TEST(ScitosMiraFrameworkTest, configureFailsWithUnknownModule) {
  auto node = sharedNode();
  setModules({"not_a_module"});
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ScitosMiraFrameworkTest, activationFailureOfAModuleIsReported) {
  auto node = sharedNode();
  setModules({"fail_activate"});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_INACTIVE);
  node->cleanup();
}

TEST(ScitosMiraFrameworkTest, activationExceptionOfAModuleIsReported) {
  auto node = sharedNode();
  setModules({"throw_activate"});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_INACTIVE);
  node->cleanup();
}

TEST(ScitosMiraFrameworkTest, diagnosticsArePublishedWhileActive) {
  auto node = sharedNode();
  setModules({"drive"});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto sub_node = std::make_shared<rclcpp::Node>("diagnostics_listener");
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr received;
  auto sub = sub_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {received = msg;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(sub_node);
  auto start = std::chrono::steady_clock::now();
  while (!received && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  executor.remove_node(node->get_node_base_interface());
  node->deactivate();
  node->cleanup();
}

TEST(ScitosMiraFrameworkTest, deactivationFailureOfAModuleIsReported) {
  auto node = sharedNode();
  setModules({"fail_deactivate"});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  // A failed deactivation leaves the node in the active state
  node->deactivate();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  // Shutting down from the active state exercises the shutdown transition
  node->shutdown();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_FINALIZED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
