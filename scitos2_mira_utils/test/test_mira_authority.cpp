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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
