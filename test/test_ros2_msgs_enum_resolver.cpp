// Copyright 2024 Your Name
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

#include <cstdlib>

#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "ros2_msgs_enum_resolver/ros2_msgs_enum_resolver.hpp"

namespace fs = std::filesystem;
using ER = ros2_msgs_enum_resolver::EnumResolver;

class EnumResolverTest : public ::testing::Test
{
protected:
  std::string tmp_dir_;
  std::string orig_prefix_;

  void SetUp() override
  {
    tmp_dir_ = (fs::temp_directory_path() / "er_test").string();
    fs::create_directories(tmp_dir_ + "/share/fake_pkg/msg");
    fs::create_directories(tmp_dir_ + "/share/fake_pkg/srv");
    fs::create_directories(tmp_dir_ + "/share/fake_pkg/action");
    fs::create_directories(tmp_dir_ + "/share/ament_index/resource_index/packages");
    std::ofstream(tmp_dir_ + "/share/ament_index/resource_index/packages/fake_pkg").close();

    write(
      tmp_dir_ + "/share/fake_pkg/msg/State.msg",
      "uint8 NONE      = 0\n"
      "uint8 INIT      = 1\n"
      "uint8 STARTING  = 1\n"
      "uint8 CONNECTED = 2\n"
      "uint8 ERROR     = 255\n"
      "uint8 HEX_VAL   = 0xFF\n"
      "uint8 state\n");

    write(
      tmp_dir_ + "/share/fake_pkg/srv/SetMode.srv",
      "uint8 MODE_A = 10\n"
      "uint8 MODE_B = 20\n"
      "uint8 mode\n"
      "---\n"
      "uint8 STATUS_OK  = 0\n"
      "uint8 STATUS_ERR = 1\n"
      "bool success\n");

    write(
      tmp_dir_ + "/share/fake_pkg/action/Navigate.action",
      "uint8 PRIORITY_LOW  = 0\n"
      "uint8 PRIORITY_HIGH = 1\n"
      "float64 target_x\n"
      "---\n"
      "uint8 RESULT_OK   = 0\n"
      "uint8 RESULT_FAIL = 1\n"
      "bool reached\n"
      "---\n"
      "uint8 FB_MOVING = 0\n"
      "uint8 FB_STUCK  = 1\n"
      "float64 progress\n");

    const char * cur = std::getenv("AMENT_PREFIX_PATH");
    orig_prefix_ = cur ? cur : "";
    std::string np = tmp_dir_ + (orig_prefix_.empty() ? "" : ":" + orig_prefix_);
    setenv("AMENT_PREFIX_PATH", np.c_str(), 1);
    ER::clearCache();
  }

  void TearDown() override
  {
    setenv("AMENT_PREFIX_PATH", orig_prefix_.c_str(), 1);
    ER::clearCache();
    fs::remove_all(tmp_dir_);
  }

  static void write(const std::string & path, const std::string & content)
  {
    std::ofstream f(path);
    f << content;
  }
};

TEST_F(EnumResolverTest, Msg_Forward_Single)
{
  auto v = ER::resolve("fake_pkg/msg/State", 0);
  ASSERT_TRUE(v.has_value());
  ASSERT_EQ(v->size(), 1u);
  EXPECT_EQ((*v)[0], "NONE");
}

TEST_F(EnumResolverTest, Msg_Forward_Alias)
{
  auto v = ER::resolve("fake_pkg/msg/State", 1);
  ASSERT_TRUE(v.has_value());
  ASSERT_EQ(v->size(), 2u);
  EXPECT_EQ((*v)[0], "INIT");
  EXPECT_EQ((*v)[1], "STARTING");
}

TEST_F(EnumResolverTest, Msg_Forward_HexAlias)
{
  auto v = ER::resolve("fake_pkg/msg/State", 255);
  ASSERT_TRUE(v.has_value());
  ASSERT_EQ(v->size(), 2u);
  EXPECT_EQ((*v)[0], "ERROR");
  EXPECT_EQ((*v)[1], "HEX_VAL");
}

TEST_F(EnumResolverTest, Msg_Forward_Missing)
{
  EXPECT_FALSE(ER::resolve("fake_pkg/msg/State", 99).has_value());
}

TEST_F(EnumResolverTest, Msg_Reverse)
{
  EXPECT_EQ(ER::reverseResolve("fake_pkg/msg/State", "NONE"), std::optional<int64_t>(0));
  EXPECT_EQ(ER::reverseResolve("fake_pkg/msg/State", "INIT"), std::optional<int64_t>(1));
  EXPECT_EQ(ER::reverseResolve("fake_pkg/msg/State", "STARTING"), std::optional<int64_t>(1));
}

TEST_F(EnumResolverTest, Msg_Reverse_Missing)
{
  EXPECT_FALSE(ER::reverseResolve("fake_pkg/msg/State", "UNKNOWN").has_value());
}

TEST_F(EnumResolverTest, Msg_GetConstants)
{
  auto c = ER::getConstants("fake_pkg/msg/State");
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->size(), 4u);
  ASSERT_EQ((*c)[1].size(), 2u);
  EXPECT_EQ((*c)[1][0], "INIT");
  EXPECT_EQ((*c)[1][1], "STARTING");
}

TEST_F(EnumResolverTest, IsUnambiguous)
{
  EXPECT_FALSE(ER::isUnambiguous("fake_pkg/msg/State"));
  EXPECT_TRUE(ER::isUnambiguous("fake_pkg/srv/SetMode/Request"));
}

TEST_F(EnumResolverTest, Srv_Request)
{
  auto v = ER::resolve("fake_pkg/srv/SetMode/Request", 10);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "MODE_A");
  EXPECT_FALSE(ER::resolve("fake_pkg/srv/SetMode/Request", 0).has_value());
}

TEST_F(EnumResolverTest, Srv_Response)
{
  auto v = ER::resolve("fake_pkg/srv/SetMode/Response", 0);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "STATUS_OK");
}

TEST_F(EnumResolverTest, Srv_DefaultIsRequest)
{
  auto v = ER::resolve("fake_pkg/srv/SetMode", 10);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "MODE_A");
}

TEST_F(EnumResolverTest, Action_Goal)
{
  auto v = ER::resolve("fake_pkg/action/Navigate/Goal", 0);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "PRIORITY_LOW");
}

TEST_F(EnumResolverTest, Action_Result)
{
  auto v = ER::resolve("fake_pkg/action/Navigate/Result", 1);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "RESULT_FAIL");
}

TEST_F(EnumResolverTest, Action_Feedback)
{
  auto v = ER::resolve("fake_pkg/action/Navigate/Feedback", 1);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "FB_STUCK");
}

TEST_F(EnumResolverTest, IsAvailable)
{
  EXPECT_TRUE(ER::isAvailable("fake_pkg/msg/State"));
  EXPECT_FALSE(ER::isAvailable("ghost_pkg/msg/Whatever"));
}

TEST_F(EnumResolverTest, BadTypeString)
{
  EXPECT_FALSE(ER::isAvailable("no_slashes"));
  EXPECT_FALSE(ER::resolve("no_slashes", 0).has_value());
}

TEST_F(EnumResolverTest, ClearCacheAndReload)
{
  ER::preload("fake_pkg/msg/State");
  ER::clearCache();
  auto v = ER::resolve("fake_pkg/msg/State", 0);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "NONE");
}

TEST_F(EnumResolverTest, Preload)
{
  EXPECT_TRUE(ER::preload("fake_pkg/msg/State"));
  EXPECT_FALSE(ER::preload("ghost_pkg/msg/Ghost"));
}


TEST_F(EnumResolverTest, Resolve_WithFieldType_Found)
{
  // int8 STATUS_* константы, запрашиваем значение 1 с field_type="int8"
  auto v = ER::resolve("fake_pkg/msg/State", 1, "uint8");
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "INIT");
}

TEST_F(EnumResolverTest, Resolve_WithFieldType_WrongType)
{
  // Значение 1 есть, но запрашиваем uint16 — таких констант нет
  auto v = ER::resolve("fake_pkg/msg/State", 1, "uint16");
  EXPECT_FALSE(v.has_value());
}

TEST_F(EnumResolverTest, Resolve_WithFieldType_EmptyIsFallback)
{
  // Пустой field_type — поведение как раньше, все константы
  auto v = ER::resolve("fake_pkg/msg/State", 1, "");
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], "INIT");
}

// ---- getConstantsForPrimitive() ----

TEST_F(EnumResolverTest, GetConstantsForPrimitive_Found)
{
  auto c = ER::getConstantsForPrimitive("fake_pkg/msg/State", "uint8");
  ASSERT_TRUE(c.has_value());
  // State.msg имеет uint8 константы: NONE=0, INIT=1, STARTING=1, ...
  EXPECT_TRUE(c->count(0));
  EXPECT_EQ((*c)[0][0], "NONE");
}

TEST_F(EnumResolverTest, GetConstantsForPrimitive_WrongType)
{
  // State.msg не имеет float32 констант
  auto c = ER::getConstantsForPrimitive("fake_pkg/msg/State", "float32");
  EXPECT_FALSE(c.has_value());
}

// ---- isEnumOnlyType() ----

TEST_F(EnumResolverTest, IsEnumOnlyType_True)
{
  // State.msg содержит только константы и одно поле "uint8 state"
  // Поэтому это НЕ enum-only
  EXPECT_FALSE(ER::isEnumOnlyType("fake_pkg/msg/State"));
}

TEST_F(EnumResolverTest, IsEnumOnlyType_PureEnum)
{
  // Создадим файл только с константами
  write(tmp_dir_ + "/share/fake_pkg/msg/PureEnum.msg",
    "uint8 A = 0\n"
    "uint8 B = 1\n"
    "uint8 C = 2\n");

  EXPECT_TRUE(ER::isEnumOnlyType("fake_pkg/msg/PureEnum"));
}

TEST_F(EnumResolverTest, IsEnumOnlyType_FieldWithDefault)
{
  // Поле с дефолтным значением — это НЕ enum-only
  // Это тест на исправленный sectionHasOnlyConstants
  write(tmp_dir_ + "/share/fake_pkg/msg/WithDefault.msg",
    "uint8 A = 0\n"
    "uint8 B = 1\n"
    "uint8 status -2\n");  // поле с дефолтом

  EXPECT_FALSE(ER::isEnumOnlyType("fake_pkg/msg/WithDefault"));
}

TEST_F(EnumResolverTest, IsEnumOnlyType_MixedInline)
{
  // NavSatStatus-style: константы двух типов + поля
  write(tmp_dir_ + "/share/fake_pkg/msg/NavStyle.msg",
    "int8 STATUS_NO_FIX = -1\n"
    "int8 STATUS_FIX    =  0\n"
    "uint16 SERVICE_GPS = 1\n"
    "int8 status\n"
    "uint16 service\n");

  EXPECT_FALSE(ER::isEnumOnlyType("fake_pkg/msg/NavStyle"));
}

// ---- resolve() с inline-константами двух типов (NavSatStatus паттерн) ----

TEST_F(EnumResolverTest, Resolve_InlineConstants_TwoTypes)
{
  write(tmp_dir_ + "/share/fake_pkg/msg/NavStyle.msg",
    "int8 STATUS_NO_FIX = -1\n"
    "int8 STATUS_FIX    =  0\n"
    "uint16 SERVICE_GPS = 1\n"
    "uint16 SERVICE_GLONASS = 2\n"
    "int8 status\n"
    "uint16 service\n");

  // int8 константы
  auto v1 = ER::resolve("fake_pkg/msg/NavStyle", 0, "int8");
  ASSERT_TRUE(v1.has_value());
  EXPECT_EQ((*v1)[0], "STATUS_FIX");

  auto v2 = ER::resolve("fake_pkg/msg/NavStyle", -1, "int8");
  ASSERT_TRUE(v2.has_value());
  EXPECT_EQ((*v2)[0], "STATUS_NO_FIX");

  // uint16 константы
  auto v3 = ER::resolve("fake_pkg/msg/NavStyle", 1, "uint16");
  ASSERT_TRUE(v3.has_value());
  EXPECT_EQ((*v3)[0], "SERVICE_GPS");

  // Значения нет — просто число
  auto v4 = ER::resolve("fake_pkg/msg/NavStyle", 3, "uint16");
  EXPECT_FALSE(v4.has_value());

  // int8 значение не найдено в uint16 константах
  auto v5 = ER::resolve("fake_pkg/msg/NavStyle", 0, "uint16");
  EXPECT_FALSE(v5.has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}