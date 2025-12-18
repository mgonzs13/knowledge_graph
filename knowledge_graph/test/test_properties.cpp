// Copyright 2025 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include "knowledge_graph/graph/properties.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

using namespace knowledge_graph::graph;
using knowledge_graph_msgs::msg::Content;
using knowledge_graph_msgs::msg::Property;

class PropertiesTest : public ::testing::Test {
protected:
  Properties props;
};

// Constructor Tests
TEST_F(PropertiesTest, DefaultConstructor) {
  EXPECT_FALSE(props.has("nonexistent"));
}

// Basic Type Tests
TEST_F(PropertiesTest, SetAndGetBool) {
  props.set<bool>("flag", true);
  EXPECT_TRUE(props.has("flag"));
  EXPECT_TRUE(props.get<bool>("flag"));
  // type() returns mangled C++ type names from typeid().name()
  EXPECT_EQ(props.type("flag"), typeid(bool).name());
}

TEST_F(PropertiesTest, SetAndGetInt) {
  props.set<int>("count", 42);
  EXPECT_TRUE(props.has("count"));
  EXPECT_EQ(props.get<int>("count"), 42);
  EXPECT_EQ(props.type("count"), typeid(int).name());
}

TEST_F(PropertiesTest, SetAndGetFloat) {
  props.set<float>("ratio", 3.14f);
  EXPECT_TRUE(props.has("ratio"));
  EXPECT_NEAR(props.get<float>("ratio"), 3.14f, 0.001f);
  EXPECT_EQ(props.type("ratio"), typeid(float).name());
}

TEST_F(PropertiesTest, SetAndGetDouble) {
  props.set<double>("value", 2.71828);
  EXPECT_TRUE(props.has("value"));
  EXPECT_NEAR(props.get<double>("value"), 2.71828, 0.00001);
  EXPECT_EQ(props.type("value"), typeid(double).name());
}

TEST_F(PropertiesTest, SetAndGetString) {
  props.set<std::string>("name", "test_value");
  EXPECT_TRUE(props.has("name"));
  EXPECT_EQ(props.get<std::string>("name"), "test_value");
  EXPECT_EQ(props.type("name"), typeid(std::string).name());
}

// Vector Type Tests
TEST_F(PropertiesTest, SetAndGetBoolVector) {
  std::vector<bool> flags = {true, false, true};
  props.set<std::vector<bool>>("flags", flags);
  EXPECT_TRUE(props.has("flags"));
  EXPECT_EQ(props.get<std::vector<bool>>("flags"), flags);
  EXPECT_EQ(props.type("flags"), typeid(std::vector<bool>).name());
}

TEST_F(PropertiesTest, SetAndGetIntVector) {
  std::vector<int> numbers = {1, 2, 3, 4, 5};
  props.set<std::vector<int>>("numbers", numbers);
  EXPECT_TRUE(props.has("numbers"));
  EXPECT_EQ(props.get<std::vector<int>>("numbers"), numbers);
  EXPECT_EQ(props.type("numbers"), typeid(std::vector<int>).name());
}

TEST_F(PropertiesTest, SetAndGetFloatVector) {
  std::vector<float> values = {1.1f, 2.2f, 3.3f};
  props.set<std::vector<float>>("values", values);
  EXPECT_TRUE(props.has("values"));
  auto retrieved = props.get<std::vector<float>>("values");
  ASSERT_EQ(retrieved.size(), values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_NEAR(retrieved[i], values[i], 0.001f);
  }
  EXPECT_EQ(props.type("values"), typeid(std::vector<float>).name());
}

TEST_F(PropertiesTest, SetAndGetDoubleVector) {
  std::vector<double> values = {1.1, 2.2, 3.3};
  props.set<std::vector<double>>("values", values);
  EXPECT_TRUE(props.has("values"));
  auto retrieved = props.get<std::vector<double>>("values");
  ASSERT_EQ(retrieved.size(), values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_NEAR(retrieved[i], values[i], 0.001);
  }
  EXPECT_EQ(props.type("values"), typeid(std::vector<double>).name());
}

TEST_F(PropertiesTest, SetAndGetStringVector) {
  std::vector<std::string> names = {"a", "b", "c"};
  props.set<std::vector<std::string>>("names", names);
  EXPECT_TRUE(props.has("names"));
  EXPECT_EQ(props.get<std::vector<std::string>>("names"), names);
  EXPECT_EQ(props.type("names"), typeid(std::vector<std::string>).name());
}

// Update Tests
TEST_F(PropertiesTest, UpdatePropertySameType) {
  props.set<int>("value", 10);
  props.set<int>("value", 20);
  EXPECT_EQ(props.get<int>("value"), 20);
}

TEST_F(PropertiesTest, UpdatePropertyDifferentTypeThrows) {
  props.set<int>("value", 10);
  EXPECT_THROW(props.set<std::string>("value", "string"), std::runtime_error);
}

// Error Handling Tests
TEST_F(PropertiesTest, GetNonexistentPropertyThrows) {
  EXPECT_THROW(props.get<int>("nonexistent"), std::runtime_error);
}

TEST_F(PropertiesTest, TypeNonexistentPropertyThrows) {
  EXPECT_THROW(props.type("nonexistent"), std::runtime_error);
}

// Message Conversion Tests
TEST_F(PropertiesTest, ToMsgBool) {
  props.set<bool>("flag", true);
  auto msg = props.to_msg("flag");
  EXPECT_EQ(msg.key, "flag");
  EXPECT_EQ(msg.value.type, Content::BOOL);
  EXPECT_TRUE(msg.value.bool_value);
}

TEST_F(PropertiesTest, ToMsgInt) {
  props.set<int>("count", 42);
  auto msg = props.to_msg("count");
  EXPECT_EQ(msg.key, "count");
  EXPECT_EQ(msg.value.type, Content::INT);
  EXPECT_EQ(msg.value.int_value, 42);
}

TEST_F(PropertiesTest, ToMsgFloat) {
  props.set<float>("ratio", 3.14f);
  auto msg = props.to_msg("ratio");
  EXPECT_EQ(msg.key, "ratio");
  EXPECT_EQ(msg.value.type, Content::FLOAT);
  EXPECT_NEAR(msg.value.float_value, 3.14f, 0.001f);
}

TEST_F(PropertiesTest, ToMsgDouble) {
  props.set<double>("value", 2.71828);
  auto msg = props.to_msg("value");
  EXPECT_EQ(msg.key, "value");
  EXPECT_EQ(msg.value.type, Content::DOUBLE);
  EXPECT_NEAR(msg.value.double_value, 2.71828, 0.00001);
}

TEST_F(PropertiesTest, ToMsgString) {
  props.set<std::string>("name", "test");
  auto msg = props.to_msg("name");
  EXPECT_EQ(msg.key, "name");
  EXPECT_EQ(msg.value.type, Content::STRING);
  EXPECT_EQ(msg.value.string_value, "test");
}

TEST_F(PropertiesTest, ToMsgAllProperties) {
  props.set<bool>("flag", true);
  props.set<int>("count", 42);
  auto msgs = props.to_msg();
  EXPECT_EQ(msgs.size(), 2u);
}

TEST_F(PropertiesTest, ConstructorFromMessages) {
  std::vector<Property> prop_msgs;

  Property prop1;
  prop1.key = "flag";
  prop1.value.type = Content::BOOL;
  prop1.value.bool_value = true;
  prop_msgs.push_back(prop1);

  Property prop2;
  prop2.key = "count";
  prop2.value.type = Content::INT;
  prop2.value.int_value = 42;
  prop_msgs.push_back(prop2);

  Property prop3;
  prop3.key = "name";
  prop3.value.type = Content::STRING;
  prop3.value.string_value = "test";
  prop_msgs.push_back(prop3);

  Properties restored(prop_msgs);
  EXPECT_TRUE(restored.get<bool>("flag"));
  EXPECT_EQ(restored.get<int>("count"), 42);
  EXPECT_EQ(restored.get<std::string>("name"), "test");
}

TEST_F(PropertiesTest, ToMsgNonexistentThrows) {
  EXPECT_THROW(props.to_msg("nonexistent"), std::runtime_error);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
