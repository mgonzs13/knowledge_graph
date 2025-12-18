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

#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

using namespace knowledge_graph::graph;
using knowledge_graph_msgs::msg::Content;
using knowledge_graph_msgs::msg::Property;

class NodeTest : public ::testing::Test {};

// Constructor Tests
TEST_F(NodeTest, ConstructorWithNameAndType) {
  Node node("robot", "robot_type");
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_EQ(node.get_type(), "robot_type");
}

TEST_F(NodeTest, ConstructorFromMessage) {
  knowledge_graph_msgs::msg::Node msg;
  msg.name = "sensor";
  msg.type = "sensor_type";

  Node node(msg);
  EXPECT_EQ(node.get_name(), "sensor");
  EXPECT_EQ(node.get_type(), "sensor_type");
}

TEST_F(NodeTest, ConstructorFromMessageWithProperties) {
  Property prop;
  prop.key = "active";
  prop.value.type = Content::BOOL;
  prop.value.bool_value = true;

  knowledge_graph_msgs::msg::Node msg;
  msg.name = "robot";
  msg.type = "robot_type";
  msg.properties.push_back(prop);

  Node node(msg);
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_TRUE(node.has_property("active"));
  EXPECT_TRUE(node.get_property<bool>("active"));
}

// Message Conversion Tests
TEST_F(NodeTest, ToMsg) {
  Node node("robot", "robot_type");
  node.set_property<double>("speed", 1.5);

  auto msg = node.to_msg();
  EXPECT_EQ(msg.name, "robot");
  EXPECT_EQ(msg.type, "robot_type");
  EXPECT_EQ(msg.properties.size(), 1u);
  EXPECT_EQ(msg.properties[0].key, "speed");
}

// String Representation Tests
TEST_F(NodeTest, ToString) {
  Node node("robot", "robot_type");
  auto str = node.to_string();
  EXPECT_NE(str.find("robot"), std::string::npos);
  EXPECT_NE(str.find("robot_type"), std::string::npos);
}

// Property Tests
TEST_F(NodeTest, SetAndGetProperty) {
  Node node("robot", "robot_type");
  node.set_property<double>("speed", 10.5);
  node.set_property<std::string>("name", "R2D2");
  node.set_property<bool>("active", true);

  EXPECT_NEAR(node.get_property<double>("speed"), 10.5, 0.001);
  EXPECT_EQ(node.get_property<std::string>("name"), "R2D2");
  EXPECT_TRUE(node.get_property<bool>("active"));
}

TEST_F(NodeTest, HasProperty) {
  Node node("robot", "robot_type");
  EXPECT_FALSE(node.has_property("speed"));
  node.set_property<double>("speed", 10.5);
  EXPECT_TRUE(node.has_property("speed"));
}

TEST_F(NodeTest, PropertyRoundtrip) {
  Node node("robot", "robot_type");
  node.set_property<double>("speed", 10.5);
  node.set_property<std::string>("name", "R2D2");
  node.set_property<bool>("active", true);
  node.set_property<int>("count", 42);

  auto msg = node.to_msg();
  Node restored_node(msg);

  EXPECT_EQ(restored_node.get_name(), "robot");
  EXPECT_EQ(restored_node.get_type(), "robot_type");
  EXPECT_NEAR(restored_node.get_property<double>("speed"), 10.5, 0.001);
  EXPECT_EQ(restored_node.get_property<std::string>("name"), "R2D2");
  EXPECT_TRUE(restored_node.get_property<bool>("active"));
  EXPECT_EQ(restored_node.get_property<int>("count"), 42);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
