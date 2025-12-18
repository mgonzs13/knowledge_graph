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

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

using namespace knowledge_graph::graph;
using knowledge_graph_msgs::msg::Content;
using knowledge_graph_msgs::msg::Property;

class EdgeTest : public ::testing::Test {};

// Constructor Tests
TEST_F(EdgeTest, ConstructorWithParameters) {
  Edge edge("connects", "node_a", "node_b");
  EXPECT_EQ(edge.get_type(), "connects");
  EXPECT_EQ(edge.get_source_node(), "node_a");
  EXPECT_EQ(edge.get_target_node(), "node_b");
}

TEST_F(EdgeTest, ConstructorFromMessage) {
  knowledge_graph_msgs::msg::Edge msg;
  msg.type = "relates";
  msg.source_node = "source";
  msg.target_node = "target";

  Edge edge(msg);
  EXPECT_EQ(edge.get_type(), "relates");
  EXPECT_EQ(edge.get_source_node(), "source");
  EXPECT_EQ(edge.get_target_node(), "target");
}

TEST_F(EdgeTest, ConstructorFromMessageWithProperties) {
  Property prop;
  prop.key = "weight";
  prop.value.type = Content::DOUBLE;
  prop.value.double_value = 0.75;

  knowledge_graph_msgs::msg::Edge msg;
  msg.type = "connects";
  msg.source_node = "a";
  msg.target_node = "b";
  msg.properties.push_back(prop);

  Edge edge(msg);
  EXPECT_EQ(edge.get_type(), "connects");
  EXPECT_TRUE(edge.has_property("weight"));
  EXPECT_NEAR(edge.get_property<double>("weight"), 0.75, 0.001);
}

// Message Conversion Tests
TEST_F(EdgeTest, ToMsg) {
  Edge edge("connects", "node_a", "node_b");
  edge.set_property<double>("weight", 1.5);

  auto msg = edge.to_msg();
  EXPECT_EQ(msg.type, "connects");
  EXPECT_EQ(msg.source_node, "node_a");
  EXPECT_EQ(msg.target_node, "node_b");
  EXPECT_EQ(msg.properties.size(), 1u);
  EXPECT_EQ(msg.properties[0].key, "weight");
}

// String Representation Tests
TEST_F(EdgeTest, ToString) {
  Edge edge("connects", "node_a", "node_b");
  auto str = edge.to_string();
  EXPECT_NE(str.find("connects"), std::string::npos);
  EXPECT_NE(str.find("node_a"), std::string::npos);
  EXPECT_NE(str.find("node_b"), std::string::npos);
}

// Property Tests
TEST_F(EdgeTest, SetAndGetProperty) {
  Edge edge("connects", "node_a", "node_b");
  edge.set_property<double>("weight", 0.9);
  edge.set_property<std::string>("label", "connection");
  edge.set_property<bool>("bidirectional", true);

  EXPECT_NEAR(edge.get_property<double>("weight"), 0.9, 0.001);
  EXPECT_EQ(edge.get_property<std::string>("label"), "connection");
  EXPECT_TRUE(edge.get_property<bool>("bidirectional"));
}

TEST_F(EdgeTest, HasProperty) {
  Edge edge("connects", "node_a", "node_b");
  EXPECT_FALSE(edge.has_property("weight"));
  edge.set_property<double>("weight", 0.5);
  EXPECT_TRUE(edge.has_property("weight"));
}

TEST_F(EdgeTest, PropertyRoundtrip) {
  Edge edge("connects", "node_a", "node_b");
  edge.set_property<double>("weight", 0.75);
  edge.set_property<std::string>("label", "test_edge");
  edge.set_property<bool>("enabled", false);

  auto msg = edge.to_msg();
  Edge restored_edge(msg);

  EXPECT_EQ(restored_edge.get_type(), "connects");
  EXPECT_EQ(restored_edge.get_source_node(), "node_a");
  EXPECT_EQ(restored_edge.get_target_node(), "node_b");
  EXPECT_NEAR(restored_edge.get_property<double>("weight"), 0.75, 0.001);
  EXPECT_EQ(restored_edge.get_property<std::string>("label"), "test_edge");
  EXPECT_FALSE(restored_edge.get_property<bool>("enabled"));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
