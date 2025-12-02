// Copyright 2023 Miguel Ángel González Santamarta
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

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

namespace knowledge_graph {
namespace test {

/**
 * @brief Test fixture for KnowledgeGraph tests.
 *
 * This fixture sets up and tears down the ROS 2 context and provides
 * a test node for each test case.
 */
class KnowledgeGraphTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp::Node>("test_node");
    graph = std::make_shared<KnowledgeGraph>(node);
  }

  void TearDown() override {
    graph.reset();
    node.reset();
  }

  /// @brief Spins the node briefly to process callbacks
  void spin_some() {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<KnowledgeGraph> graph;
};

// =============================================================================
// Node Operations Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, InitialGraphIsEmpty) {
  EXPECT_EQ(graph->get_num_nodes(), 0u);
  EXPECT_EQ(graph->get_num_edges(), 0u);
  EXPECT_TRUE(graph->get_nodes().empty());
  EXPECT_TRUE(graph->get_edges().empty());
}

TEST_F(KnowledgeGraphTest, AddSingleNode) {
  auto robot = new_node("robot1", "robot");
  bool result = graph->update_node(robot, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(graph->get_num_nodes(), 1u);
  EXPECT_TRUE(graph->exist_node("robot1"));
}

TEST_F(KnowledgeGraphTest, AddMultipleNodes) {
  graph->update_node(new_node("robot1", "robot"), false);
  graph->update_node(new_node("room1", "room"), false);
  graph->update_node(new_node("object1", "object"), false);

  EXPECT_EQ(graph->get_num_nodes(), 3u);
  EXPECT_TRUE(graph->exist_node("robot1"));
  EXPECT_TRUE(graph->exist_node("room1"));
  EXPECT_TRUE(graph->exist_node("object1"));
}

TEST_F(KnowledgeGraphTest, UpdateExistingNode) {
  auto robot = new_node("robot1", "robot");
  add_property<int>(robot, "speed", 10);
  graph->update_node(robot, false);

  // Update with new properties
  auto robot_updated = new_node("robot1", "robot");
  add_property<int>(robot_updated, "speed", 20);
  graph->update_node(robot_updated, false);

  EXPECT_EQ(graph->get_num_nodes(), 1u);

  auto retrieved = graph->get_node("robot1");
  ASSERT_TRUE(retrieved.has_value());

  auto speed = get_property<int>(retrieved.value(), "speed");
  ASSERT_TRUE(speed.has_value());
  EXPECT_EQ(speed.value(), 20);
}

TEST_F(KnowledgeGraphTest, GetNodeThatExists) {
  auto robot = new_node("robot1", "robot");
  add_property<std::string>(robot, "model", "TurtleBot");
  graph->update_node(robot, false);

  auto retrieved = graph->get_node("robot1");

  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved.value().node_name, "robot1");
  EXPECT_EQ(retrieved.value().node_class, "robot");

  auto model = get_property<std::string>(retrieved.value(), "model");
  ASSERT_TRUE(model.has_value());
  EXPECT_EQ(model.value(), "TurtleBot");
}

TEST_F(KnowledgeGraphTest, GetNodeThatDoesNotExist) {
  auto retrieved = graph->get_node("nonexistent");

  EXPECT_FALSE(retrieved.has_value());
}

TEST_F(KnowledgeGraphTest, ExistNodePositive) {
  graph->update_node(new_node("robot1", "robot"), false);

  EXPECT_TRUE(graph->exist_node("robot1"));
}

TEST_F(KnowledgeGraphTest, ExistNodeNegative) {
  EXPECT_FALSE(graph->exist_node("nonexistent"));
}

TEST_F(KnowledgeGraphTest, RemoveExistingNode) {
  graph->update_node(new_node("robot1", "robot"), false);
  ASSERT_TRUE(graph->exist_node("robot1"));

  bool result = graph->remove_node("robot1", false);

  EXPECT_TRUE(result);
  EXPECT_FALSE(graph->exist_node("robot1"));
  EXPECT_EQ(graph->get_num_nodes(), 0u);
}

TEST_F(KnowledgeGraphTest, RemoveNonExistingNode) {
  bool result = graph->remove_node("nonexistent", false);

  EXPECT_FALSE(result);
}

TEST_F(KnowledgeGraphTest, RemoveNodeRemovesConnectedEdges) {
  // Create nodes
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);
  graph->update_node(new_node("node_c", "type"), false);

  // Create edges
  graph->update_edge(new_edge("connects", "node_a", "node_b"), false);
  graph->update_edge(new_edge("connects", "node_b", "node_c"), false);
  graph->update_edge(new_edge("connects", "node_a", "node_c"), false);

  ASSERT_EQ(graph->get_num_edges(), 3u);

  // Remove node_b
  graph->remove_node("node_b", false);

  // Only edge from node_a to node_c should remain
  EXPECT_EQ(graph->get_num_edges(), 1u);
  EXPECT_EQ(graph->get_num_nodes(), 2u);

  auto remaining_edges = graph->get_edges("node_a", "node_c");
  EXPECT_EQ(remaining_edges.size(), 1u);
}

TEST_F(KnowledgeGraphTest, GetNodeNames) {
  graph->update_node(new_node("alpha", "type"), false);
  graph->update_node(new_node("beta", "type"), false);
  graph->update_node(new_node("gamma", "type"), false);

  auto names = graph->get_node_names();

  EXPECT_EQ(names.size(), 3u);
  EXPECT_TRUE(std::find(names.begin(), names.end(), "alpha") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "beta") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "gamma") != names.end());
}

TEST_F(KnowledgeGraphTest, GetAllNodes) {
  graph->update_node(new_node("node1", "type_a"), false);
  graph->update_node(new_node("node2", "type_b"), false);

  auto nodes = graph->get_nodes();

  EXPECT_EQ(nodes.size(), 2u);
}

// =============================================================================
// Edge Operations Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, AddEdgeBetweenExistingNodes) {
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph->update_edge(edge, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(graph->get_num_edges(), 1u);
}

TEST_F(KnowledgeGraphTest, AddEdgeWithNonExistentSource) {
  graph->update_node(new_node("node_b", "type"), false);

  auto edge = new_edge("connects", "nonexistent", "node_b");
  bool result = graph->update_edge(edge, false);

  EXPECT_FALSE(result);
  EXPECT_EQ(graph->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, AddEdgeWithNonExistentTarget) {
  graph->update_node(new_node("node_a", "type"), false);

  auto edge = new_edge("connects", "node_a", "nonexistent");
  bool result = graph->update_edge(edge, false);

  EXPECT_FALSE(result);
  EXPECT_EQ(graph->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, AddMultipleEdgesSameNodes) {
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);

  graph->update_edge(new_edge("edge_type_1", "node_a", "node_b"), false);
  graph->update_edge(new_edge("edge_type_2", "node_a", "node_b"), false);

  EXPECT_EQ(graph->get_num_edges(), 2u);

  auto edges = graph->get_edges("node_a", "node_b");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, UpdateExistingEdge) {
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);

  auto edge1 = new_edge("connects", "node_a", "node_b");
  add_property<int>(edge1, "weight", 10);
  graph->update_edge(edge1, false);

  auto edge2 = new_edge("connects", "node_a", "node_b");
  add_property<int>(edge2, "weight", 20);
  graph->update_edge(edge2, false);

  EXPECT_EQ(graph->get_num_edges(), 1u);

  auto edges = graph->get_edges("node_a", "node_b");
  ASSERT_EQ(edges.size(), 1u);

  auto weight = get_property<int>(edges[0], "weight");
  ASSERT_TRUE(weight.has_value());
  EXPECT_EQ(weight.value(), 20);
}

TEST_F(KnowledgeGraphTest, RemoveExistingEdge) {
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);
  graph->update_edge(new_edge("connects", "node_a", "node_b"), false);

  ASSERT_EQ(graph->get_num_edges(), 1u);

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph->remove_edge(edge, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(graph->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, RemoveNonExistingEdge) {
  graph->update_node(new_node("node_a", "type"), false);
  graph->update_node(new_node("node_b", "type"), false);

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph->remove_edge(edge, false);

  EXPECT_FALSE(result);
}

TEST_F(KnowledgeGraphTest, GetEdgesBySourceAndTarget) {
  graph->update_node(new_node("a", "type"), false);
  graph->update_node(new_node("b", "type"), false);
  graph->update_node(new_node("c", "type"), false);

  graph->update_edge(new_edge("e1", "a", "b"), false);
  graph->update_edge(new_edge("e2", "a", "b"), false);
  graph->update_edge(new_edge("e3", "a", "c"), false);

  auto edges_ab = graph->get_edges("a", "b");
  auto edges_ac = graph->get_edges("a", "c");
  auto edges_bc = graph->get_edges("b", "c");

  EXPECT_EQ(edges_ab.size(), 2u);
  EXPECT_EQ(edges_ac.size(), 1u);
  EXPECT_EQ(edges_bc.size(), 0u);
}

TEST_F(KnowledgeGraphTest, GetEdgesByClass) {
  graph->update_node(new_node("a", "type"), false);
  graph->update_node(new_node("b", "type"), false);
  graph->update_node(new_node("c", "type"), false);

  graph->update_edge(new_edge("connects", "a", "b"), false);
  graph->update_edge(new_edge("connects", "b", "c"), false);
  graph->update_edge(new_edge("contains", "a", "c"), false);

  auto connects_edges = graph->get_edges("connects");
  auto contains_edges = graph->get_edges("contains");
  auto other_edges = graph->get_edges("other");

  EXPECT_EQ(connects_edges.size(), 2u);
  EXPECT_EQ(contains_edges.size(), 1u);
  EXPECT_EQ(other_edges.size(), 0u);
}

TEST_F(KnowledgeGraphTest, GetOutEdges) {
  graph->update_node(new_node("center", "type"), false);
  graph->update_node(new_node("target1", "type"), false);
  graph->update_node(new_node("target2", "type"), false);
  graph->update_node(new_node("source", "type"), false);

  graph->update_edge(new_edge("out1", "center", "target1"), false);
  graph->update_edge(new_edge("out2", "center", "target2"), false);
  graph->update_edge(new_edge("in1", "source", "center"), false);

  auto out_edges = graph->get_out_edges("center");

  EXPECT_EQ(out_edges.size(), 2u);
  for (const auto &edge : out_edges) {
    EXPECT_EQ(edge.source_node, "center");
  }
}

TEST_F(KnowledgeGraphTest, GetInEdges) {
  graph->update_node(new_node("center", "type"), false);
  graph->update_node(new_node("source1", "type"), false);
  graph->update_node(new_node("source2", "type"), false);
  graph->update_node(new_node("target", "type"), false);

  graph->update_edge(new_edge("in1", "source1", "center"), false);
  graph->update_edge(new_edge("in2", "source2", "center"), false);
  graph->update_edge(new_edge("out1", "center", "target"), false);

  auto in_edges = graph->get_in_edges("center");

  EXPECT_EQ(in_edges.size(), 2u);
  for (const auto &edge : in_edges) {
    EXPECT_EQ(edge.target_node, "center");
  }
}

TEST_F(KnowledgeGraphTest, GetAllEdges) {
  graph->update_node(new_node("a", "type"), false);
  graph->update_node(new_node("b", "type"), false);

  graph->update_edge(new_edge("e1", "a", "b"), false);
  graph->update_edge(new_edge("e2", "b", "a"), false);

  auto all_edges = graph->get_edges();

  EXPECT_EQ(all_edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, SelfReferencingEdge) {
  graph->update_node(new_node("node", "type"), false);

  auto edge = new_edge("self_loop", "node", "node");
  bool result = graph->update_edge(edge, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(graph->get_num_edges(), 1u);

  auto out_edges = graph->get_out_edges("node");
  auto in_edges = graph->get_in_edges("node");

  EXPECT_EQ(out_edges.size(), 1u);
  EXPECT_EQ(in_edges.size(), 1u);
}

// =============================================================================
// Graph Size Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, GetNumNodesAfterOperations) {
  EXPECT_EQ(graph->get_num_nodes(), 0u);

  graph->update_node(new_node("n1", "type"), false);
  EXPECT_EQ(graph->get_num_nodes(), 1u);

  graph->update_node(new_node("n2", "type"), false);
  EXPECT_EQ(graph->get_num_nodes(), 2u);

  graph->remove_node("n1", false);
  EXPECT_EQ(graph->get_num_nodes(), 1u);

  graph->remove_node("n2", false);
  EXPECT_EQ(graph->get_num_nodes(), 0u);
}

TEST_F(KnowledgeGraphTest, GetNumEdgesAfterOperations) {
  graph->update_node(new_node("a", "type"), false);
  graph->update_node(new_node("b", "type"), false);

  EXPECT_EQ(graph->get_num_edges(), 0u);

  graph->update_edge(new_edge("e1", "a", "b"), false);
  EXPECT_EQ(graph->get_num_edges(), 1u);

  graph->update_edge(new_edge("e2", "a", "b"), false);
  EXPECT_EQ(graph->get_num_edges(), 2u);

  graph->remove_edge(new_edge("e1", "a", "b"), false);
  EXPECT_EQ(graph->get_num_edges(), 1u);
}

// =============================================================================
// Complex Graph Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, BuildComplexGraph) {
  // Create a more complex graph structure
  // Rooms
  graph->update_node(new_node("living_room", "room"), false);
  graph->update_node(new_node("kitchen", "room"), false);
  graph->update_node(new_node("bedroom", "room"), false);

  // Objects
  graph->update_node(new_node("table", "object"), false);
  graph->update_node(new_node("chair", "object"), false);
  graph->update_node(new_node("fridge", "object"), false);

  // Robot
  auto robot = new_node("turtlebot", "robot");
  add_property<double>(robot, "battery", 85.5);
  add_property<std::string>(robot, "status", "idle");
  graph->update_node(robot, false);

  // Room connections
  graph->update_edge(new_edge("connected_to", "living_room", "kitchen"), false);
  graph->update_edge(new_edge("connected_to", "living_room", "bedroom"), false);

  // Object locations
  graph->update_edge(new_edge("is_in", "table", "living_room"), false);
  graph->update_edge(new_edge("is_in", "chair", "living_room"), false);
  graph->update_edge(new_edge("is_in", "fridge", "kitchen"), false);

  // Robot location
  graph->update_edge(new_edge("is_at", "turtlebot", "living_room"), false);

  // Verify structure
  EXPECT_EQ(graph->get_num_nodes(), 7u);
  EXPECT_EQ(graph->get_num_edges(), 6u);

  // Check robot properties
  auto retrieved_robot = graph->get_node("turtlebot");
  ASSERT_TRUE(retrieved_robot.has_value());

  auto battery = get_property<double>(retrieved_robot.value(), "battery");
  ASSERT_TRUE(battery.has_value());
  EXPECT_DOUBLE_EQ(battery.value(), 85.5);

  // Check connections from living room
  auto living_room_connections = graph->get_out_edges("living_room");
  EXPECT_EQ(living_room_connections.size(), 2u);

  // Check objects in living room
  auto objects_in_living = graph->get_in_edges("living_room");
  EXPECT_GE(objects_in_living.size(), 2u); // table, chair, and robot
}

} // namespace test
} // namespace knowledge_graph

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
