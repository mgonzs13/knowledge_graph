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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

namespace knowledge_graph {
namespace test {

/**
 * @brief Test fixture for KnowledgeGraph tests.
 *
 * This fixture sets up and tears down the ROS 2 context and provides
 * two test nodes and graphs for distributed testing.
 */
class KnowledgeGraphTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    node1 = std::make_shared<rclcpp::Node>("test_node1");
    node2 = std::make_shared<rclcpp::Node>("test_node2");
    graph1 = std::make_shared<KnowledgeGraph>(node1);
    graph2 = std::make_shared<KnowledgeGraph>(node2);

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node1);
    executor->add_node(node2);

    spinning = true;
    spin_thread = std::thread([this]() {
      while (spinning && rclcpp::ok()) {
        executor->spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  void TearDown() override {
    spinning = false;
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor.reset();
    graph1.reset();
    graph2.reset();
    node1.reset();
    node2.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node1;
  rclcpp::Node::SharedPtr node2;
  std::shared_ptr<KnowledgeGraph> graph1;
  std::shared_ptr<KnowledgeGraph> graph2;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;
  std::atomic<bool> spinning;
};

// =============================================================================
// Node Operations Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, InitialGraphIsEmpty) {
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_nodes(), 0u);
  EXPECT_EQ(graph1->get_num_edges(), 0u);
  EXPECT_TRUE(graph1->get_nodes().empty());
  EXPECT_TRUE(graph1->get_edges().empty());
  EXPECT_EQ(graph2->get_num_nodes(), 0u);
  EXPECT_EQ(graph2->get_num_edges(), 0u);
  EXPECT_TRUE(graph2->get_nodes().empty());
  EXPECT_TRUE(graph2->get_edges().empty());
}

TEST_F(KnowledgeGraphTest, AddSingleNode) {
  auto robot = new_node("robot1", "robot");
  bool result = graph1->update_node(robot, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(result);
  EXPECT_EQ(graph1->get_num_nodes(), 1u);
  EXPECT_TRUE(graph1->exist_node("robot1"));
  EXPECT_EQ(graph2->get_num_nodes(), 1u);
  EXPECT_TRUE(graph2->exist_node("robot1"));
}

TEST_F(KnowledgeGraphTest, AddMultipleNodes) {
  graph1->update_node(new_node("robot1", "robot"), true);
  graph1->update_node(new_node("room1", "room"), true);
  graph1->update_node(new_node("object1", "object"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_EQ(graph1->get_num_nodes(), 3u);
  EXPECT_TRUE(graph1->exist_node("robot1"));
  EXPECT_TRUE(graph1->exist_node("room1"));
  EXPECT_TRUE(graph1->exist_node("object1"));
  EXPECT_EQ(graph2->get_num_nodes(), 3u);
  EXPECT_TRUE(graph2->exist_node("robot1"));
  EXPECT_TRUE(graph2->exist_node("room1"));
  EXPECT_TRUE(graph2->exist_node("object1"));
}

TEST_F(KnowledgeGraphTest, UpdateExistingNode) {
  auto robot = new_node("robot1", "robot");
  add_property<int>(robot, "speed", 10);
  graph1->update_node(robot, true);

  // Update with new properties
  auto robot_updated = new_node("robot1", "robot");
  add_property<int>(robot_updated, "speed", 20);
  graph1->update_node(robot_updated, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_EQ(graph1->get_num_nodes(), 1u);

  auto retrieved = graph1->get_node("robot1");
  ASSERT_TRUE(retrieved.has_value());

  auto speed = get_property<int>(retrieved.value(), "speed");
  ASSERT_TRUE(speed.has_value());
  EXPECT_EQ(speed.value(), 20);

  auto retrieved2 = graph2->get_node("robot1");
  ASSERT_TRUE(retrieved2.has_value());

  auto speed2 = get_property<int>(retrieved2.value(), "speed");
  ASSERT_TRUE(speed2.has_value());
  EXPECT_EQ(speed2.value(), 20);
}

TEST_F(KnowledgeGraphTest, GetNodeThatExists) {
  auto robot = new_node("robot1", "robot");
  add_property<std::string>(robot, "model", "TurtleBot");
  graph1->update_node(robot, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto retrieved = graph1->get_node("robot1");

  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved.value().node_name, "robot1");
  EXPECT_EQ(retrieved.value().node_class, "robot");

  auto model = get_property<std::string>(retrieved.value(), "model");
  ASSERT_TRUE(model.has_value());
  EXPECT_EQ(model.value(), "TurtleBot");

  auto retrieved2 = graph2->get_node("robot1");

  ASSERT_TRUE(retrieved2.has_value());
  EXPECT_EQ(retrieved2.value().node_name, "robot1");
  EXPECT_EQ(retrieved2.value().node_class, "robot");

  auto model2 = get_property<std::string>(retrieved2.value(), "model");
  ASSERT_TRUE(model2.has_value());
  EXPECT_EQ(model2.value(), "TurtleBot");
}

TEST_F(KnowledgeGraphTest, GetNodeThatDoesNotExist) {
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  auto retrieved = graph1->get_node("nonexistent");

  EXPECT_FALSE(retrieved.has_value());

  auto retrieved2 = graph2->get_node("nonexistent");

  EXPECT_FALSE(retrieved2.has_value());
}

TEST_F(KnowledgeGraphTest, ExistNodePositive) {
  graph1->update_node(new_node("robot1", "robot"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(graph1->exist_node("robot1"));
  EXPECT_TRUE(graph2->exist_node("robot1"));
}

TEST_F(KnowledgeGraphTest, ExistNodeNegative) {
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_FALSE(graph1->exist_node("nonexistent"));
  EXPECT_FALSE(graph2->exist_node("nonexistent"));
}

TEST_F(KnowledgeGraphTest, RemoveExistingNode) {
  graph1->update_node(new_node("robot1", "robot"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  ASSERT_TRUE(graph1->exist_node("robot1"));
  ASSERT_TRUE(graph2->exist_node("robot1"));

  bool result = graph1->remove_node("robot1", true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(result);
  EXPECT_FALSE(graph1->exist_node("robot1"));
  EXPECT_EQ(graph1->get_num_nodes(), 0u);
  EXPECT_FALSE(graph2->exist_node("robot1"));
  EXPECT_EQ(graph2->get_num_nodes(), 0u);
}

TEST_F(KnowledgeGraphTest, RemoveNonExistingNode) {
  bool result = graph1->remove_node("nonexistent", true);

  EXPECT_FALSE(result);
}

TEST_F(KnowledgeGraphTest, RemoveNodeRemovesConnectedEdges) {
  // Create nodes
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);
  graph1->update_node(new_node("node_c", "type"), true);

  // Create edges
  graph1->update_edge(new_edge("connects", "node_a", "node_b"), true);
  graph1->update_edge(new_edge("connects", "node_b", "node_c"), true);
  graph1->update_edge(new_edge("connects", "node_a", "node_c"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  ASSERT_EQ(graph1->get_num_edges(), 3u);
  ASSERT_EQ(graph2->get_num_edges(), 3u);

  // Remove node_b
  graph1->remove_node("node_b", true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  // Only edge from node_a to node_c should remain
  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph1->get_num_nodes(), 2u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_nodes(), 2u);

  auto remaining_edges = graph1->get_edges("node_a", "node_c");
  EXPECT_EQ(remaining_edges.size(), 1u);
  auto remaining_edges2 = graph2->get_edges("node_a", "node_c");
  EXPECT_EQ(remaining_edges2.size(), 1u);
}

TEST_F(KnowledgeGraphTest, GetNodeNames) {
  graph1->update_node(new_node("alpha", "type"), true);
  graph1->update_node(new_node("beta", "type"), true);
  graph1->update_node(new_node("gamma", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto names = graph1->get_node_names();

  EXPECT_EQ(names.size(), 3u);
  EXPECT_TRUE(std::find(names.begin(), names.end(), "alpha") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "beta") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "gamma") != names.end());

  auto names2 = graph2->get_node_names();

  EXPECT_EQ(names2.size(), 3u);
  EXPECT_TRUE(std::find(names2.begin(), names2.end(), "alpha") != names2.end());
  EXPECT_TRUE(std::find(names2.begin(), names2.end(), "beta") != names2.end());
  EXPECT_TRUE(std::find(names2.begin(), names2.end(), "gamma") != names2.end());
}

TEST_F(KnowledgeGraphTest, GetAllNodes) {
  graph1->update_node(new_node("node1", "type_a"), true);
  graph1->update_node(new_node("node2", "type_b"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto nodes = graph1->get_nodes();

  EXPECT_EQ(nodes.size(), 2u);

  auto nodes2 = graph2->get_nodes();

  EXPECT_EQ(nodes2.size(), 2u);
}

// =============================================================================
// Edge Operations Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, AddEdgeBetweenExistingNodes) {
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph1->update_edge(edge, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(result);
  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);
}

TEST_F(KnowledgeGraphTest, AddEdgeWithNonExistentSource) {
  graph1->update_node(new_node("node_b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge = new_edge("connects", "nonexistent", "node_b");
  bool result = graph1->update_edge(edge, true);

  EXPECT_FALSE(result);
  EXPECT_EQ(graph1->get_num_edges(), 0u);
  EXPECT_EQ(graph2->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, AddEdgeWithNonExistentTarget) {
  graph1->update_node(new_node("node_a", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge = new_edge("connects", "node_a", "nonexistent");
  bool result = graph1->update_edge(edge, true);

  EXPECT_FALSE(result);
  EXPECT_EQ(graph1->get_num_edges(), 0u);
  EXPECT_EQ(graph2->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, AddMultipleEdgesSameNodes) {
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  graph1->update_edge(new_edge("edge_type_1", "node_a", "node_b"), true);
  graph1->update_edge(new_edge("edge_type_2", "node_a", "node_b"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_EQ(graph1->get_num_edges(), 2u);
  EXPECT_EQ(graph2->get_num_edges(), 2u);

  auto edges = graph1->get_edges("node_a", "node_b");
  EXPECT_EQ(edges.size(), 2u);
  auto edges2 = graph2->get_edges("node_a", "node_b");
  EXPECT_EQ(edges2.size(), 2u);
}

TEST_F(KnowledgeGraphTest, UpdateExistingEdge) {
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge1 = new_edge("connects", "node_a", "node_b");
  add_property<int>(edge1, "weight", 10);
  graph1->update_edge(edge1, true);

  auto edge2 = new_edge("connects", "node_a", "node_b");
  add_property<int>(edge2, "weight", 20);
  graph1->update_edge(edge2, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);

  auto edges = graph1->get_edges("node_a", "node_b");
  ASSERT_EQ(edges.size(), 1u);

  auto weight = get_property<int>(edges[0], "weight");
  ASSERT_TRUE(weight.has_value());
  EXPECT_EQ(weight.value(), 20);

  auto edges2 = graph2->get_edges("node_a", "node_b");
  ASSERT_EQ(edges2.size(), 1u);

  auto weight2 = get_property<int>(edges2[0], "weight");
  ASSERT_TRUE(weight2.has_value());
  EXPECT_EQ(weight2.value(), 20);
}

TEST_F(KnowledgeGraphTest, RemoveExistingEdge) {
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);
  graph1->update_edge(new_edge("connects", "node_a", "node_b"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  ASSERT_EQ(graph1->get_num_edges(), 1u);
  ASSERT_EQ(graph2->get_num_edges(), 1u);

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph1->remove_edge(edge, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(result);
  EXPECT_EQ(graph1->get_num_edges(), 0u);
  EXPECT_EQ(graph2->get_num_edges(), 0u);
}

TEST_F(KnowledgeGraphTest, RemoveNonExistingEdge) {
  graph1->update_node(new_node("node_a", "type"), true);
  graph1->update_node(new_node("node_b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge = new_edge("connects", "node_a", "node_b");
  bool result = graph1->remove_edge(edge, true);

  EXPECT_FALSE(result);
}

TEST_F(KnowledgeGraphTest, GetEdgesBySourceAndTarget) {
  graph1->update_node(new_node("a", "type"), true);
  graph1->update_node(new_node("b", "type"), true);
  graph1->update_node(new_node("c", "type"), true);

  graph1->update_edge(new_edge("e1", "a", "b"), true);
  graph1->update_edge(new_edge("e2", "a", "b"), true);
  graph1->update_edge(new_edge("e3", "a", "c"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edges_ab = graph1->get_edges("a", "b");
  auto edges_ac = graph1->get_edges("a", "c");
  auto edges_bc = graph1->get_edges("b", "c");

  EXPECT_EQ(edges_ab.size(), 2u);
  EXPECT_EQ(edges_ac.size(), 1u);
  EXPECT_EQ(edges_bc.size(), 0u);

  auto edges_ab2 = graph2->get_edges("a", "b");
  auto edges_ac2 = graph2->get_edges("a", "c");
  auto edges_bc2 = graph2->get_edges("b", "c");

  EXPECT_EQ(edges_ab2.size(), 2u);
  EXPECT_EQ(edges_ac2.size(), 1u);
  EXPECT_EQ(edges_bc2.size(), 0u);
}

TEST_F(KnowledgeGraphTest, GetEdgesByClass) {
  graph1->update_node(new_node("a", "type"), true);
  graph1->update_node(new_node("b", "type"), true);
  graph1->update_node(new_node("c", "type"), true);

  graph1->update_edge(new_edge("connects", "a", "b"), true);
  graph1->update_edge(new_edge("connects", "b", "c"), true);
  graph1->update_edge(new_edge("contains", "a", "c"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto connects_edges = graph1->get_edges("connects");
  auto contains_edges = graph1->get_edges("contains");
  auto other_edges = graph1->get_edges("other");

  EXPECT_EQ(connects_edges.size(), 2u);
  EXPECT_EQ(contains_edges.size(), 1u);
  EXPECT_EQ(other_edges.size(), 0u);

  auto connects_edges2 = graph2->get_edges("connects");
  auto contains_edges2 = graph2->get_edges("contains");
  auto other_edges2 = graph2->get_edges("other");

  EXPECT_EQ(connects_edges2.size(), 2u);
  EXPECT_EQ(contains_edges2.size(), 1u);
  EXPECT_EQ(other_edges2.size(), 0u);
}

TEST_F(KnowledgeGraphTest, GetOutEdges) {
  graph1->update_node(new_node("center", "type"), true);
  graph1->update_node(new_node("target1", "type"), true);
  graph1->update_node(new_node("target2", "type"), true);
  graph1->update_node(new_node("source", "type"), true);

  graph1->update_edge(new_edge("out1", "center", "target1"), true);
  graph1->update_edge(new_edge("out2", "center", "target2"), true);
  graph1->update_edge(new_edge("in1", "source", "center"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto out_edges = graph1->get_out_edges("center");

  EXPECT_EQ(out_edges.size(), 2u);
  for (const auto &edge : out_edges) {
    EXPECT_EQ(edge.source_node, "center");
  }

  auto out_edges2 = graph2->get_out_edges("center");

  EXPECT_EQ(out_edges2.size(), 2u);
  for (const auto &edge : out_edges2) {
    EXPECT_EQ(edge.source_node, "center");
  }
}

TEST_F(KnowledgeGraphTest, GetInEdges) {
  graph1->update_node(new_node("center", "type"), true);
  graph1->update_node(new_node("source1", "type"), true);
  graph1->update_node(new_node("source2", "type"), true);
  graph1->update_node(new_node("target", "type"), true);

  graph1->update_edge(new_edge("in1", "source1", "center"), true);
  graph1->update_edge(new_edge("in2", "source2", "center"), true);
  graph1->update_edge(new_edge("out1", "center", "target"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto in_edges = graph1->get_in_edges("center");

  EXPECT_EQ(in_edges.size(), 2u);
  for (const auto &edge : in_edges) {
    EXPECT_EQ(edge.target_node, "center");
  }

  auto in_edges2 = graph2->get_in_edges("center");

  EXPECT_EQ(in_edges2.size(), 2u);
  for (const auto &edge : in_edges2) {
    EXPECT_EQ(edge.target_node, "center");
  }
}

TEST_F(KnowledgeGraphTest, GetAllEdges) {
  graph1->update_node(new_node("a", "type"), true);
  graph1->update_node(new_node("b", "type"), true);

  graph1->update_edge(new_edge("e1", "a", "b"), true);
  graph1->update_edge(new_edge("e2", "b", "a"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto all_edges = graph1->get_edges();

  EXPECT_EQ(all_edges.size(), 2u);

  auto all_edges2 = graph2->get_edges();

  EXPECT_EQ(all_edges2.size(), 2u);
}

TEST_F(KnowledgeGraphTest, SelfReferencingEdge) {
  graph1->update_node(new_node("node", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  auto edge = new_edge("self_loop", "node", "node");
  bool result = graph1->update_edge(edge, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_TRUE(result);
  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);

  auto out_edges = graph1->get_out_edges("node");
  auto in_edges = graph1->get_in_edges("node");

  EXPECT_EQ(out_edges.size(), 1u);
  EXPECT_EQ(in_edges.size(), 1u);

  auto out_edges2 = graph2->get_out_edges("node");
  auto in_edges2 = graph2->get_in_edges("node");

  EXPECT_EQ(out_edges2.size(), 1u);
  EXPECT_EQ(in_edges2.size(), 1u);
}

// =============================================================================
// Graph Size Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, GetNumNodesAfterOperations) {
  EXPECT_EQ(graph1->get_num_nodes(), 0u);
  EXPECT_EQ(graph2->get_num_nodes(), 0u);

  graph1->update_node(new_node("n1", "type"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_nodes(), 1u);
  EXPECT_EQ(graph2->get_num_nodes(), 1u);

  graph1->update_node(new_node("n2", "type"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_nodes(), 2u);
  EXPECT_EQ(graph2->get_num_nodes(), 2u);

  graph1->remove_node("n1", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_nodes(), 1u);
  EXPECT_EQ(graph2->get_num_nodes(), 1u);

  graph1->remove_node("n2", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_nodes(), 0u);
  EXPECT_EQ(graph2->get_num_nodes(), 0u);
}

TEST_F(KnowledgeGraphTest, GetNumEdgesAfterOperations) {
  graph1->update_node(new_node("a", "type"), true);
  graph1->update_node(new_node("b", "type"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync

  EXPECT_EQ(graph1->get_num_edges(), 0u);
  EXPECT_EQ(graph2->get_num_edges(), 0u);

  graph1->update_edge(new_edge("e1", "a", "b"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);

  graph1->update_edge(new_edge("e2", "a", "b"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_edges(), 2u);
  EXPECT_EQ(graph2->get_num_edges(), 2u);

  graph1->remove_edge(new_edge("e1", "a", "b"), true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow sync
  EXPECT_EQ(graph1->get_num_edges(), 1u);
  EXPECT_EQ(graph2->get_num_edges(), 1u);
}

// =============================================================================
// Complex Graph Tests
// =============================================================================

TEST_F(KnowledgeGraphTest, BuildComplexGraph) {
  // Create a more complex graph structure
  // Rooms
  graph1->update_node(new_node("living_room", "room"), true);
  graph1->update_node(new_node("kitchen", "room"), true);
  graph1->update_node(new_node("bedroom", "room"), true);

  // Objects
  graph1->update_node(new_node("table", "object"), true);
  graph1->update_node(new_node("chair", "object"), true);
  graph1->update_node(new_node("fridge", "object"), true);

  // Robot
  auto robot = new_node("turtlebot", "robot");
  add_property<double>(robot, "battery", 85.5);
  add_property<std::string>(robot, "status", "idle");
  graph1->update_node(robot, true);

  // Room connections
  graph1->update_edge(new_edge("connected_to", "living_room", "kitchen"),
                      false);
  graph1->update_edge(new_edge("connected_to", "living_room", "bedroom"),
                      false);

  // Object locations
  graph1->update_edge(new_edge("is_in", "table", "living_room"), true);
  graph1->update_edge(new_edge("is_in", "chair", "living_room"), true);
  graph1->update_edge(new_edge("is_in", "fridge", "kitchen"), true);

  // Robot location
  graph1->update_edge(new_edge("is_at", "turtlebot", "living_room"), true);

  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Allow sync

  // Verify structure
  EXPECT_EQ(graph1->get_num_nodes(), 7u);
  EXPECT_EQ(graph1->get_num_edges(), 6u);
  EXPECT_EQ(graph2->get_num_nodes(), 7u);
  EXPECT_EQ(graph2->get_num_edges(), 6u);

  // Check robot properties
  auto retrieved_robot = graph1->get_node("turtlebot");
  ASSERT_TRUE(retrieved_robot.has_value());

  auto battery = get_property<double>(retrieved_robot.value(), "battery");
  ASSERT_TRUE(battery.has_value());
  EXPECT_DOUBLE_EQ(battery.value(), 85.5);

  auto retrieved_robot2 = graph2->get_node("turtlebot");
  ASSERT_TRUE(retrieved_robot2.has_value());

  auto battery2 = get_property<double>(retrieved_robot2.value(), "battery");
  ASSERT_TRUE(battery2.has_value());
  EXPECT_DOUBLE_EQ(battery2.value(), 85.5);

  // Check connections from living room
  auto living_room_connections = graph1->get_out_edges("living_room");
  EXPECT_EQ(living_room_connections.size(), 2u);

  auto living_room_connections2 = graph2->get_out_edges("living_room");
  EXPECT_EQ(living_room_connections2.size(), 2u);

  // Check objects in living room
  auto objects_in_living = graph1->get_in_edges("living_room");
  EXPECT_GE(objects_in_living.size(), 2u); // table, chair, and robot

  auto objects_in_living2 = graph2->get_in_edges("living_room");
  EXPECT_GE(objects_in_living2.size(), 2u); // table, chair, and robot
}

} // namespace test
} // namespace knowledge_graph

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
