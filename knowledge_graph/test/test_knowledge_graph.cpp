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
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/graph.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"

using namespace knowledge_graph;

class KnowledgeGraphTest : public ::testing::Test {
protected:
  std::shared_ptr<KnowledgeGraph> kg;

  void SetUp() override {
    kg = KnowledgeGraph::get_instance();
    // Clear the graph before each test
    for (const auto &edge : kg->get_edges()) {
      kg->remove_edge(edge);
    }
    for (const auto &node : kg->get_nodes()) {
      kg->remove_node(node);
    }
  }
};

// Singleton Tests
TEST_F(KnowledgeGraphTest, GetInstanceReturnsSameInstance) {
  auto kg1 = KnowledgeGraph::get_instance();
  auto kg2 = KnowledgeGraph::get_instance();
  EXPECT_EQ(kg1.get(), kg2.get());
}

// Node Creation Tests
TEST_F(KnowledgeGraphTest, CreateNode) {
  auto node = kg->create_node("robot", "robot_type");
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_EQ(node.get_type(), "robot_type");
  EXPECT_EQ(kg->get_num_nodes(), 1);
}

// Node Query Tests
TEST_F(KnowledgeGraphTest, HasNode) {
  EXPECT_FALSE(kg->has_node("robot"));
  kg->create_node("robot", "robot_type");
  EXPECT_TRUE(kg->has_node("robot"));
}

TEST_F(KnowledgeGraphTest, GetNode) {
  kg->create_node("robot", "robot_type");
  auto node = kg->get_node("robot");
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_EQ(node.get_type(), "robot_type");
}

TEST_F(KnowledgeGraphTest, GetNonexistentNodeThrows) {
  EXPECT_THROW(kg->get_node("nonexistent"), std::runtime_error);
}

TEST_F(KnowledgeGraphTest, GetNodes) {
  kg->create_node("robot1", "type1");
  kg->create_node("robot2", "type2");
  auto nodes = kg->get_nodes();
  EXPECT_EQ(nodes.size(), 2u);

  std::set<std::string> names;
  for (const auto &n : nodes) {
    names.insert(n.get_name());
  }
  EXPECT_EQ(names.count("robot1"), 1u);
  EXPECT_EQ(names.count("robot2"), 1u);
}

TEST_F(KnowledgeGraphTest, GetNumNodes) {
  EXPECT_EQ(kg->get_num_nodes(), 0);
  kg->create_node("robot1", "type1");
  EXPECT_EQ(kg->get_num_nodes(), 1);
  kg->create_node("robot2", "type2");
  EXPECT_EQ(kg->get_num_nodes(), 2);
}

// Node Update Tests
TEST_F(KnowledgeGraphTest, UpdateNodeExisting) {
  kg->create_node("robot", "robot_type");

  graph::Node updated_node("robot", "new_type");
  updated_node.set_property<double>("speed", 10.0);
  kg->update_node(updated_node);

  auto node = kg->get_node("robot");
  EXPECT_EQ(node.get_type(), "new_type");
  EXPECT_NEAR(node.get_property<double>("speed"), 10.0, 0.001);
}

TEST_F(KnowledgeGraphTest, UpdateNodeNew) {
  graph::Node node("robot", "robot_type");
  kg->update_node(node);
  EXPECT_TRUE(kg->has_node("robot"));
}

TEST_F(KnowledgeGraphTest, UpdateNodes) {
  std::vector<graph::Node> nodes;
  nodes.push_back(graph::Node("robot1", "type1"));
  nodes.push_back(graph::Node("robot2", "type2"));
  kg->update_nodes(nodes);
  EXPECT_EQ(kg->get_num_nodes(), 2);
}

// Node Removal Tests
TEST_F(KnowledgeGraphTest, RemoveNode) {
  auto node = kg->create_node("robot", "robot_type");
  EXPECT_TRUE(kg->remove_node(node));
  EXPECT_FALSE(kg->has_node("robot"));
}

TEST_F(KnowledgeGraphTest, RemoveNonexistentNode) {
  graph::Node node("nonexistent", "type");
  EXPECT_FALSE(kg->remove_node(node));
}

TEST_F(KnowledgeGraphTest, RemoveNodes) {
  auto node1 = kg->create_node("robot1", "type1");
  auto node2 = kg->create_node("robot2", "type2");
  kg->create_node("robot3", "type3");

  std::vector<graph::Node> to_remove = {node1, node2};
  kg->remove_nodes(to_remove);
  EXPECT_EQ(kg->get_num_nodes(), 1);
  EXPECT_TRUE(kg->has_node("robot3"));
}

// Edge Creation Tests
TEST_F(KnowledgeGraphTest, CreateEdge) {
  kg->create_node("node_a", "type");
  kg->create_node("node_b", "type");
  auto edge = kg->create_edge("connects", "node_a", "node_b");
  EXPECT_EQ(edge.get_type(), "connects");
  EXPECT_EQ(edge.get_source_node(), "node_a");
  EXPECT_EQ(edge.get_target_node(), "node_b");
  EXPECT_EQ(kg->get_num_edges(), 1);
}

// Edge Query Tests
TEST_F(KnowledgeGraphTest, HasEdge) {
  kg->create_node("node_a", "type");
  kg->create_node("node_b", "type");
  EXPECT_FALSE(kg->has_edge("connects", "node_a", "node_b"));
  kg->create_edge("connects", "node_a", "node_b");
  EXPECT_TRUE(kg->has_edge("connects", "node_a", "node_b"));
}

TEST_F(KnowledgeGraphTest, GetNumEdges) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  EXPECT_EQ(kg->get_num_edges(), 0);
  kg->create_edge("connects", "a", "b");
  EXPECT_EQ(kg->get_num_edges(), 1);
}

TEST_F(KnowledgeGraphTest, GetEdges) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_node("c", "type");
  kg->create_edge("connects", "a", "b");
  kg->create_edge("connects", "b", "c");
  auto edges = kg->get_edges();
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdgesFromNode) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_node("c", "type");
  kg->create_edge("connects", "a", "b");
  kg->create_edge("connects", "a", "c");
  kg->create_edge("connects", "b", "c");
  auto edges = kg->get_edges_from_node("a");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdgesToNode) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_node("c", "type");
  kg->create_edge("connects", "a", "c");
  kg->create_edge("connects", "b", "c");
  auto edges = kg->get_edges_to_node("c");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdgesBetweenNodes) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_edge("connects1", "a", "b");
  kg->create_edge("connects2", "a", "b");
  auto edges = kg->get_edges_between_nodes("a", "b");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdgesByType) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_edge("type1", "a", "b");
  kg->create_edge("type2", "a", "b");
  auto edges = kg->get_edges_by_type("type1");
  EXPECT_EQ(edges.size(), 1u);
  EXPECT_EQ(edges[0].get_type(), "type1");
}

TEST_F(KnowledgeGraphTest, GetEdgesFromNodeByType) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_node("c", "type");
  kg->create_edge("type1", "a", "b");
  kg->create_edge("type1", "a", "c");
  kg->create_edge("type2", "a", "b");
  auto edges = kg->get_edges_from_node_by_type("type1", "a");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdgesToNodeByType) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_node("c", "type");
  kg->create_edge("type1", "a", "c");
  kg->create_edge("type1", "b", "c");
  kg->create_edge("type2", "a", "c");
  auto edges = kg->get_edges_to_node_by_type("type1", "c");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(KnowledgeGraphTest, GetEdge) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_edge("connects", "a", "b");
  auto edge = kg->get_edge("connects", "a", "b");
  EXPECT_EQ(edge.get_type(), "connects");
  EXPECT_EQ(edge.get_source_node(), "a");
  EXPECT_EQ(edge.get_target_node(), "b");
}

TEST_F(KnowledgeGraphTest, GetNonexistentEdgeThrows) {
  EXPECT_THROW(kg->get_edge("connects", "a", "b"), std::runtime_error);
}

// Edge Update Tests
TEST_F(KnowledgeGraphTest, UpdateEdge) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_edge("connects", "a", "b");

  graph::Edge updated_edge("connects", "a", "b");
  updated_edge.set_property<double>("weight", 5.0);
  kg->update_edge(updated_edge);

  auto edge = kg->get_edge("connects", "a", "b");
  EXPECT_NEAR(edge.get_property<double>("weight"), 5.0, 0.001);
}

TEST_F(KnowledgeGraphTest, UpdateEdges) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  std::vector<graph::Edge> edges;
  edges.push_back(graph::Edge("type1", "a", "b"));
  edges.push_back(graph::Edge("type2", "a", "b"));
  kg->update_edges(edges);
  EXPECT_EQ(kg->get_num_edges(), 2);
}

// Edge Removal Tests
TEST_F(KnowledgeGraphTest, RemoveEdge) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  auto edge = kg->create_edge("connects", "a", "b");
  EXPECT_TRUE(kg->remove_edge(edge));
  EXPECT_FALSE(kg->has_edge("connects", "a", "b"));
}

TEST_F(KnowledgeGraphTest, RemoveNonexistentEdge) {
  graph::Edge edge("connects", "a", "b");
  EXPECT_FALSE(kg->remove_edge(edge));
}

TEST_F(KnowledgeGraphTest, RemoveEdges) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  auto edge1 = kg->create_edge("type1", "a", "b");
  auto edge2 = kg->create_edge("type2", "a", "b");
  kg->create_edge("type3", "a", "b");

  std::vector<graph::Edge> to_remove = {edge1, edge2};
  kg->remove_edges(to_remove);
  EXPECT_EQ(kg->get_num_edges(), 1);
  EXPECT_TRUE(kg->has_edge("type3", "a", "b"));
}

// Graph Operations Tests
TEST_F(KnowledgeGraphTest, UpdateGraph) {
  graph::Graph other_graph;
  other_graph.create_node("external_node1", "type1");
  other_graph.create_node("external_node2", "type2");
  other_graph.create_edge("connects", "external_node1", "external_node2");

  kg->update_graph(other_graph);

  EXPECT_TRUE(kg->has_node("external_node1"));
  EXPECT_TRUE(kg->has_node("external_node2"));
  EXPECT_TRUE(kg->has_edge("connects", "external_node1", "external_node2"));
}

TEST_F(KnowledgeGraphTest, ToMsg) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  kg->create_edge("connects", "a", "b");

  auto msg = kg->to_msg();

  EXPECT_EQ(msg.nodes.size(), 2u);
  EXPECT_EQ(msg.edges.size(), 1u);
}

// Node Properties Tests
TEST_F(KnowledgeGraphTest, NodeWithProperties) {
  auto node = kg->create_node("robot", "robot_type");
  node.set_property<double>("battery", 100.0);
  node.set_property<bool>("active", true);
  node.set_property<std::string>("name", "RobotOne");
  kg->update_node(node);

  auto retrieved = kg->get_node("robot");
  EXPECT_NEAR(retrieved.get_property<double>("battery"), 100.0, 0.001);
  EXPECT_EQ(retrieved.get_property<bool>("active"), true);
  EXPECT_EQ(retrieved.get_property<std::string>("name"), "RobotOne");
}

// Edge Properties Tests
TEST_F(KnowledgeGraphTest, EdgeWithProperties) {
  kg->create_node("a", "type");
  kg->create_node("b", "type");
  auto edge = kg->create_edge("path", "a", "b");
  edge.set_property<double>("distance", 10.5);
  edge.set_property<bool>("traversable", true);
  kg->update_edge(edge);

  auto retrieved = kg->get_edge("path", "a", "b");
  EXPECT_NEAR(retrieved.get_property<double>("distance"), 10.5, 0.001);
  EXPECT_EQ(retrieved.get_property<bool>("traversable"), true);
}

// Complex Scenarios Tests
TEST_F(KnowledgeGraphTest, GraphWithMultipleEdgeTypes) {
  kg->create_node("robot", "robot_type");
  kg->create_node("location", "location_type");

  kg->create_edge("at", "robot", "location");
  kg->create_edge("visited", "robot", "location");
  kg->create_edge("can_reach", "robot", "location");

  auto edges = kg->get_edges_between_nodes("robot", "location");
  EXPECT_EQ(edges.size(), 3u);

  std::set<std::string> types;
  for (const auto &e : edges) {
    types.insert(e.get_type());
  }
  EXPECT_EQ(types.count("at"), 1u);
  EXPECT_EQ(types.count("visited"), 1u);
  EXPECT_EQ(types.count("can_reach"), 1u);
}

TEST_F(KnowledgeGraphTest, SelfLoopEdge) {
  kg->create_node("robot", "robot_type");
  auto edge = kg->create_edge("monitors", "robot", "robot");

  EXPECT_EQ(edge.get_source_node(), "robot");
  EXPECT_EQ(edge.get_target_node(), "robot");
  EXPECT_TRUE(kg->has_edge("monitors", "robot", "robot"));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  // Shutdown ROS 2 to clean up resources properly
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
