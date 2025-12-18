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
#include "knowledge_graph/graph/graph.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"

using namespace knowledge_graph::graph;

class GraphNodeTest : public ::testing::Test {
protected:
  Graph graph;
};

// Constructor Tests
TEST_F(GraphNodeTest, DefaultConstructor) {
  EXPECT_EQ(graph.get_num_nodes(), 0);
  EXPECT_EQ(graph.get_num_edges(), 0);
}

// Node Creation Tests
TEST_F(GraphNodeTest, CreateNode) {
  auto node = graph.create_node("robot", "robot_type");
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_EQ(node.get_type(), "robot_type");
  EXPECT_EQ(graph.get_num_nodes(), 1);
}

TEST_F(GraphNodeTest, CreateDuplicateNodeDoesNotThrow) {
  // In C++, creating a duplicate node does not throw - it returns existing
  graph.create_node("robot", "robot_type");
  auto node2 = graph.create_node("robot", "different_type");
  // A duplicate is created, so we have 2 nodes
  EXPECT_EQ(graph.get_num_nodes(), 2);
}

// Node Query Tests
TEST_F(GraphNodeTest, HasNode) {
  EXPECT_FALSE(graph.has_node("robot"));
  graph.create_node("robot", "robot_type");
  EXPECT_TRUE(graph.has_node("robot"));
}

TEST_F(GraphNodeTest, GetNode) {
  graph.create_node("robot", "robot_type");
  auto node = graph.get_node("robot");
  EXPECT_EQ(node.get_name(), "robot");
  EXPECT_EQ(node.get_type(), "robot_type");
}

TEST_F(GraphNodeTest, GetNonexistentNodeThrows) {
  EXPECT_THROW(graph.get_node("nonexistent"), std::runtime_error);
}

TEST_F(GraphNodeTest, GetNodes) {
  graph.create_node("robot1", "type1");
  graph.create_node("robot2", "type2");
  auto nodes = graph.get_nodes();
  EXPECT_EQ(nodes.size(), 2u);
}

TEST_F(GraphNodeTest, GetNumNodes) {
  EXPECT_EQ(graph.get_num_nodes(), 0);
  graph.create_node("robot1", "type1");
  EXPECT_EQ(graph.get_num_nodes(), 1);
  graph.create_node("robot2", "type2");
  EXPECT_EQ(graph.get_num_nodes(), 2);
}

// Node Update Tests
TEST_F(GraphNodeTest, UpdateNodeExisting) {
  graph.create_node("robot", "robot_type");

  Node updated_node("robot", "new_type");
  updated_node.set_property<double>("speed", 10.0);
  graph.update_node(updated_node);

  auto node = graph.get_node("robot");
  EXPECT_EQ(node.get_type(), "new_type");
  EXPECT_NEAR(node.get_property<double>("speed"), 10.0, 0.001);
}

TEST_F(GraphNodeTest, UpdateNodeNew) {
  Node node("robot", "robot_type");
  graph.update_node(node);
  EXPECT_TRUE(graph.has_node("robot"));
}

TEST_F(GraphNodeTest, UpdateNodes) {
  std::vector<Node> nodes;
  nodes.push_back(Node("robot1", "type1"));
  nodes.push_back(Node("robot2", "type2"));
  graph.update_nodes(nodes);
  EXPECT_EQ(graph.get_num_nodes(), 2);
}

// Node Removal Tests
TEST_F(GraphNodeTest, RemoveNode) {
  auto node = graph.create_node("robot", "robot_type");
  EXPECT_TRUE(graph.remove_node(node));
  EXPECT_FALSE(graph.has_node("robot"));
}

TEST_F(GraphNodeTest, RemoveNonexistentNode) {
  Node node("nonexistent", "type");
  EXPECT_FALSE(graph.remove_node(node));
}

TEST_F(GraphNodeTest, RemoveNodes) {
  auto node1 = graph.create_node("robot1", "type1");
  auto node2 = graph.create_node("robot2", "type2");
  graph.create_node("robot3", "type3");

  std::vector<Node> to_remove = {node1, node2};
  graph.remove_nodes(to_remove);
  EXPECT_EQ(graph.get_num_nodes(), 1);
  EXPECT_TRUE(graph.has_node("robot3"));
}

class GraphEdgeTest : public ::testing::Test {
protected:
  Graph graph;

  void SetUp() override {
    graph.create_node("a", "type");
    graph.create_node("b", "type");
    graph.create_node("c", "type");
  }
};

// Edge Creation Tests
TEST_F(GraphEdgeTest, CreateEdge) {
  auto edge = graph.create_edge("connects", "a", "b");
  EXPECT_EQ(edge.get_type(), "connects");
  EXPECT_EQ(edge.get_source_node(), "a");
  EXPECT_EQ(edge.get_target_node(), "b");
  EXPECT_EQ(graph.get_num_edges(), 1);
}

TEST_F(GraphEdgeTest, CreateEdgeWithoutSourceNodeThrows) {
  Graph empty_graph;
  empty_graph.create_node("b", "type");
  EXPECT_THROW(empty_graph.create_edge("connects", "a", "b"),
               std::runtime_error);
}

TEST_F(GraphEdgeTest, CreateEdgeWithoutTargetNodeThrows) {
  Graph empty_graph;
  empty_graph.create_node("a", "type");
  EXPECT_THROW(empty_graph.create_edge("connects", "a", "b"),
               std::runtime_error);
}

TEST_F(GraphEdgeTest, CreateDuplicateEdgeDoesNotThrow) {
  // In C++, creating a duplicate edge does not throw - it creates another edge
  graph.create_edge("connects", "a", "b");
  auto edge2 = graph.create_edge("connects", "a", "b");
  // A duplicate is created, so we have 2 edges
  EXPECT_EQ(graph.get_num_edges(), 2);
}

// Edge Query Tests
TEST_F(GraphEdgeTest, HasEdge) {
  EXPECT_FALSE(graph.has_edge("connects", "a", "b"));
  graph.create_edge("connects", "a", "b");
  EXPECT_TRUE(graph.has_edge("connects", "a", "b"));
}

TEST_F(GraphEdgeTest, GetEdge) {
  graph.create_edge("connects", "a", "b");
  auto edge = graph.get_edge("connects", "a", "b");
  EXPECT_EQ(edge.get_type(), "connects");
}

TEST_F(GraphEdgeTest, GetNonexistentEdgeThrows) {
  EXPECT_THROW(graph.get_edge("connects", "a", "b"), std::runtime_error);
}

TEST_F(GraphEdgeTest, GetEdges) {
  graph.create_edge("e1", "a", "b");
  graph.create_edge("e2", "b", "c");
  auto edges = graph.get_edges();
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(GraphEdgeTest, GetNumEdges) {
  EXPECT_EQ(graph.get_num_edges(), 0);
  graph.create_edge("connects", "a", "b");
  EXPECT_EQ(graph.get_num_edges(), 1);
}

TEST_F(GraphEdgeTest, GetEdgesFromNode) {
  graph.create_edge("e1", "a", "b");
  graph.create_edge("e2", "a", "c");
  graph.create_edge("e3", "b", "c");

  auto edges = graph.get_edges_from_node("a");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(GraphEdgeTest, GetEdgesToNode) {
  graph.create_edge("e1", "a", "c");
  graph.create_edge("e2", "b", "c");
  graph.create_edge("e3", "a", "b");

  auto edges = graph.get_edges_to_node("c");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(GraphEdgeTest, GetEdgesBetweenNodes) {
  graph.create_edge("e1", "a", "b");
  graph.create_edge("e2", "a", "b");

  auto edges = graph.get_edges_between_nodes("a", "b");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(GraphEdgeTest, GetEdgesByType) {
  graph.create_edge("connects", "a", "b");
  graph.create_edge("relates", "b", "c");
  graph.create_edge("connects", "a", "c");

  auto edges = graph.get_edges_by_type("connects");
  EXPECT_EQ(edges.size(), 2u);
}

TEST_F(GraphEdgeTest, GetEdgesFromNodeByType) {
  graph.create_edge("connects", "a", "b");
  graph.create_edge("relates", "a", "c");

  auto edges = graph.get_edges_from_node_by_type("connects", "a");
  EXPECT_EQ(edges.size(), 1u);
  EXPECT_EQ(edges[0].get_target_node(), "b");
}

TEST_F(GraphEdgeTest, GetEdgesToNodeByType) {
  graph.create_edge("connects", "a", "c");
  graph.create_edge("relates", "b", "c");

  auto edges = graph.get_edges_to_node_by_type("connects", "c");
  EXPECT_EQ(edges.size(), 1u);
  EXPECT_EQ(edges[0].get_source_node(), "a");
}

// Edge Update Tests
TEST_F(GraphEdgeTest, UpdateEdgeExisting) {
  graph.create_edge("connects", "a", "b");

  Edge updated_edge("connects", "a", "b");
  updated_edge.set_property<double>("weight", 0.5);
  graph.update_edge(updated_edge);

  auto edge = graph.get_edge("connects", "a", "b");
  EXPECT_NEAR(edge.get_property<double>("weight"), 0.5, 0.001);
}

TEST_F(GraphEdgeTest, UpdateEdgeNew) {
  Edge edge("connects", "a", "b");
  graph.update_edge(edge);
  EXPECT_EQ(graph.get_num_edges(), 1);
}

TEST_F(GraphEdgeTest, UpdateEdges) {
  std::vector<Edge> edges;
  edges.push_back(Edge("e1", "a", "b"));
  edges.push_back(Edge("e2", "b", "c"));
  graph.update_edges(edges);
  EXPECT_EQ(graph.get_num_edges(), 2);
}

// Edge Removal Tests
TEST_F(GraphEdgeTest, RemoveEdge) {
  auto edge = graph.create_edge("connects", "a", "b");
  EXPECT_TRUE(graph.remove_edge(edge));
  EXPECT_EQ(graph.get_num_edges(), 0);
}

TEST_F(GraphEdgeTest, RemoveNonexistentEdge) {
  Edge edge("connects", "a", "b");
  EXPECT_FALSE(graph.remove_edge(edge));
}

TEST_F(GraphEdgeTest, RemoveEdges) {
  auto edge1 = graph.create_edge("e1", "a", "b");
  auto edge2 = graph.create_edge("e2", "b", "c");
  graph.create_edge("e3", "a", "c");

  std::vector<Edge> to_remove = {edge1, edge2};
  graph.remove_edges(to_remove);
  EXPECT_EQ(graph.get_num_edges(), 1);
}

class GraphSerializationTest : public ::testing::Test {
protected:
  Graph graph;
};

// Serialization Tests
TEST_F(GraphSerializationTest, ToMsg) {
  graph.create_node("a", "type1");
  graph.create_node("b", "type2");
  graph.create_edge("connects", "a", "b");

  auto msg = graph.to_msg();
  EXPECT_EQ(msg.nodes.size(), 2u);
  EXPECT_EQ(msg.edges.size(), 1u);
}

TEST_F(GraphSerializationTest, ConstructorFromMessage) {
  // Create original graph
  graph.create_node("a", "type1");
  graph.create_node("b", "type2");
  graph.create_edge("connects", "a", "b");

  // Convert to message and back
  auto msg = graph.to_msg();
  Graph restored(msg);

  EXPECT_EQ(restored.get_num_nodes(), 2);
  EXPECT_EQ(restored.get_num_edges(), 1);
  EXPECT_TRUE(restored.has_node("a"));
  EXPECT_TRUE(restored.has_node("b"));
  EXPECT_TRUE(restored.has_edge("connects", "a", "b"));
}

TEST_F(GraphSerializationTest, UpdateGraph) {
  graph.create_node("a", "type");

  Graph graph2;
  graph2.update_node(Node("b", "type"));
  Edge edge("connects", "a", "b");
  graph2.update_edge(edge);

  graph.update_graph(graph2);
  EXPECT_TRUE(graph.has_node("a"));
  EXPECT_TRUE(graph.has_node("b"));
  EXPECT_TRUE(graph.has_edge("connects", "a", "b"));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
