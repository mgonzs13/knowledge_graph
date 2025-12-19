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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the knowledge graph demo node
  auto graph = knowledge_graph::KnowledgeGraph::get_instance();

  // Create robot instance
  graph->create_node("leia", "robot");

  // Create room instances
  graph->create_node("entrance", "room");

  graph->create_node("kitchen", "room");

  graph->create_node("bedroom", "room");

  graph->create_node("dinning", "room");

  graph->create_node("bathroom", "room");

  graph->create_node("chargingroom", "room");

  // Connected predicates (bidirectional)
  graph->create_edge("connected", "entrance", "dinning");
  graph->create_edge("connected", "dinning", "entrance");

  graph->create_edge("connected", "dinning", "kitchen");
  graph->create_edge("connected", "kitchen", "dinning");

  graph->create_edge("connected", "dinning", "bedroom");
  graph->create_edge("connected", "bedroom", "dinning");

  graph->create_edge("connected", "bathroom", "bedroom");
  graph->create_edge("connected", "bedroom", "bathroom");

  graph->create_edge("connected", "chargingroom", "kitchen");
  graph->create_edge("connected", "kitchen", "chargingroom");

  // Other predicates
  graph->create_edge("charging_point_at", "chargingroom", "chargingroom");
  graph->create_edge("battery_low", "leia", "leia");

  graph->create_edge("robot_at", "leia", "entrance");

  // Goal predicate
  auto goal_edge = graph->create_edge("robot_at", "leia", "bathroom");
  goal_edge.set_property<bool>("is_goal", true);
  graph->update_edge(goal_edge);

  std::cout << "Knowledge Graph Demo" << std::endl;
  std::cout << "====================" << std::endl;

  auto nodes = graph->get_nodes();
  std::cout << "Nodes (" << nodes.size() << "):" << std::endl;
  for (const auto &node : nodes) {
    std::cout << "  " << node.to_string() << std::endl;
  }

  auto edges = graph->get_edges();
  std::cout << "Edges (" << edges.size() << "):" << std::endl;
  for (const auto &edge : edges) {
    std::cout << "  " << edge.to_string() << std::endl;
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}