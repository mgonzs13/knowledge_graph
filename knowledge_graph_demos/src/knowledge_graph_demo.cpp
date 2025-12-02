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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class KnowledgeGraphDemo : public rclcpp::Node {
public:
  KnowledgeGraphDemo() : Node("knowledge_graph_demo") {
    // Initialize the knowledge graph
    this->graph_ = std::make_shared<knowledge_graph::KnowledgeGraph>(this);

    // Create instances (nodes)
    this->create_instances();

    // Create predicates (edges)
    this->create_predicates();

    // Print the graph
    this->print_graph();
  }

private:
  void create_instances() {
    // Create robot instance
    auto leia = knowledge_graph::new_node("leia", "robot");
    this->graph_->update_node(leia);

    // Create room instances
    auto entrance = knowledge_graph::new_node("entrance", "room");
    this->graph_->update_node(entrance);

    auto kitchen = knowledge_graph::new_node("kitchen", "room");
    this->graph_->update_node(kitchen);

    auto bedroom = knowledge_graph::new_node("bedroom", "room");
    this->graph_->update_node(bedroom);

    auto dinning = knowledge_graph::new_node("dinning", "room");
    this->graph_->update_node(dinning);

    auto bathroom = knowledge_graph::new_node("bathroom", "room");
    this->graph_->update_node(bathroom);

    auto chargingroom = knowledge_graph::new_node("chargingroom", "room");
    this->graph_->update_node(chargingroom);
  }

  void create_predicates() {
    // Connected predicates (bidirectional)
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "entrance", "dinning"));
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "dinning", "entrance"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "dinning", "kitchen"));
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "kitchen", "dinning"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "dinning", "bedroom"));
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "bedroom", "dinning"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "bathroom", "bedroom"));
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "bedroom", "bathroom"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "chargingroom", "kitchen"));
    this->graph_->update_edge(
        knowledge_graph::new_edge("connected", "kitchen", "chargingroom"));

    // Other predicates
    this->graph_->update_edge(knowledge_graph::new_edge(
        "charging_point_at", "chargingroom", "chargingroom"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("battery_low", "leia", "leia"));

    this->graph_->update_edge(
        knowledge_graph::new_edge("robot_at", "leia", "entrance"));

    // Goal predicate
    this->graph_->update_edge(
        knowledge_graph::new_edge("goal", "leia", "bathroom"));
  }

  void print_graph() {
    RCLCPP_INFO(this->get_logger(), "Knowledge Graph Demo");
    RCLCPP_INFO(this->get_logger(), "====================");

    auto nodes = this->graph_->get_nodes();
    RCLCPP_INFO(this->get_logger(), "Nodes (%zu):", nodes.size());
    for (const auto &node : nodes) {
      RCLCPP_INFO(this->get_logger(), "  %s",
                  knowledge_graph::to_string(node).c_str());
    }

    auto edges = this->graph_->get_edges();
    RCLCPP_INFO(this->get_logger(), "Edges (%zu):", edges.size());
    for (const auto &edge : edges) {
      RCLCPP_INFO(this->get_logger(), "  %s",
                  knowledge_graph::to_string(edge).c_str());
    }
  }

  std::shared_ptr<knowledge_graph::KnowledgeGraph> graph_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KnowledgeGraphDemo>());
  rclcpp::shutdown();
  return 0;
}