// Copyright 2023 Miguel Ángel González Santamarta
// Copyright 2021 Intelligent Robotics Lab
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

#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace knowledge_graph {

class KnowledgeGraph {
public:
  explicit KnowledgeGraph(rclcpp::Node *provided_node);
  explicit KnowledgeGraph(rclcpp::Node::SharedPtr provided_node)
      : KnowledgeGraph(provided_node.get()) {}
  explicit KnowledgeGraph(
      rclcpp_lifecycle::LifecycleNode::SharedPtr provided_node)
      : KnowledgeGraph((rclcpp::Node *)provided_node.get()) {}
  ~KnowledgeGraph() {
    update_sub.reset();
    update_pub.reset();
  }

  KnowledgeGraph(KnowledgeGraph &other) = delete;
  void operator=(const KnowledgeGraph &) = delete;

  bool remove_node(const std::string node, bool sync = true);
  bool exist_node(const std::string node);
  std::optional<knowledge_graph_msgs::msg::Node>
  get_node(const std::string node);

  bool remove_edge(const knowledge_graph_msgs::msg::Edge &edge,
                   bool sync = true);
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_edges(const std::string &source, const std::string &target);
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_edges(const std::string &edge_class);
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_out_edges(const std::string &source);
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_in_edges(const std::string &target);

  const std::vector<knowledge_graph_msgs::msg::Node> &get_nodes() {
    return this->graph->nodes;
  }
  const std::vector<knowledge_graph_msgs::msg::Edge> &get_edges() {
    return this->graph->edges;
  }
  const std::vector<std::string> get_node_names();

  size_t get_num_edges() const;
  size_t get_num_nodes() const;

  bool update_node(const knowledge_graph_msgs::msg::Node &node,
                   bool sync = true);
  bool update_edge(const knowledge_graph_msgs::msg::Edge &edge,
                   bool sync = true);

protected:
  rclcpp::Node *provided_node;

  knowledge_graph_msgs::msg::Graph::UniquePtr graph;
  std::string graph_id;
  rclcpp::Time last_ts;

  void update_graph(knowledge_graph_msgs::msg::Graph msg);
  void update_callback(knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg);
  void reqsync_timer_callback();

private:
  rclcpp::Publisher<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_pub;
  rclcpp::Subscription<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_sub;

  rclcpp::TimerBase::SharedPtr reqsync_timer;
  rclcpp::Time start_time;
};

// singleton

} // namespace knowledge_graph

#endif
