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
#include <mutex>
#include <optional>
#include <string>

#include "knowledge_graph/graph/graph.hpp"

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace knowledge_graph {

/**
 * @brief A distributed knowledge graph implementation for ROS 2.
 *
 * This class provides a synchronized knowledge graph that can be shared
 * across multiple ROS 2 nodes. It supports adding, updating, and removing
 * nodes and edges, with automatic synchronization between instances.
 *
 */
class KnowledgeGraph : public graph::Graph {
public:
  /**
   * @brief Constructs a KnowledgeGraph with a raw pointer to a ROS 2 node.
   */
  explicit KnowledgeGraph(rclcpp::Node *node_ptr);

  /**
   * @brief Constructs a KnowledgeGraph with a shared pointer to a ROS 2 node.
   * @param provided_node Shared pointer to the ROS 2 node that owns this graph.
   */
  explicit KnowledgeGraph(rclcpp::Node::SharedPtr provided_node)
      : KnowledgeGraph(provided_node.get()) {}

  /**
   * @brief Constructs a KnowledgeGraph with a lifecycle node.
   * @param provided_node Shared pointer to the lifecycle node that owns this
   * graph.
   */
  explicit KnowledgeGraph(
      rclcpp_lifecycle::LifecycleNode::SharedPtr provided_node)
      : KnowledgeGraph((rclcpp::Node *)provided_node.get()) {}

  /**
   * @brief Default destructor.
   */
  ~KnowledgeGraph() = default;

  void update_graph(const graph::Graph &graph) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    for (const auto &node : graph.get_nodes()) {
      graph::Graph::update_node(node);
    }

    for (const auto &edge : graph.get_edges()) {
      graph::Graph::update_edge(edge);
    }
  }

  graph::Node create_node(const std::string &name,
                          const std::string &type) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    graph::Node node = graph::Graph::create_node(name, type);
    this->publish_update(
        knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
        knowledge_graph_msgs::msg::GraphUpdate::NODE,
        std::vector<knowledge_graph_msgs::msg::Node>{node.to_msg()}, {});
    return node;
  }

  void update_node(const graph::Node &node) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    graph::Graph::update_node(node);
    this->publish_update(
        knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
        knowledge_graph_msgs::msg::GraphUpdate::NODE,
        std::vector<knowledge_graph_msgs::msg::Node>{node.to_msg()}, {});
  }

  void update_nodes(const std::vector<graph::Node> &nodes) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    std::vector<knowledge_graph_msgs::msg::Node> node_msgs;
    for (const auto &node : nodes) {
      graph::Graph::update_node(node);
      node_msgs.push_back(node.to_msg());
    }
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                         knowledge_graph_msgs::msg::GraphUpdate::NODE,
                         node_msgs, {});
  }

  bool remove_node(const graph::Node &node) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    bool removed = graph::Graph::remove_node(node);
    if (removed) {
      this->publish_update(
          knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
          knowledge_graph_msgs::msg::GraphUpdate::NODE,
          std::vector<knowledge_graph_msgs::msg::Node>{node.to_msg()}, {});
    }
    return removed;
  }

  void remove_nodes(const std::vector<graph::Node> &nodes) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    std::vector<knowledge_graph_msgs::msg::Node> removed_nodes;
    for (const auto &node : nodes) {
      if (graph::Graph::remove_node(node)) {
        removed_nodes.push_back(node.to_msg());
      }
    }
    if (!removed_nodes.empty()) {
      this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                           knowledge_graph_msgs::msg::GraphUpdate::NODE,
                           removed_nodes, {});
    }
  }

  graph::Edge create_edge(const std::string &type,
                          const std::string &source_node,
                          const std::string &target_node) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    graph::Edge edge =
        graph::Graph::create_edge(type, source_node, target_node);
    this->publish_update(
        knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
        knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
        std::vector<knowledge_graph_msgs::msg::Edge>{edge.to_msg()});
    return edge;
  }

  void update_edge(const graph::Edge &edge) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    graph::Graph::update_edge(edge);
    this->publish_update(
        knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
        knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
        std::vector<knowledge_graph_msgs::msg::Edge>{edge.to_msg()});
  }

  void update_edges(const std::vector<graph::Edge> &edges) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    std::vector<knowledge_graph_msgs::msg::Edge> edge_msgs;
    for (const auto &edge : edges) {
      graph::Graph::update_edge(edge);
      edge_msgs.push_back(edge.to_msg());
    }
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                         knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
                         edge_msgs);
  }

  bool remove_edge(const graph::Edge &edge) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    bool removed = graph::Graph::remove_edge(edge);
    if (removed) {
      this->publish_update(
          knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
          knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
          std::vector<knowledge_graph_msgs::msg::Edge>{edge.to_msg()});
    }
    return removed;
  }

  void remove_edges(const std::vector<graph::Edge> &edges) override {
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
    std::vector<knowledge_graph_msgs::msg::Edge> removed_edges;
    for (auto &edge : edges) {
      if (graph::Graph::remove_edge(edge)) {
        removed_edges.push_back(edge.to_msg());
      }
    }
    if (!removed_edges.empty()) {
      this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                           knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
                           removed_edges);
    }
  }

protected:
  /// @brief Pointer to the ROS 2 node that owns this graph.
  rclcpp::Node *node;

  /// @brief Unique identifier for this graph instance.
  std::string graph_id;

  /// @brief Timestamp of the last graph modification.
  rclcpp::Time last_ts;

  /**
   * @brief Callback for handling incoming graph update messages.
   * @param msg The graph update message.
   */
  void update_callback(knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg);

  /**
   * @brief Timer callback for requesting graph synchronization.
   */
  void reqsync_timer_callback();

  /**
   * @brief Publishes a graph update message.
   * @param operation The operation type (UPDATE, REMOVE).
   * @param element The element type (NODE, EDGE).
   * @param node Optional node data for node operations.
   * @param edge Optional edge data for edge operations.
   */
  void
  publish_update(uint8_t operation, uint8_t element,
                 const std::vector<knowledge_graph_msgs::msg::Node> &nodes,
                 const std::vector<knowledge_graph_msgs::msg::Edge> &edges);

private:
  /// @brief Publisher for graph update messages.
  rclcpp::Publisher<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_pub;

  /// @brief Subscriber for graph update messages.
  rclcpp::Subscription<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_sub;

  /// @brief Timer for periodic synchronization requests.
  rclcpp::TimerBase::SharedPtr reqsync_timer;

  /// @brief Start time for synchronization timeout.
  rclcpp::Time start_time;

  /// @brief Mutex for thread-safe graph operations.
  mutable std::recursive_mutex graph_mutex_;
};

} // namespace knowledge_graph

#endif
