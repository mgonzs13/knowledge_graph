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

#include <algorithm>
#include <memory>
#include <random>
#include <shared_mutex>
#include <string>
#include <vector>

#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace knowledge_graph;

using std::placeholders::_1;
using namespace std::chrono_literals;

/// @brief Synchronization request timeout in seconds.
static constexpr double SYNC_TIMEOUT_SEC = 1.0;

/// @brief Timer interval for synchronization requests in milliseconds.
static constexpr std::chrono::milliseconds SYNC_TIMER_INTERVAL{100};

/**
 * @brief Generates a unique UUID as a string.
 *
 * This function uses random numbers to generate a 16-character hexadecimal
 * UUID.
 *
 * @return A string containing a 16-character hexadecimal UUID.
 */
inline std::string generateUUID() {
  static constexpr char hex_digits[] = "0123456789abcdef";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);

  std::string result;
  result.reserve(16);
  for (int i = 0; i < 16; ++i) {
    result += hex_digits[dis(gen)];
  }
  return result;
}

KnowledgeGraph::KnowledgeGraph() {

  this->node = std::make_shared<rclcpp::Node>("knowledge_graph_" +
                                              generateUUID() + "_node");
  // Add this node's base interface to the executor for multi-threaded
  // execution.
  this->executor.add_node(this->node->get_node_base_interface());

  // Initialize and detach the spin thread to run the executor asynchronously.
  this->spin_thread =
      std::make_unique<std::thread>(&rclcpp::Executor::spin, &this->executor);

  // Detach the spin thread to allow background execution.
  this->spin_thread->detach();

  this->graph_id = this->node->get_name();

  this->update_pub =
      this->node->create_publisher<knowledge_graph_msgs::msg::GraphUpdate>(
          "graph_update", rclcpp::QoS(100).reliable());
  this->update_sub =
      this->node->create_subscription<knowledge_graph_msgs::msg::GraphUpdate>(
          "graph_update", rclcpp::QoS(100).reliable(),
          std::bind(&KnowledgeGraph::update_callback, this, _1));

  this->last_ts = this->node->now();
  this->start_time = this->node->now();

  this->reqsync_timer = this->node->create_wall_timer(
      SYNC_TIMER_INTERVAL,
      std::bind(&KnowledgeGraph::reqsync_timer_callback, this));
  this->reqsync_timer_callback();
}

void KnowledgeGraph::publish_update(
    uint8_t operation, uint8_t element,
    const std::vector<knowledge_graph_msgs::msg::Node> &nodes,
    const std::vector<knowledge_graph_msgs::msg::Edge> &edges) {

  knowledge_graph_msgs::msg::GraphUpdate update_msg;
  update_msg.stamp = this->node->get_clock()->now();
  update_msg.node_id = this->graph_id;
  update_msg.operation_type = operation;
  update_msg.element_type = element;

  update_msg.nodes = nodes;
  update_msg.edges = edges;

  this->update_pub->publish(update_msg);
}

void KnowledgeGraph::reqsync_timer_callback() {
  if ((this->node->get_clock()->now() - this->start_time).seconds() >
      SYNC_TIMEOUT_SEC) {
    this->reqsync_timer->cancel();
  }

  knowledge_graph_msgs::msg::GraphUpdate hello_msg;
  hello_msg.stamp = this->node->get_clock()->now();
  hello_msg.node_id = this->graph_id;
  hello_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  hello_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  hello_msg.graph = this->to_msg();

  this->update_pub->publish(hello_msg);
}

void KnowledgeGraph::update_callback(
    knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg) {
  const auto &author_id = msg->node_id;
  const auto &element = msg->element_type;
  const auto &operation = msg->operation_type;
  const auto &ts = msg->stamp;

  // Ignore messages from self
  if (author_id == this->graph_id) {
    return;
  }

  // Check for out-of-order updates (except for sync operations)
  if (operation != knowledge_graph_msgs::msg::GraphUpdate::REQSYNC &&
      operation != knowledge_graph_msgs::msg::GraphUpdate::SYNC &&
      rclcpp::Time(ts) < this->last_ts) {
    RCLCPP_WARN(this->node->get_logger(), "UNORDERED UPDATE [%d] %lf > %lf",
                operation, this->last_ts.seconds(), rclcpp::Time(ts).seconds());
  }

  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  switch (element) {
  case knowledge_graph_msgs::msg::GraphUpdate::NODE: {
    for (const auto &node_msg : msg->nodes) {
      graph::Node node(node_msg);
      if (operation == knowledge_graph_msgs::msg::GraphUpdate::UPDATE) {
        graph::Graph::update_node(node);
      } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REMOVE) {
        graph::Graph::remove_node(node);
      }
    }
    break;
  }

  case knowledge_graph_msgs::msg::GraphUpdate::EDGE: {
    for (const auto &edge_msg : msg->edges) {
      graph::Edge edge(edge_msg);
      if (operation == knowledge_graph_msgs::msg::GraphUpdate::UPDATE) {
        graph::Graph::update_edge(edge);
      } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REMOVE) {
        graph::Graph::remove_edge(edge);
      }
    }
    break;
  }

  case knowledge_graph_msgs::msg::GraphUpdate::GRAPH: {
    graph::Graph remote_graph(msg->graph);

    if (operation == knowledge_graph_msgs::msg::GraphUpdate::SYNC &&
        msg->target_node == this->graph_id) {
      this->reqsync_timer->cancel();
      this->update_graph(remote_graph);
    } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REQSYNC) {
      // Respond with current graph state
      knowledge_graph_msgs::msg::GraphUpdate out_msg;
      out_msg.stamp = this->node->get_clock()->now();
      out_msg.node_id = this->graph_id;
      out_msg.target_node = msg->node_id;
      out_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::SYNC;
      out_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
      out_msg.graph = this->to_msg();

      this->update_pub->publish(out_msg);
      this->update_graph(remote_graph);
    }
    break;
  }
  }

  // Update last timestamp
  this->last_ts = std::max(this->last_ts, rclcpp::Time(ts));
}

void KnowledgeGraph::update_graph(const graph::Graph &graph) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  for (const auto &node : graph.get_nodes()) {
    graph::Graph::update_node(node);
  }

  for (const auto &edge : graph.get_edges()) {
    graph::Graph::update_edge(edge);
  }
}

knowledge_graph_msgs::msg::Graph KnowledgeGraph::to_msg() const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::to_msg();
}

graph::Node KnowledgeGraph::create_node(const std::string &name,
                                        const std::string &type) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  graph::Node node = graph::Graph::create_node(name, type);
  this->publish_update(
      knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
      knowledge_graph_msgs::msg::GraphUpdate::NODE,
      std::vector<knowledge_graph_msgs::msg::Node>{node.to_msg()}, {});
  return node;
}

void KnowledgeGraph::update_node(const graph::Node &node) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  graph::Graph::update_node(node);
  this->publish_update(
      knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
      knowledge_graph_msgs::msg::GraphUpdate::NODE,
      std::vector<knowledge_graph_msgs::msg::Node>{node.to_msg()}, {});
}

void KnowledgeGraph::update_nodes(const std::vector<graph::Node> &nodes) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  std::vector<knowledge_graph_msgs::msg::Node> node_msgs;
  for (const auto &node : nodes) {
    node_msgs.push_back(node.to_msg());
  }
  graph::Graph::update_nodes(nodes);
  this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                       knowledge_graph_msgs::msg::GraphUpdate::NODE, node_msgs,
                       {});
}

bool KnowledgeGraph::remove_node(const graph::Node &node) {
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

const std::vector<graph::Node>
KnowledgeGraph::remove_nodes(const std::vector<graph::Node> &nodes) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  std::vector<graph::Node> removed_nodes = graph::Graph::remove_nodes(nodes);
  std::vector<knowledge_graph_msgs::msg::Node> node_msgs;
  for (const auto &node : removed_nodes) {
    node_msgs.push_back(node.to_msg());
  }
  if (!removed_nodes.empty()) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                         knowledge_graph_msgs::msg::GraphUpdate::NODE,
                         node_msgs, {});
  }
  return removed_nodes;
}

bool KnowledgeGraph::has_node(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::has_node(name);
}

int KnowledgeGraph::get_num_nodes() const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_num_nodes();
}

std::vector<graph::Node> KnowledgeGraph::get_nodes() const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_nodes();
}

graph::Node KnowledgeGraph::get_node(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_node(name);
}

graph::Edge KnowledgeGraph::create_edge(const std::string &type,
                                        const std::string &source_node,
                                        const std::string &target_node) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  graph::Edge edge = graph::Graph::create_edge(type, source_node, target_node);
  this->publish_update(
      knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
      knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
      std::vector<knowledge_graph_msgs::msg::Edge>{edge.to_msg()});
  return edge;
}

void KnowledgeGraph::update_edge(const graph::Edge &edge) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  graph::Graph::update_edge(edge);
  this->publish_update(
      knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
      knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
      std::vector<knowledge_graph_msgs::msg::Edge>{edge.to_msg()});
}

void KnowledgeGraph::update_edges(const std::vector<graph::Edge> &edges) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  std::vector<knowledge_graph_msgs::msg::Edge> edge_msgs;
  for (const auto &edge : edges) {
    edge_msgs.push_back(edge.to_msg());
  }
  graph::Graph::update_edges(edges);
  this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                       knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
                       edge_msgs);
}

bool KnowledgeGraph::remove_edge(const graph::Edge &edge) {
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

const std::vector<graph::Edge>
KnowledgeGraph::remove_edges(const std::vector<graph::Edge> &edges) {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  std::vector<graph::Edge> removed_edges = graph::Graph::remove_edges(edges);
  std::vector<knowledge_graph_msgs::msg::Edge> edge_msgs;
  for (const auto &edge : removed_edges) {
    edge_msgs.push_back(edge.to_msg());
  }
  if (!removed_edges.empty()) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                         knowledge_graph_msgs::msg::GraphUpdate::EDGE, {},
                         edge_msgs);
  }
  return removed_edges;
}

bool KnowledgeGraph::has_edge(const std::string &type,
                              const std::string &source_node,
                              const std::string &target_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::has_edge(type, source_node, target_node);
}

int KnowledgeGraph::get_num_edges() const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_num_edges();
}

std::vector<graph::Edge> KnowledgeGraph::get_edges() const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges();
}

std::vector<graph::Edge>
KnowledgeGraph::get_edges_from_node(const std::string &source_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_from_node(source_node);
}

std::vector<graph::Edge>
KnowledgeGraph::get_edges_to_node(const std::string &target_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_to_node(target_node);
}

std::vector<graph::Edge>
KnowledgeGraph::get_edges_between_nodes(const std::string &source_node,
                                        const std::string &target_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_between_nodes(source_node, target_node);
}

std::vector<graph::Edge>
KnowledgeGraph::get_edges_by_type(const std::string &type) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_by_type(type);
}

std::vector<graph::Edge> KnowledgeGraph::get_edges_from_node_by_type(
    const std::string &type, const std::string &source_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_from_node_by_type(type, source_node);
}

std::vector<graph::Edge> KnowledgeGraph::get_edges_to_node_by_type(
    const std::string &type, const std::string &target_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edges_to_node_by_type(type, target_node);
}

graph::Edge KnowledgeGraph::get_edge(const std::string &type,
                                     const std::string &source_node,
                                     const std::string &target_node) const {
  std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
  return graph::Graph::get_edge(type, source_node, target_node);
}
