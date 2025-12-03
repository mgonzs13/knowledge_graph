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
#include <shared_mutex>
#include <string>
#include <vector>

#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace knowledge_graph {

using std::placeholders::_1;
using namespace std::chrono_literals;

/// @brief Synchronization request timeout in seconds.
static constexpr double SYNC_TIMEOUT_SEC = 1.0;

/// @brief Timer interval for synchronization requests in milliseconds.
static constexpr std::chrono::milliseconds SYNC_TIMER_INTERVAL{100};

KnowledgeGraph::KnowledgeGraph(rclcpp::Node *provided_node)
    : provided_node(provided_node) {

  this->graph = std::make_unique<knowledge_graph_msgs::msg::Graph>();
  this->graph_id = this->provided_node->get_name();

  this->update_pub =
      this->provided_node
          ->create_publisher<knowledge_graph_msgs::msg::GraphUpdate>(
              "graph_update", rclcpp::QoS(100).reliable());
  this->update_sub =
      provided_node
          ->create_subscription<knowledge_graph_msgs::msg::GraphUpdate>(
              "graph_update", rclcpp::QoS(100).reliable(),
              std::bind(&KnowledgeGraph::update_callback, this, _1));

  this->last_ts = this->provided_node->now();
  this->start_time = this->provided_node->now();

  this->reqsync_timer = this->provided_node->create_wall_timer(
      SYNC_TIMER_INTERVAL,
      std::bind(&KnowledgeGraph::reqsync_timer_callback, this));
  this->reqsync_timer_callback();
}

void KnowledgeGraph::publish_update(
    uint8_t operation, uint8_t element,
    const std::optional<knowledge_graph_msgs::msg::Node> &node,
    const std::optional<knowledge_graph_msgs::msg::Edge> &edge,
    const std::string &removed_node) {
  knowledge_graph_msgs::msg::GraphUpdate update_msg;
  update_msg.stamp = this->provided_node->get_clock()->now();
  update_msg.node_id = this->graph_id;
  update_msg.operation_type = operation;
  update_msg.element_type = element;

  if (node.has_value()) {
    update_msg.node = node.value();
  }
  if (edge.has_value()) {
    update_msg.edge = edge.value();
  }
  if (!removed_node.empty()) {
    update_msg.removed_node = removed_node;
  }

  this->update_pub->publish(update_msg);
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::filter_edges(const EdgePredicate &predicate) const {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  std::vector<knowledge_graph_msgs::msg::Edge> result;
  std::copy_if(this->graph->edges.begin(), this->graph->edges.end(),
               std::back_inserter(result), predicate);
  return result;
}

void KnowledgeGraph::reqsync_timer_callback() {
  if ((this->provided_node->get_clock()->now() - this->start_time).seconds() >
      SYNC_TIMEOUT_SEC) {
    this->reqsync_timer->cancel();
  }

  knowledge_graph_msgs::msg::GraphUpdate hello_msg;
  hello_msg.stamp = this->provided_node->get_clock()->now();
  hello_msg.node_id = this->graph_id;
  hello_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  hello_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  {
    std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
    hello_msg.graph = *this->graph;
  }
  this->update_pub->publish(hello_msg);
}

bool KnowledgeGraph::remove_node(const std::string &node, bool sync) {
  std::unique_lock<std::shared_mutex> lock(this->graph_mutex);

  auto it =
      std::find_if(this->graph->nodes.begin(), this->graph->nodes.end(),
                   [&node](const auto &n) { return n.node_name == node; });

  if (it == this->graph->nodes.end()) {
    return false;
  }

  this->graph->nodes.erase(it);

  // Remove all edges connected to the removed node using erase-remove idiom
  this->graph->edges.erase(std::remove_if(this->graph->edges.begin(),
                                          this->graph->edges.end(),
                                          [&node](const auto &edge) {
                                            return edge.source_node == node ||
                                                   edge.target_node == node;
                                          }),
                           this->graph->edges.end());

  this->last_ts = this->provided_node->get_clock()->now();
  lock.unlock();

  if (sync) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                         knowledge_graph_msgs::msg::GraphUpdate::NODE,
                         std::nullopt, std::nullopt, node);
  }

  return true;
}

bool KnowledgeGraph::exist_node(const std::string &node) {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  return std::any_of(this->graph->nodes.begin(), this->graph->nodes.end(),
                     [&node](const auto &n) { return n.node_name == node; });
}

std::optional<knowledge_graph_msgs::msg::Node>
KnowledgeGraph::get_node(const std::string &node) {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  auto it =
      std::find_if(this->graph->nodes.begin(), this->graph->nodes.end(),
                   [&node](const auto &n) { return n.node_name == node; });
  if (it != this->graph->nodes.end()) {
    return *it;
  }
  return {};
}

const std::vector<std::string> KnowledgeGraph::get_node_names() {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  std::vector<std::string> ret;
  ret.reserve(this->graph->nodes.size());
  std::transform(this->graph->nodes.begin(), this->graph->nodes.end(),
                 std::back_inserter(ret),
                 [](const auto &node) { return node.node_name; });
  return ret;
}

std::vector<knowledge_graph_msgs::msg::Node> KnowledgeGraph::get_nodes() const {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  return this->graph->nodes;
}

std::vector<knowledge_graph_msgs::msg::Edge> KnowledgeGraph::get_edges() const {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  return this->graph->edges;
}

bool KnowledgeGraph::remove_edge(const knowledge_graph_msgs::msg::Edge &edge,
                                 bool sync) {
  std::unique_lock<std::shared_mutex> lock(this->graph_mutex);

  auto it = std::find_if(this->graph->edges.begin(), this->graph->edges.end(),
                         [&edge](const auto &e) {
                           return e.source_node == edge.source_node &&
                                  e.target_node == edge.target_node &&
                                  e.edge_class == edge.edge_class;
                         });

  if (it == this->graph->edges.end()) {
    return false;
  }

  this->graph->edges.erase(it);
  this->last_ts = this->provided_node->get_clock()->now();
  lock.unlock();

  if (sync) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::REMOVE,
                         knowledge_graph_msgs::msg::GraphUpdate::EDGE,
                         std::nullopt, edge);
  }

  return true;
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_edges(const std::string &source,
                          const std::string &target) {
  return this->filter_edges([&source, &target](const auto &edge) {
    return edge.source_node == source && edge.target_node == target;
  });
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_edges(const std::string &edge_class) {
  return this->filter_edges([&edge_class](const auto &edge) {
    return edge.edge_class == edge_class;
  });
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_out_edges(const std::string &source) {
  return this->filter_edges(
      [&source](const auto &edge) { return edge.source_node == source; });
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_in_edges(const std::string &target) {
  return this->filter_edges(
      [&target](const auto &edge) { return edge.target_node == target; });
}

size_t KnowledgeGraph::get_num_edges() const {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  return this->graph->edges.size();
}

size_t KnowledgeGraph::get_num_nodes() const {
  std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
  return this->graph->nodes.size();
}

bool KnowledgeGraph::update_node(const knowledge_graph_msgs::msg::Node &node,
                                 bool sync) {
  {
    std::unique_lock<std::shared_mutex> lock(this->graph_mutex);
    auto it = std::find_if(
        this->graph->nodes.begin(), this->graph->nodes.end(),
        [&node](const auto &n) { return n.node_name == node.node_name; });

    if (it != this->graph->nodes.end()) {
      *it = node;
    } else {
      this->graph->nodes.push_back(node);
    }
    this->last_ts = this->provided_node->get_clock()->now();
  }

  if (sync) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                         knowledge_graph_msgs::msg::GraphUpdate::NODE, node);
  }

  return true;
}

bool KnowledgeGraph::update_edge(const knowledge_graph_msgs::msg::Edge &edge,
                                 bool sync) {
  // Check node existence with shared lock first
  {
    std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
    bool source_exists = std::any_of(
        this->graph->nodes.begin(), this->graph->nodes.end(),
        [&edge](const auto &n) { return n.node_name == edge.source_node; });
    if (!source_exists) {
      RCLCPP_ERROR_STREAM(this->provided_node->get_logger(),
                          "Node source [" << edge.source_node
                                          << "] doesn't exist adding edge");
      return false;
    }

    bool target_exists = std::any_of(
        this->graph->nodes.begin(), this->graph->nodes.end(),
        [&edge](const auto &n) { return n.node_name == edge.target_node; });
    if (!target_exists) {
      RCLCPP_ERROR_STREAM(this->provided_node->get_logger(),
                          "Node target [" << edge.target_node
                                          << "] doesn't exist adding edge");
      return false;
    }
  }

  {
    std::unique_lock<std::shared_mutex> lock(this->graph_mutex);
    auto it = std::find_if(this->graph->edges.begin(), this->graph->edges.end(),
                           [&edge](const auto &e) {
                             return e.source_node == edge.source_node &&
                                    e.target_node == edge.target_node &&
                                    e.edge_class == edge.edge_class;
                           });

    if (it != this->graph->edges.end()) {
      *it = edge;
    } else {
      this->graph->edges.push_back(edge);
    }
    this->last_ts = this->provided_node->get_clock()->now();
  }

  if (sync) {
    this->publish_update(knowledge_graph_msgs::msg::GraphUpdate::UPDATE,
                         knowledge_graph_msgs::msg::GraphUpdate::EDGE,
                         std::nullopt, edge);
  }

  return true;
}

void KnowledgeGraph::update_graph(knowledge_graph_msgs::msg::Graph msg) {
  for (const auto &n : msg.nodes) {
    this->update_node(n, false);
  }

  for (const auto &e : msg.edges) {
    this->update_edge(e, false);
  }

  std::unique_lock<std::shared_mutex> lock(this->graph_mutex);
  this->last_ts = this->provided_node->get_clock()->now();
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
      operation != knowledge_graph_msgs::msg::GraphUpdate::SYNC) {
    std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
    if (rclcpp::Time(ts) < this->last_ts) {
      RCLCPP_ERROR(this->provided_node->get_logger(),
                   "UNORDERED UPDATE [%d] %lf > %lf", operation,
                   this->last_ts.seconds(), rclcpp::Time(ts).seconds());
    }
  }

  switch (element) {
  case knowledge_graph_msgs::msg::GraphUpdate::NODE:
    if (operation == knowledge_graph_msgs::msg::GraphUpdate::UPDATE) {
      this->update_node(msg->node, false);
    } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REMOVE) {
      this->remove_node(msg->removed_node, false);
    }
    break;

  case knowledge_graph_msgs::msg::GraphUpdate::EDGE:
    if (operation == knowledge_graph_msgs::msg::GraphUpdate::UPDATE) {
      this->update_edge(msg->edge, false);
    } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REMOVE) {
      this->remove_edge(msg->edge, false);
    }
    break;

  case knowledge_graph_msgs::msg::GraphUpdate::GRAPH:
    if (operation == knowledge_graph_msgs::msg::GraphUpdate::SYNC &&
        msg->target_node == this->graph_id) {
      this->reqsync_timer->cancel();
      this->update_graph(msg->graph);
    } else if (operation == knowledge_graph_msgs::msg::GraphUpdate::REQSYNC) {
      // Respond with current graph state
      knowledge_graph_msgs::msg::GraphUpdate out_msg;
      out_msg.stamp = this->provided_node->get_clock()->now();
      out_msg.node_id = this->graph_id;
      out_msg.target_node = msg->node_id;
      out_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::SYNC;
      out_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
      {
        std::shared_lock<std::shared_mutex> lock(this->graph_mutex);
        out_msg.graph = *this->graph;
      }
      this->update_pub->publish(out_msg);

      this->update_graph(msg->graph);
    }
    break;
  }
}

} // namespace knowledge_graph
