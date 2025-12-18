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

using namespace knowledge_graph;

using std::placeholders::_1;
using namespace std::chrono_literals;

/// @brief Synchronization request timeout in seconds.
static constexpr double SYNC_TIMEOUT_SEC = 1.0;

/// @brief Timer interval for synchronization requests in milliseconds.
static constexpr std::chrono::milliseconds SYNC_TIMER_INTERVAL{100};

KnowledgeGraph::KnowledgeGraph(rclcpp::Node *node_ptr) : node(node_ptr) {

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
    std::lock_guard<std::recursive_mutex> lock(this->graph_mutex_);
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
