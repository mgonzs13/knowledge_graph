// Copyright 2024 Miguel Fernández Cortizas
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

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace knowledge_graph
{

using std::placeholders::_1;
using namespace std::chrono_literals;

KnowledgeGraph::KnowledgeGraph(rclcpp::Node * provided_node)
: provided_node(provided_node)
{

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
    100ms, std::bind(&KnowledgeGraph::reqsync_timer_callback, this));
  this->reqsync_timer_callback();
}

void KnowledgeGraph::reqsync_timer_callback()
{
  if ((this->provided_node->get_clock()->now() - this->start_time).seconds() >
    1.0)
  {
    this->reqsync_timer->cancel();
  }

  knowledge_graph_msgs::msg::GraphUpdate hello_msg;
  hello_msg.stamp = this->provided_node->get_clock()->now();
  hello_msg.node_id = this->graph_id;
  hello_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  hello_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  hello_msg.graph = *this->graph;
  this->update_pub->publish(hello_msg);
}

bool KnowledgeGraph::remove_node(const std::string node, bool sync)
{
  bool removed = false;
  auto it = this->graph->nodes.begin();
  while (!removed && it != this->graph->nodes.end()) {
    if (it->node_name == node) {
      it = this->graph->nodes.erase(it);
      removed = true;
    } else {
      ++it;
    }
  }

  if (removed) {
    auto ite = this->graph->edges.begin();
    while (ite != this->graph->edges.end()) {
      if (ite->source_node == node || ite->target_node == node) {
        this->graph->edges.erase(ite);
      } else {
        ++ite;
      }
    }
  }

  if (removed) {
    if (sync) {
      knowledge_graph_msgs::msg::GraphUpdate update_msg;
      update_msg.stamp = this->provided_node->get_clock()->now();
      update_msg.node_id = this->graph_id;
      update_msg.operation_type =
        knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
      update_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::NODE;
      update_msg.removed_node = node;
      this->update_pub->publish(update_msg);
    }

    this->last_ts = this->provided_node->get_clock()->now();
  }

  return removed;
}

bool KnowledgeGraph::exist_node(const std::string node)
{
  return this->get_node(node).has_value();
}

std::optional<knowledge_graph_msgs::msg::Node>
KnowledgeGraph::get_node(const std::string node)
{
  auto it = this->graph->nodes.begin();
  while (it != this->graph->nodes.end()) {
    if (it->node_name == node) {
      return *it;
    } else {
      ++it;
    }
  }

  return {};
}

const std::vector<std::string> KnowledgeGraph::get_node_names()
{
  std::vector<std::string> ret;
  for (const auto & node : this->graph->nodes) {
    ret.push_back(node.node_name);
  }
  return ret;
}

bool KnowledgeGraph::remove_edge(
  const knowledge_graph_msgs::msg::Edge & edge,
  bool sync)
{
  bool removed = false;

  auto it = this->graph->edges.begin();
  while (!removed && it != this->graph->edges.end()) {
    if (it->source_node == edge.source_node &&
      it->target_node == edge.target_node &&
      it->edge_class == edge.edge_class)
    {
      it = this->graph->edges.erase(it);
      removed = true;
    } else {
      ++it;
    }
  }

  if (removed) {
    if (sync) {
      knowledge_graph_msgs::msg::GraphUpdate update_msg;
      update_msg.stamp = this->provided_node->get_clock()->now();
      update_msg.node_id = this->graph_id;
      update_msg.operation_type =
        knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
      update_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::EDGE;
      update_msg.edge = edge;
      this->update_pub->publish(update_msg);
    }

    this->last_ts = this->provided_node->get_clock()->now();
  }

  return removed;
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_edges(
  const std::string & source,
  const std::string & target)
{
  std::vector<knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : this->graph->edges) {
    if (edge.source_node == source && edge.target_node == target) {
      ret.push_back(edge);
    }
  }

  return ret;
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_edges(const std::string & edge_class)
{
  std::vector<knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : this->graph->edges) {
    if (edge.edge_class == edge_class) {
      ret.push_back(edge);
    }
  }

  return ret;
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_out_edges(const std::string & source)
{
  std::vector<knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : this->graph->edges) {
    if (edge.source_node == source) {
      ret.push_back(edge);
    }
  }

  return ret;
}

std::vector<knowledge_graph_msgs::msg::Edge>
KnowledgeGraph::get_in_edges(const std::string & target)
{
  std::vector<knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : this->graph->edges) {
    if (edge.target_node == target) {
      ret.push_back(edge);
    }
  }

  return ret;
}

size_t KnowledgeGraph::get_num_edges() const
{
  return this->graph->edges.size();
}

size_t KnowledgeGraph::get_num_nodes() const
{
  return this->graph->nodes.size();
}

bool KnowledgeGraph::update_node(
  const knowledge_graph_msgs::msg::Node & node,
  bool sync)
{
  bool found = false;
  auto it = this->graph->nodes.begin();
  while (!found && it != this->graph->nodes.end()) {
    if (it->node_name == node.node_name) {
      *it = node;
      found = true;
    }
    ++it;
  }

  if (!found) {
    this->graph->nodes.push_back(node);
  }

  if (sync) {
    knowledge_graph_msgs::msg::GraphUpdate update_msg;
    update_msg.stamp = this->provided_node->get_clock()->now();
    update_msg.node_id = this->graph_id;
    update_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::UPDATE;
    update_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::NODE;
    update_msg.node = node;
    this->update_pub->publish(update_msg);
  }

  this->last_ts = this->provided_node->get_clock()->now();
  return true;
}

bool KnowledgeGraph::update_edge(
  const knowledge_graph_msgs::msg::Edge & edge,
  bool sync)
{

  if (!this->exist_node(edge.source_node)) {
    RCLCPP_ERROR_STREAM(
      this->provided_node->get_logger(),
      "Node source [" << edge.source_node
                      << "] doesn't exist adding edge");
    return false;
  }

  if (!this->exist_node(edge.target_node)) {
    RCLCPP_ERROR_STREAM(
      this->provided_node->get_logger(),
      "Node target [" << edge.target_node
                      << "] doesn't exist adding edge");
    return false;
  }

  bool found = false;
  auto it = this->graph->edges.begin();
  while (!found && it != this->graph->edges.end()) {
    if (it->source_node == edge.source_node &&
      it->target_node == edge.target_node &&
      it->edge_class == edge.edge_class)
    {
      *it = edge;
      found = true;
    }
    ++it;
  }

  knowledge_graph_msgs::msg::Edge mod_edge = edge;
  if (!found) {
    this->graph->edges.push_back(edge);
  }

  if (sync) {
    knowledge_graph_msgs::msg::GraphUpdate update_msg;
    update_msg.stamp = this->provided_node->get_clock()->now();
    update_msg.node_id = this->graph_id;
    update_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::UPDATE;
    update_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::EDGE;
    update_msg.edge = edge;
    this->update_pub->publish(update_msg);
  }

  this->last_ts = this->provided_node->get_clock()->now();

  return true;
}

void KnowledgeGraph::update_graph(knowledge_graph_msgs::msg::Graph msg)
{

  for (const auto & n : msg.nodes) {
    this->update_node(n, false);
  }

  for (const auto & e : msg.edges) {
    this->update_edge(e, false);
  }

  this->last_ts = this->provided_node->get_clock()->now();
}

void KnowledgeGraph::update_callback(
  knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg)
{
  const auto & author_id = msg->node_id;
  const auto & element = msg->element_type;
  const auto & operation = msg->operation_type;
  const auto & ts = msg->stamp;

  if (author_id == this->graph_id) {
    return;
  }

  if (rclcpp::Time(ts) < this->last_ts &&
    operation != knowledge_graph_msgs::msg::GraphUpdate::REQSYNC &&
    operation != knowledge_graph_msgs::msg::GraphUpdate::SYNC)
  {
    RCLCPP_ERROR(
      this->provided_node->get_logger(),
      "UNORDERER UPDATE [%d] %lf > %lf", operation,
      this->last_ts.seconds(), rclcpp::Time(ts).seconds());
  }

  switch (element) {
    case knowledge_graph_msgs::msg::GraphUpdate::NODE: {
        if (author_id == this->graph_id) {
          return;
        }

        switch (operation) {
          case knowledge_graph_msgs::msg::GraphUpdate::UPDATE: {
              this->update_node(msg->node, false);
              break;
            }

          case knowledge_graph_msgs::msg::GraphUpdate::REMOVE: {
              this->remove_node(msg->node.node_name, false);
              break;
            }
        }
      } break;

    case knowledge_graph_msgs::msg::GraphUpdate::EDGE: {
        if (author_id == this->graph_id) {
          return;
        }

        switch (operation) {
          case knowledge_graph_msgs::msg::GraphUpdate::UPDATE:
            this->update_edge(msg->edge, false);
            break;

          case knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            this->remove_edge(msg->edge, false);
            break;
        }
      } break;

    case knowledge_graph_msgs::msg::GraphUpdate::GRAPH: {
        switch (operation) {
          case knowledge_graph_msgs::msg::GraphUpdate::SYNC:

            if (msg->target_node == this->graph_id) {
              this->reqsync_timer->cancel();
              this->update_graph(msg->graph);
            }
            break;

          case knowledge_graph_msgs::msg::GraphUpdate::REQSYNC:
            if (msg->node_id != this->graph_id) {
              knowledge_graph_msgs::msg::GraphUpdate out_msg;
              out_msg.stamp = this->provided_node->get_clock()->now();
              out_msg.node_id = this->graph_id;
              out_msg.target_node = msg->node_id;
              out_msg.operation_type = knowledge_graph_msgs::msg::GraphUpdate::SYNC;
              out_msg.element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
              out_msg.graph = *this->graph;
              this->update_pub->publish(out_msg);

              this->update_graph(msg->graph);
            }
            break;
        }
      } break;
  }
}

} // namespace knowledge_graph
