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

#include <algorithm>
#include <functional>
#include <string>
#include <variant>
#include <vector>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/graph.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"

using namespace knowledge_graph::graph;

Graph::Graph(const knowledge_graph_msgs::msg::Graph &msg) {
  for (const auto &node_msg : msg.nodes) {
    Node node(node_msg);
    this->nodes_.push_back(node);
  }

  for (const auto &edge_msg : msg.edges) {
    Edge edge(edge_msg);
    this->edges_.push_back(edge);
  }
}

void Graph::update_graph(const Graph &graph) {
  for (const auto &node : graph.get_nodes()) {
    this->update_node(node);
  }

  for (const auto &edge : graph.get_edges()) {
    this->update_edge(edge);
  }
}

knowledge_graph_msgs::msg::Graph Graph::to_msg() const {
  knowledge_graph_msgs::msg::Graph graph_msg;

  for (const auto &node : this->nodes_) {
    graph_msg.nodes.push_back(node.to_msg());
  }

  for (const auto &edge : this->edges_) {
    graph_msg.edges.push_back(edge.to_msg());
  }

  return graph_msg;
}

void Graph::add_callback(
    std::function<void(const std::string &, const std::string &,
                       const std::vector<std::variant<Node, Edge>> &)>
        callback) {
  this->callbacks_.push_back(callback);
}

void Graph::clear_callbacks() { this->callbacks_.clear(); }

void Graph::notify_callbacks(
    const std::string &operation, const std::string &element_type,
    const std::vector<std::variant<Node, Edge>> &elements) {
  for (auto &cb : this->callbacks_) {
    cb(operation, element_type, elements);
  }
}

/************************************************************
 * Node Management Functions
 ************************************************************/
Node Graph::create_node(const std::string &name, const std::string &type) {
  if (this->has_node(name)) {
    this->get_node(name);
  }

  Node node(name, type);
  this->nodes_.push_back(node);
  this->notify_callbacks("add", "node", {node});
  return node;
}

bool Graph::has_node(const std::string &name) const {
  for (const auto &node : this->nodes_) {
    if (node.get_name() == name) {
      return true;
    }
  }
  return false;
}

bool Graph::has_node(const Node &node) const {
  return this->has_node(node.get_name());
}

bool Graph::update_node_internal(const Node &node) {
  bool existing = false;
  for (auto &existing_node : this->nodes_) {
    if (existing_node.get_name() == node.get_name()) {
      existing_node = node;
      existing = true;
      break;
    }
  }
  if (!existing) {
    this->nodes_.push_back(node);
  }
  return existing;
}

void Graph::update_node(const Node &node) {
  bool existing = this->update_node_internal(node);
  this->notify_callbacks(existing ? "update" : "add", "node", {node});
}

void Graph::update_nodes(const std::vector<Node> &nodes) {
  std::vector<Node> added;
  std::vector<Node> updated;
  for (const auto &node : nodes) {
    bool existing = this->update_node_internal(node);
    if (existing) {
      updated.push_back(node);
    } else {
      added.push_back(node);
    }
  }
  if (!added.empty()) {
    this->notify_callbacks(
        "add", "node",
        std::vector<std::variant<Node, Edge>>(added.begin(), added.end()));
  }
  if (!updated.empty()) {
    this->notify_callbacks(
        "update", "node",
        std::vector<std::variant<Node, Edge>>(updated.begin(), updated.end()));
  }
}

bool Graph::remove_node_internal(const Node &node) {
  for (auto it = this->nodes_.begin(); it != this->nodes_.end(); ++it) {
    if (it->get_name() == node.get_name()) {
      this->nodes_.erase(it);
      return true;
    }
  }
  return false;
}

bool Graph::remove_node(const Node &node) {
  bool removed = this->remove_node_internal(node);

  for (const auto &edge : this->get_edges()) {
    if (edge.get_source_node() == node.get_name() ||
        edge.get_target_node() == node.get_name()) {
      this->remove_edge(edge);
    }
  }

  if (removed) {
    this->notify_callbacks("remove", "node", {node});
  }
  return removed;
}

const std::vector<Node> Graph::remove_nodes(const std::vector<Node> &nodes) {
  std::vector<Node> removed;
  for (const auto &node : nodes) {
    if (this->remove_node_internal(node)) {
      removed.push_back(node);
    }
  }
  if (!removed.empty()) {
    this->notify_callbacks(
        "remove", "node",
        std::vector<std::variant<Node, Edge>>(removed.begin(), removed.end()));
  }
  return removed;
}

int Graph::get_num_nodes() const { return this->nodes_.size(); }

std::vector<Node> Graph::get_nodes() const { return this->nodes_; }

Node Graph::get_node(const std::string &name) const {
  for (const auto &node : this->nodes_) {
    if (node.get_name() == name) {
      return node;
    }
  }
  throw std::runtime_error("Node not found: " + name);
}

/************************************************************
 * Edge Management Functions
 ************************************************************/
Edge Graph::create_edge(const std::string &type, const std::string &source_node,
                        const std::string &target_node) {

  if (!this->has_node(source_node)) {
    throw std::runtime_error("Source node does not exist: " + source_node);
  }

  if (!this->has_node(target_node)) {
    throw std::runtime_error("Target node does not exist: " + target_node);
  }

  if (this->has_edge(type, source_node, target_node)) {
    this->get_edge(type, source_node, target_node);
  }

  Edge edge(type, source_node, target_node);
  this->edges_.push_back(edge);
  this->notify_callbacks("add", "edge", {edge});
  return edge;
}

bool Graph::update_edge_internal(const Edge &edge) {
  bool existing = false;
  for (auto &existing_edge : this->edges_) {
    if (existing_edge.get_type() == edge.get_type() &&
        existing_edge.get_source_node() == edge.get_source_node() &&
        existing_edge.get_target_node() == edge.get_target_node()) {
      existing_edge = edge;
      existing = true;
      break;
    }
  }

  if (!existing) {
    if (!this->has_node(edge.get_source_node())) {
      throw std::runtime_error("Source node does not exist: " +
                               edge.get_source_node());
    }

    if (!this->has_node(edge.get_target_node())) {
      throw std::runtime_error("Target node does not exist: " +
                               edge.get_target_node());
    }
    this->edges_.push_back(edge);
  }
  return existing;
}

void Graph::update_edge(const Edge &edge) {
  bool existing = this->update_edge_internal(edge);
  this->notify_callbacks(existing ? "update" : "add", "edge", {edge});
}

void Graph::update_edges(const std::vector<Edge> &edges) {
  std::vector<Edge> added;
  std::vector<Edge> updated;
  for (const auto &edge : edges) {
    bool existing = this->update_edge_internal(edge);
    if (existing) {
      updated.push_back(edge);
    } else {
      added.push_back(edge);
    }
  }
  if (!added.empty()) {
    this->notify_callbacks(
        "add", "edge",
        std::vector<std::variant<Node, Edge>>(added.begin(), added.end()));
  }
  if (!updated.empty()) {
    this->notify_callbacks(
        "update", "edge",
        std::vector<std::variant<Node, Edge>>(updated.begin(), updated.end()));
  }
}

bool Graph::remove_edge_internal(const Edge &edge) {
  for (auto it = this->edges_.begin(); it != this->edges_.end(); ++it) {
    if (it->get_type() == edge.get_type() &&
        it->get_source_node() == edge.get_source_node() &&
        it->get_target_node() == edge.get_target_node()) {
      this->edges_.erase(it);
      return true;
    }
  }
  return false;
}

bool Graph::remove_edge(const Edge &edge) {
  bool removed = this->remove_edge_internal(edge);
  if (removed) {
    this->notify_callbacks("remove", "edge", {edge});
  }
  return removed;
}

const std::vector<Edge> Graph::remove_edges(const std::vector<Edge> &edges) {
  std::vector<Edge> removed;
  for (auto &edge : edges) {
    if (this->remove_edge_internal(edge)) {
      removed.push_back(edge);
    }
  }
  if (!removed.empty()) {
    this->notify_callbacks(
        "remove", "edge",
        std::vector<std::variant<Node, Edge>>(removed.begin(), removed.end()));
  }
  return removed;
}

template <typename Predicate>
std::vector<Edge> Graph::filter_edges(Predicate pred) const {
  std::vector<Edge> result;
  for (const auto &edge : edges_) {
    if (pred(edge)) {
      result.push_back(edge);
    }
  }
  return result;
}

template <typename Predicate> bool Graph::remove_edges_if(Predicate pred) {
  auto it = std::remove_if(edges_.begin(), edges_.end(), pred);
  if (it != edges_.end()) {
    edges_.erase(it, edges_.end());
    return true;
  }
  return false;
}

bool Graph::has_edge(const std::string &type, const std::string &source_node,
                     const std::string &target_node) const {
  for (const auto &edge : this->edges_) {
    if (edge.get_type() == type && edge.get_source_node() == source_node &&
        edge.get_target_node() == target_node) {
      return true;
    }
  }
  return false;
}

bool Graph::has_edge(const Edge &edge) const {
  return this->has_edge(edge.get_type(), edge.get_source_node(),
                        edge.get_target_node());
}

int Graph::get_num_edges() const { return this->edges_.size(); }

std::vector<Edge> Graph::get_edges() const { return this->edges_; }

std::vector<Edge>
Graph::get_edges_from_node(const std::string &source_node) const {
  return this->filter_edges(
      [&](const Edge &e) { return e.get_source_node() == source_node; });
}

std::vector<Edge>
Graph::get_edges_to_node(const std::string &target_node) const {
  return this->filter_edges(
      [&](const Edge &e) { return e.get_target_node() == target_node; });
}

std::vector<Edge>
Graph::get_edges_between_nodes(const std::string &source_node,
                               const std::string &target_node) const {
  return this->filter_edges([&](const Edge &e) {
    return e.get_source_node() == source_node &&
           e.get_target_node() == target_node;
  });
}

std::vector<Edge> Graph::get_edges_by_type(const std::string &type) const {
  return this->filter_edges(
      [&](const Edge &e) { return e.get_type() == type; });
}

std::vector<Edge>
Graph::get_edges_from_node_by_type(const std::string &type,
                                   const std::string &source_node) const {
  return this->filter_edges([&](const Edge &e) {
    return e.get_source_node() == source_node && e.get_type() == type;
  });
}

std::vector<Edge>
Graph::get_edges_to_node_by_type(const std::string &type,
                                 const std::string &target_node) const {
  return this->filter_edges([&](const Edge &e) {
    return e.get_target_node() == target_node && e.get_type() == type;
  });
}

Edge Graph::get_edge(const std::string &type, const std::string &source_node,
                     const std::string &target_node) const {
  for (const auto &edge : this->edges_) {
    if (edge.get_type() == type && edge.get_source_node() == source_node &&
        edge.get_target_node() == target_node) {
      return edge;
    }
  }
  throw std::runtime_error("Edge not found: " + type + " from " + source_node +
                           " to " + target_node);
}
