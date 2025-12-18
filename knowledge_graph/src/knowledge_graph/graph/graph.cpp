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
#include <string>
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

/************************************************************
 * Node Management Functions
 ************************************************************/
Node Graph::create_node(const std::string &name, const std::string &type) {
  if (this->has_node(name)) {
    this->get_node(name);
  }

  Node node(name, type);
  this->nodes_.push_back(node);
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

void Graph::update_node(const Node &node) {
  for (auto &existing_node : this->nodes_) {
    if (existing_node.get_name() == node.get_name()) {
      existing_node = node;
      return;
    }
  }
  this->nodes_.push_back(node);
}

void Graph::update_nodes(const std::vector<Node> &nodes) {
  for (const auto &node : nodes) {
    this->update_node(node);
  }
}

bool Graph::remove_node(const Node &node) {
  for (auto it = this->nodes_.begin(); it != this->nodes_.end(); ++it) {
    if (it->get_name() == node.get_name()) {
      this->nodes_.erase(it);
      return true;
    }
  }
  return false;
}

void Graph::remove_nodes(const std::vector<Node> &nodes) {
  for (const auto &node : nodes) {
    this->remove_node(node);
  }
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
  return edge;
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

void Graph::update_edge(const Edge &edge) {
  for (auto &existing_edge : this->edges_) {
    if (existing_edge.get_type() == edge.get_type() &&
        existing_edge.get_source_node() == edge.get_source_node() &&
        existing_edge.get_target_node() == edge.get_target_node()) {
      existing_edge = edge;
      return;
    }
  }
  this->edges_.push_back(edge);
}

void Graph::update_edges(const std::vector<Edge> &edges) {
  for (const auto &edge : edges) {
    this->update_edge(edge);
  }
}

bool Graph::remove_edge(const Edge &edge) {
  return this->remove_edges_if([&](const Edge &e) {
    return e.get_type() == edge.get_type() &&
           e.get_source_node() == edge.get_source_node() &&
           e.get_target_node() == edge.get_target_node();
  });
}

void Graph::remove_edges(const std::vector<Edge> &edges) {
  for (auto &edge : edges) {
    this->remove_edge(edge);
  }
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
