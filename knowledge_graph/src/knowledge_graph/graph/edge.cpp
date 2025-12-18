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

#include <string>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/properties_container.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"

using namespace knowledge_graph::graph;

Edge::Edge(const std::string &type, const std::string &source_node,
           const std::string &target_node)
    : type_(type), source_node_(source_node), target_node_(target_node) {};

Edge::Edge(const knowledge_graph_msgs::msg::Edge &msg)
    : PropertiesContainer(msg.properties), type_(msg.type),
      source_node_(msg.source_node), target_node_(msg.target_node) {};

std::string Edge::get_type() const { return this->type_; }

std::string Edge::get_source_node() const { return this->source_node_; }

std::string Edge::get_target_node() const { return this->target_node_; }

knowledge_graph_msgs::msg::Edge Edge::to_msg() const {
  knowledge_graph_msgs::msg::Edge edge_msg;
  edge_msg.type = this->type_;
  edge_msg.properties = this->properties_to_msg();
  edge_msg.source_node = this->source_node_;
  edge_msg.target_node = this->target_node_;
  return edge_msg;
}

std::string Edge::to_string() const {
  return "Edge(type: " + this->type_ + ", source: " + this->source_node_ +
         ", target: " + this->target_node_ + ")";
}
