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

#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph/graph/properties_container.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

using namespace knowledge_graph::graph;

Node::Node(const std::string &name, const std::string &type)
    : name_(name), type_(type) {};

Node::Node(const knowledge_graph_msgs::msg::Node &msg)
    : PropertiesContainer(msg.properties), name_(msg.name), type_(msg.type) {};

std::string Node::get_name() const { return this->name_; }

std::string Node::get_type() const { return this->type_; }

knowledge_graph_msgs::msg::Node Node::to_msg() const {
  knowledge_graph_msgs::msg::Node node_msg;
  node_msg.name = this->name_;
  node_msg.type = this->type_;
  node_msg.properties = this->properties_to_msg();
  return node_msg;
}

std::string Node::to_string() const {
  return "Node(name: " + this->name_ + ", type: " + this->type_ + ")";
}
