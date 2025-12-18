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

#include "knowledge_graph/graph/properties.hpp"
#include "knowledge_graph/graph/properties_container.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

using namespace knowledge_graph::graph;

PropertiesContainer::PropertiesContainer(
    const std::vector<knowledge_graph_msgs::msg::Property> &msg)
    : properties_(msg) {}

template <typename T>
void PropertiesContainer::set_property(const std::string &key, const T &value) {
  this->properties_.set<T>(key, value);
}

template <typename T>
T PropertiesContainer::get_property(const std::string &key) const {
  return this->properties_.get<T>(key);
}

bool PropertiesContainer::has_property(const std::string &key) const {
  return this->properties_.has(key);
}

std::vector<knowledge_graph_msgs::msg::Property>
PropertiesContainer::properties_to_msg() const {
  return this->properties_.to_msg();
}
