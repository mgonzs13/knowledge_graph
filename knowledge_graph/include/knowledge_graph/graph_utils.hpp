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

#ifndef KNOWLEDGE_GRAPH__GRAPH_UTILS_HPP_
#define KNOWLEDGE_GRAPH__GRAPH_UTILS_HPP_

#include <iostream>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

namespace knowledge_graph {

template <class T>
knowledge_graph_msgs::msg::Content new_content(const T &content) {
  (void)content;
  knowledge_graph_msgs::msg::Content ret;
  ret.type = knowledge_graph_msgs::msg::Content::ERROR;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content new_content<bool>(const bool &content);
template <>
knowledge_graph_msgs::msg::Content new_content<int>(const int &content);
template <>
knowledge_graph_msgs::msg::Content new_content<float>(const float &content);
template <>
knowledge_graph_msgs::msg::Content new_content<double>(const double &content);

template <>
knowledge_graph_msgs::msg::Content
new_content<std::string>(const std::string &content);
template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<bool>>(const std::vector<bool> &content);
template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<int>>(const std::vector<int> &content);
template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<float>>(const std::vector<float> &content);
template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<double>>(const std::vector<double> &content);
template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<std::string>>(const std::vector<std::string> &content);

knowledge_graph_msgs::msg::Node new_node(const std::string &node_name,
                                         const std::string &node_class);

knowledge_graph_msgs::msg::Edge new_edge(const std::string &edge_class,
                                         const std::string &edge_source,
                                         const std::string &edge_target);

template <class T>
std::optional<T>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  return {};
}

template <class T>
bool add_property(std::vector<knowledge_graph_msgs::msg::Property> &properties,
                  const std::string key, const T &content) {
  bool found = false;
  auto newc = new_content<T>(content);

  if (newc.type == knowledge_graph_msgs::msg::Content::ERROR) {
    std::cerr << "Adding a property of type ERROR" << std::endl;
    return false;
  }

  auto it = properties.begin();
  while (!found && it != properties.end()) {
    if (it->key == key) {
      found = true;
      it->value = newc;
    } else {
      ++it;
    }
  }

  if (!found) {
    knowledge_graph_msgs::msg::Property prop;
    prop.key = key;
    prop.value = newc;
    properties.push_back(prop);
  }

  return true;
}

template <class T>
bool add_property(knowledge_graph_msgs::msg::Node &node, const std::string key,
                  const T &content) {
  return add_property<T>(node.properties, key, content);
}

template <class T>
bool add_property(knowledge_graph_msgs::msg::Edge &edge, const std::string key,
                  const T &content) {
  return add_property<T>(edge.properties, key, content);
}

template <class T>
std::optional<T>
get_property(std::vector<knowledge_graph_msgs::msg::Property> &properties,
             const std::string key) {
  auto it = properties.begin();
  while (it != properties.end()) {
    if (it->key == key) {
      return get_content<T>(it->value);
    } else {
      ++it;
    }
  }
  return {};
}

template <class T>
std::optional<T> get_property(knowledge_graph_msgs::msg::Node &node,
                              const std::string key) {
  return get_property<T>(node.properties);
}

template <class T>
std::optional<T> get_property(knowledge_graph_msgs::msg::Edge &edge,
                              const std::string key) {
  return get_property<T>(edge.properties);
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<std::string>>(const std::vector<std::string> &content);

template <>
std::optional<bool>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<int>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<float>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<double>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::string>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::vector<bool>>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::vector<int>>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::vector<float>>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::vector<double>>
get_content(const knowledge_graph_msgs::msg::Content &content);

template <>
std::optional<std::vector<std::string>>
get_content(const knowledge_graph_msgs::msg::Content &content);

uint8_t
get_property_type(std::vector<knowledge_graph_msgs::msg::Property> &properties,
                  const std::string key);

uint8_t get_property_type(knowledge_graph_msgs::msg::Node &node,
                          const std::string key);

uint8_t get_property_type(knowledge_graph_msgs::msg::Edge &edge,
                          const std::string key);

std::string to_string(uint8_t edge_type);

std::string to_string(const knowledge_graph_msgs::msg::Content &content);

uint8_t type_from_string(const std::string &type);

std::string to_string(const knowledge_graph_msgs::msg::Node &node);

std::string to_string(const knowledge_graph_msgs::msg::Edge &edge);

} // namespace knowledge_graph

#endif
