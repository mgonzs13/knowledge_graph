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

knowledge_graph_msgs::msg::Node new_node(const std::string &node_name,
                                         const std::string &node_class) {
  knowledge_graph_msgs::msg::Node ret;
  ret.node_name = node_name;
  ret.node_class = node_class;
  return ret;
}

template <class T>
knowledge_graph_msgs::msg::Content new_content(const T &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.type = knowledge_graph_msgs::msg::Content::ERROR;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content new_content<bool>(const bool &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.bool_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::BOOL;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content new_content<int>(const int &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.int_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::INT;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content new_content<float>(const float &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.float_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::FLOAT;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content new_content<double>(const double &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.double_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::DOUBLE;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::string>(const std::string &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.string_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::STRING;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<bool>>(const std::vector<bool> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.bool_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VBOOL;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<int>>(const std::vector<int> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.int_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VINT;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<float>>(const std::vector<float> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.float_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VFLOAT;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<double>>(const std::vector<double> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.double_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VDOUBLE;
  return ret;
}

template <>
knowledge_graph_msgs::msg::Content
new_content<std::vector<std::string>>(const std::vector<std::string> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.string_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VSTRING;
  return ret;
}

knowledge_graph_msgs::msg::Edge new_edge(const std::string &edge_class,
                                         const std::string &edge_source,
                                         const std::string &edge_target) {
  knowledge_graph_msgs::msg::Edge ret;
  ret.edge_class = edge_class;
  ret.source_node = edge_source;
  ret.target_node = edge_target;
  return ret;
}

template <class T>
std::optional<T>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  return {};
}

template <>
std::optional<bool>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::BOOL) {
    return content.bool_value;
  } else {
    return {};
  }
}

template <>
std::optional<int>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::INT) {
    return content.int_value;
  } else {
    return {};
  }
}

template <>
std::optional<float>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::FLOAT) {
    return content.float_value;
  } else {
    return {};
  }
}

template <>
std::optional<double>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::DOUBLE) {
    return content.double_value;
  } else {
    return {};
  }
}

template <>
std::optional<std::string>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::STRING) {
    return content.string_value;
  } else {
    return {};
  }
}

template <>
std::optional<std::vector<bool>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VBOOL) {
    return content.bool_vector;
  } else {
    return {};
  }
}

template <>
std::optional<std::vector<int>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VINT) {
    return content.int_vector;
  } else {
    return {};
  }
}

template <>
std::optional<std::vector<float>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VFLOAT) {
    return content.float_vector;
  } else {
    return {};
  }
}

template <>
std::optional<std::vector<double>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VDOUBLE) {
    return content.double_vector;
  } else {
    return {};
  }
}

template <>
std::optional<std::vector<std::string>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VSTRING) {
    return content.string_vector;
  } else {
    return {};
  }
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

uint8_t
get_property_type(std::vector<knowledge_graph_msgs::msg::Property> &properties,
                  const std::string key) {
  auto it = properties.begin();
  while (it != properties.end()) {
    if (it->key == key) {
      return it->value.type;
    } else {
      ++it;
    }
  }
  return knowledge_graph_msgs::msg::Content::ERROR;
}

uint8_t get_property_type(knowledge_graph_msgs::msg::Node &node,
                          const std::string key) {
  return get_property_type(node.properties, key);
}

uint8_t get_property_type(knowledge_graph_msgs::msg::Edge &edge,
                          const std::string key) {
  return get_property_type(edge.properties, key);
}

std::string to_string(uint8_t edge_type) {
  switch (edge_type) {
  case knowledge_graph_msgs::msg::Content::BOOL:
    return "bool";
    break;
  case knowledge_graph_msgs::msg::Content::INT:
    return "int";
    break;
  case knowledge_graph_msgs::msg::Content::FLOAT:
    return "float";
    break;
  case knowledge_graph_msgs::msg::Content::DOUBLE:
    return "double";
    break;
  case knowledge_graph_msgs::msg::Content::STRING:
    return "string";
    break;
  case knowledge_graph_msgs::msg::Content::VBOOL:
    return "bool[]";
    break;
  case knowledge_graph_msgs::msg::Content::VINT:
    return "int[]";
    break;
  case knowledge_graph_msgs::msg::Content::VFLOAT:
    return "float[]";
    break;
  case knowledge_graph_msgs::msg::Content::VDOUBLE:
    return "double[]";
    break;
  case knowledge_graph_msgs::msg::Content::VSTRING:
    return "string[]";
    break;
  case knowledge_graph_msgs::msg::Content::ERROR:
    return "error";
    break;
  default:
    return "Unknown";
    break;
  }
}

std::string to_string(const knowledge_graph_msgs::msg::Content &content) {
  std::string ret;

  if (content.type == knowledge_graph_msgs::msg::Content::BOOL) {
    ret = content.bool_value ? "true" : "false";
  } else if (content.type == knowledge_graph_msgs::msg::Content::INT) {
    ret = std::to_string(content.int_value);
  } else if (content.type == knowledge_graph_msgs::msg::Content::FLOAT) {
    ret = std::to_string(content.float_value);
  } else if (content.type == knowledge_graph_msgs::msg::Content::DOUBLE) {
    ret = std::to_string(content.double_value);
  } else if (content.type == knowledge_graph_msgs::msg::Content::STRING) {
    ret = content.string_value;
  } else if (content.type == knowledge_graph_msgs::msg::Content::VBOOL) {
    ret = "[";
    for (const auto value : content.bool_vector) {
      ret = ret + " " + (value ? "true" : "false");
    }
    ret = ret + "]";
  } else if (content.type == knowledge_graph_msgs::msg::Content::VINT) {
    ret = "[";
    for (const auto value : content.int_vector) {
      ret = ret + " " + std::to_string(value);
    }
    ret = ret + "]";
  } else if (content.type == knowledge_graph_msgs::msg::Content::VFLOAT) {
    ret = "[";
    for (const auto value : content.float_vector) {
      ret = ret + " " + std::to_string(value);
    }
    ret = ret + "]";
  } else if (content.type == knowledge_graph_msgs::msg::Content::VDOUBLE) {
    ret = "[";
    for (const auto value : content.double_vector) {
      ret = ret + " " + std::to_string(value);
    }
    ret = ret + "]";
  } else if (content.type == knowledge_graph_msgs::msg::Content::VSTRING) {
    ret = "[";
    for (const auto value : content.string_vector) {
      ret = ret + " " + value;
    }
    ret = ret + "]";
  } else {
    ret = "error";
  }

  return ret;
}

uint8_t type_from_string(const std::string &type) {
  for (uint8_t i = 0; i < knowledge_graph_msgs::msg::Content::NUM_TYPES; i++) {
    if (type == to_string(i)) {
      return i;
    }
  }
  return knowledge_graph_msgs::msg::Content::ERROR;
}

std::string to_string(const knowledge_graph_msgs::msg::Node &node) {
  std::string ret;
  ret = ret + node.node_name + " (" + node.node_class + ")";

  for (const auto &prop : node.properties) {
    ret = ret + "\n\t" + prop.key + ": [" + to_string(prop.value) + "]";
  }

  return ret;
}

std::string to_string(const knowledge_graph_msgs::msg::Edge &edge) {
  std::string ret;
  ret = ret + " [" + edge.edge_class + "]" + edge.source_node + " -> " +
        edge.target_node;
  return ret;
}

} // namespace knowledge_graph

#endif
