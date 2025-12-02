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

#include <algorithm>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

namespace knowledge_graph {

namespace detail {

/**
 * @brief Helper function to convert a vector to string representation.
 * @tparam T The type of elements in the vector.
 * @tparam Converter Function type for converting elements to string.
 * @param vec The vector to convert.
 * @param converter Function to convert each element to string.
 * @return String representation of the vector.
 */
template <typename T, typename Converter>
inline std::string vector_to_string(const std::vector<T> &vec,
                                    Converter converter) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    if (i > 0) {
      oss << " ";
    }
    oss << " " << converter(vec[i]);
  }
  oss << "]";
  return oss.str();
}

} // namespace detail

/**
 * @brief Creates a new graph node.
 * @param node_name The name of the node.
 * @param node_class The class/type of the node.
 * @return A new Node message with the specified name and class.
 */
inline knowledge_graph_msgs::msg::Node new_node(const std::string &node_name,
                                                const std::string &node_class) {
  knowledge_graph_msgs::msg::Node ret;
  ret.node_name = node_name;
  ret.node_class = node_class;
  return ret;
}

/**
 * @brief Creates a new Content message from a value.
 * @tparam T The type of the content value.
 * @param content The content value.
 * @return A Content message with ERROR type for unsupported types.
 */
template <class T>
knowledge_graph_msgs::msg::Content new_content(const T &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.type = knowledge_graph_msgs::msg::Content::ERROR;
  return ret;
}

/**
 * @brief Creates a new Content message from a boolean value.
 * @param content The boolean value.
 * @return A Content message with BOOL type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<bool>(const bool &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.bool_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::BOOL;
  return ret;
}

/**
 * @brief Creates a new Content message from an integer value.
 * @param content The integer value.
 * @return A Content message with INT type.
 */
template <>
inline knowledge_graph_msgs::msg::Content new_content<int>(const int &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.int_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::INT;
  return ret;
}

/**
 * @brief Creates a new Content message from a float value.
 * @param content The float value.
 * @return A Content message with FLOAT type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<float>(const float &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.float_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::FLOAT;
  return ret;
}

/**
 * @brief Creates a new Content message from a double value.
 * @param content The double value.
 * @return A Content message with DOUBLE type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<double>(const double &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.double_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::DOUBLE;
  return ret;
}

/**
 * @brief Creates a new Content message from a string value.
 * @param content The string value.
 * @return A Content message with STRING type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::string>(const std::string &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.string_value = content;
  ret.type = knowledge_graph_msgs::msg::Content::STRING;
  return ret;
}

/**
 * @brief Creates a new Content message from a vector of booleans.
 * @param content The vector of boolean values.
 * @return A Content message with VBOOL type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::vector<bool>>(const std::vector<bool> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.bool_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VBOOL;
  return ret;
}

/**
 * @brief Creates a new Content message from a vector of integers.
 * @param content The vector of integer values.
 * @return A Content message with VINT type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::vector<int>>(const std::vector<int> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.int_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VINT;
  return ret;
}

/**
 * @brief Creates a new Content message from a vector of floats.
 * @param content The vector of float values.
 * @return A Content message with VFLOAT type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::vector<float>>(const std::vector<float> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.float_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VFLOAT;
  return ret;
}

/**
 * @brief Creates a new Content message from a vector of doubles.
 * @param content The vector of double values.
 * @return A Content message with VDOUBLE type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::vector<double>>(const std::vector<double> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.double_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VDOUBLE;
  return ret;
}

/**
 * @brief Creates a new Content message from a vector of strings.
 * @param content The vector of string values.
 * @return A Content message with VSTRING type.
 */
template <>
inline knowledge_graph_msgs::msg::Content
new_content<std::vector<std::string>>(const std::vector<std::string> &content) {
  knowledge_graph_msgs::msg::Content ret;
  ret.string_vector = content;
  ret.type = knowledge_graph_msgs::msg::Content::VSTRING;
  return ret;
}

/**
 * @brief Creates a new graph edge.
 * @param edge_class The class/type of the edge.
 * @param edge_source The name of the source node.
 * @param edge_target The name of the target node.
 * @return A new Edge message with the specified properties.
 */
inline knowledge_graph_msgs::msg::Edge
new_edge(const std::string &edge_class, const std::string &edge_source,
         const std::string &edge_target) {
  knowledge_graph_msgs::msg::Edge ret;
  ret.edge_class = edge_class;
  ret.source_node = edge_source;
  ret.target_node = edge_target;
  return ret;
}

/**
 * @brief Extracts a typed value from a Content message.
 * @tparam T The expected type of the content.
 * @param content The Content message to extract from.
 * @return An empty optional for unsupported types.
 */
template <class T>
std::optional<T>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  return {};
}

/**
 * @brief Extracts a boolean value from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the boolean if type matches, empty otherwise.
 */
template <>
inline std::optional<bool>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::BOOL) {
    return content.bool_value;
  } else {
    return {};
  }
}

/**
 * @brief Extracts an integer value from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the integer if type matches, empty otherwise.
 */
template <>
inline std::optional<int>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::INT) {
    return content.int_value;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a float value from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the float if type matches, empty otherwise.
 */
template <>
inline std::optional<float>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::FLOAT) {
    return content.float_value;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a double value from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the double if type matches, empty otherwise.
 */
template <>
inline std::optional<double>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::DOUBLE) {
    return content.double_value;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a string value from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the string if type matches, empty otherwise.
 */
template <>
inline std::optional<std::string>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::STRING) {
    return content.string_value;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a vector of booleans from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the vector if type matches, empty otherwise.
 */
template <>
inline std::optional<std::vector<bool>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VBOOL) {
    return content.bool_vector;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a vector of integers from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the vector if type matches, empty otherwise.
 */
template <>
inline std::optional<std::vector<int>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VINT) {
    return content.int_vector;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a vector of floats from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the vector if type matches, empty otherwise.
 */
template <>
inline std::optional<std::vector<float>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VFLOAT) {
    return content.float_vector;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a vector of doubles from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the vector if type matches, empty otherwise.
 */
template <>
inline std::optional<std::vector<double>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VDOUBLE) {
    return content.double_vector;
  } else {
    return {};
  }
}

/**
 * @brief Extracts a vector of strings from a Content message.
 * @param content The Content message to extract from.
 * @return An optional containing the vector if type matches, empty otherwise.
 */
template <>
inline std::optional<std::vector<std::string>>
get_content(const knowledge_graph_msgs::msg::Content &content) {
  if (content.type == knowledge_graph_msgs::msg::Content::VSTRING) {
    return content.string_vector;
  } else {
    return {};
  }
}

/**
 * @brief Adds or updates a property in a property list.
 * @tparam T The type of the property value.
 * @param properties The vector of properties to modify.
 * @param key The property key.
 * @param content The property value.
 * @return True if the property was added/updated, false if the content type is
 * ERROR.
 */
template <class T>
bool add_property(std::vector<knowledge_graph_msgs::msg::Property> &properties,
                  const std::string &key, const T &content) {
  auto newc = new_content<T>(content);

  if (newc.type == knowledge_graph_msgs::msg::Content::ERROR) {
    std::cerr << "Adding a property of type ERROR" << std::endl;
    return false;
  }

  auto it = std::find_if(properties.begin(), properties.end(),
                         [&key](const auto &prop) { return prop.key == key; });

  if (it != properties.end()) {
    it->value = newc;
  } else {
    knowledge_graph_msgs::msg::Property prop;
    prop.key = key;
    prop.value = newc;
    properties.push_back(prop);
  }

  return true;
}

/**
 * @brief Adds or updates a property in a node.
 * @tparam T The type of the property value.
 * @param node The node to modify.
 * @param key The property key.
 * @param content The property value.
 * @return True if the property was added/updated, false if the content type is
 * ERROR.
 */
template <class T>
bool add_property(knowledge_graph_msgs::msg::Node &node, const std::string &key,
                  const T &content) {
  return add_property<T>(node.properties, key, content);
}

/**
 * @brief Adds or updates a property in an edge.
 * @tparam T The type of the property value.
 * @param edge The edge to modify.
 * @param key The property key.
 * @param content The property value.
 * @return True if the property was added/updated, false if the content type is
 * ERROR.
 */
template <class T>
bool add_property(knowledge_graph_msgs::msg::Edge &edge, const std::string &key,
                  const T &content) {
  return add_property<T>(edge.properties, key, content);
}

/**
 * @brief Retrieves a typed property value from a property list.
 * @tparam T The expected type of the property value.
 * @param properties The vector of properties to search.
 * @param key The property key to find.
 * @return An optional containing the value if found and type matches, empty
 * otherwise.
 */
template <class T>
std::optional<T>
get_property(const std::vector<knowledge_graph_msgs::msg::Property> &properties,
             const std::string &key) {
  auto it = std::find_if(properties.begin(), properties.end(),
                         [&key](const auto &prop) { return prop.key == key; });
  if (it != properties.end()) {
    return get_content<T>(it->value);
  }
  return {};
}

/**
 * @brief Retrieves a typed property value from a node.
 * @tparam T The expected type of the property value.
 * @param node The node to search.
 * @param key The property key to find.
 * @return An optional containing the value if found and type matches, empty
 * otherwise.
 */
template <class T>
std::optional<T> get_property(const knowledge_graph_msgs::msg::Node &node,
                              const std::string &key) {
  return get_property<T>(node.properties, key);
}

/**
 * @brief Retrieves a typed property value from an edge.
 * @tparam T The expected type of the property value.
 * @param edge The edge to search.
 * @param key The property key to find.
 * @return An optional containing the value if found and type matches, empty
 * otherwise.
 */
template <class T>
std::optional<T> get_property(const knowledge_graph_msgs::msg::Edge &edge,
                              const std::string &key) {
  return get_property<T>(edge.properties, key);
}

/**
 * @brief Gets the type of a property from a property list.
 * @param properties The vector of properties to search.
 * @param key The property key to find.
 * @return The Content type of the property, or ERROR if not found.
 */
inline uint8_t get_property_type(
    const std::vector<knowledge_graph_msgs::msg::Property> &properties,
    const std::string &key) {
  auto it = std::find_if(properties.begin(), properties.end(),
                         [&key](const auto &prop) { return prop.key == key; });
  if (it != properties.end()) {
    return it->value.type;
  }
  return knowledge_graph_msgs::msg::Content::ERROR;
}

/**
 * @brief Gets the type of a property from a node.
 * @param node The node to search.
 * @param key The property key to find.
 * @return The Content type of the property, or ERROR if not found.
 */
inline uint8_t get_property_type(const knowledge_graph_msgs::msg::Node &node,
                                 const std::string &key) {
  return get_property_type(node.properties, key);
}

/**
 * @brief Gets the type of a property from an edge.
 * @param edge The edge to search.
 * @param key The property key to find.
 * @return The Content type of the property, or ERROR if not found.
 */
inline uint8_t get_property_type(const knowledge_graph_msgs::msg::Edge &edge,
                                 const std::string &key) {
  return get_property_type(edge.properties, key);
}

/**
 * @brief Converts a Content type to its string representation.
 * @param content_type The Content type value.
 * @return String representation of the type (e.g., "bool", "int", "string[]").
 */
inline std::string to_string(uint8_t content_type) {
  switch (content_type) {
  case knowledge_graph_msgs::msg::Content::BOOL:
    return "bool";
  case knowledge_graph_msgs::msg::Content::INT:
    return "int";
  case knowledge_graph_msgs::msg::Content::FLOAT:
    return "float";
  case knowledge_graph_msgs::msg::Content::DOUBLE:
    return "double";
  case knowledge_graph_msgs::msg::Content::STRING:
    return "string";
  case knowledge_graph_msgs::msg::Content::VBOOL:
    return "bool[]";
  case knowledge_graph_msgs::msg::Content::VINT:
    return "int[]";
  case knowledge_graph_msgs::msg::Content::VFLOAT:
    return "float[]";
  case knowledge_graph_msgs::msg::Content::VDOUBLE:
    return "double[]";
  case knowledge_graph_msgs::msg::Content::VSTRING:
    return "string[]";
  case knowledge_graph_msgs::msg::Content::ERROR:
    return "error";
  default:
    return "Unknown";
  }
}

/**
 * @brief Converts a Content message to its string representation.
 * @param content The Content message to convert.
 * @return String representation of the content value.
 */
inline std::string
to_string(const knowledge_graph_msgs::msg::Content &content) {
  switch (content.type) {
  case knowledge_graph_msgs::msg::Content::BOOL:
    return content.bool_value ? "true" : "false";
  case knowledge_graph_msgs::msg::Content::INT:
    return std::to_string(content.int_value);
  case knowledge_graph_msgs::msg::Content::FLOAT:
    return std::to_string(content.float_value);
  case knowledge_graph_msgs::msg::Content::DOUBLE:
    return std::to_string(content.double_value);
  case knowledge_graph_msgs::msg::Content::STRING:
    return content.string_value;
  case knowledge_graph_msgs::msg::Content::VBOOL:
    return detail::vector_to_string(
        content.bool_vector, [](bool v) { return v ? "true" : "false"; });
  case knowledge_graph_msgs::msg::Content::VINT:
    return detail::vector_to_string(content.int_vector,
                                    [](int v) { return std::to_string(v); });
  case knowledge_graph_msgs::msg::Content::VFLOAT:
    return detail::vector_to_string(content.float_vector,
                                    [](float v) { return std::to_string(v); });
  case knowledge_graph_msgs::msg::Content::VDOUBLE:
    return detail::vector_to_string(content.double_vector,
                                    [](double v) { return std::to_string(v); });
  case knowledge_graph_msgs::msg::Content::VSTRING:
    return detail::vector_to_string(content.string_vector,
                                    [](const std::string &v) { return v; });
  default:
    return "error";
  }
}

/**
 * @brief Converts a type string to its Content type value.
 * @param type The string representation of the type.
 * @return The Content type value, or ERROR if not recognized.
 */
inline uint8_t type_from_string(const std::string &type) {
  for (uint8_t i = 0; i < knowledge_graph_msgs::msg::Content::NUM_TYPES; i++) {
    if (type == to_string(i)) {
      return i;
    }
  }
  return knowledge_graph_msgs::msg::Content::ERROR;
}

/**
 * @brief Converts a Node message to its string representation.
 * @param node The Node message to convert.
 * @return String representation of the node with its properties.
 */
inline std::string to_string(const knowledge_graph_msgs::msg::Node &node) {
  std::string ret;
  ret = ret + node.node_name + " (" + node.node_class + ")";

  for (const auto &prop : node.properties) {
    ret = ret + "\n\t" + prop.key + ": [" + to_string(prop.value) + "]";
  }

  return ret;
}

/**
 * @brief Converts an Edge message to its string representation.
 * @param edge The Edge message to convert.
 * @return String representation of the edge.
 */
inline std::string to_string(const knowledge_graph_msgs::msg::Edge &edge) {
  std::string ret;
  ret = ret + " [" + edge.edge_class + "]" + edge.source_node + " -> " +
        edge.target_node;
  return ret;
}

} // namespace knowledge_graph

#endif
