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

#ifndef KNOWLEDGE__GRAPH__NODE_HPP_
#define KNOWLEDGE__GRAPH__NODE_HPP_

#include <string>

#include "knowledge_graph/graph/properties_container.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

namespace knowledge_graph {
namespace graph {

/**
 * @brief Class representing a node in the knowledge graph.
 */
class Node : public PropertiesContainer {
public:
  /**
   * @brief Constructor initializing a Node with a name and type.
   * @param name The name of the node.
   * @param type The type of the node.
   */
  Node(const std::string &name, const std::string &type);

  /**
   * @brief Constructor initializing a Node from a Node message.
   * @param msg The Node message to initialize from.
   */
  Node(const knowledge_graph_msgs::msg::Node &msg);

  /**
   * @brief Destructor.
   */
  ~Node() = default;

  /**
   * @brief Get the name of the node.
   * @return The name of the node.
   */
  std::string get_name() const;

  /**
   * @brief Get the type of the node.
   * @return The type of the node.
   */
  std::string get_type() const;

  /**
   * @brief Convert the Node to a Node message.
   * @return The Node message.
   */
  knowledge_graph_msgs::msg::Node to_msg() const;

  /**
   * @brief Get a string representation of the Node.
   * @return String representation of the Node.
   */
  std::string to_string() const;

private:
  /// @brief Name of the node.
  std::string name_;
  /// @brief Type of the node.
  std::string type_;
};

} // namespace graph
} // namespace knowledge_graph

#endif // KNOWLEDGE__GRAPH__NODE_HPP_