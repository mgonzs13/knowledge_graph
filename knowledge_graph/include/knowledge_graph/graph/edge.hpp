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

#ifndef KNOWLEDGE__GRAPH__EDGE_HPP_
#define KNOWLEDGE__GRAPH__EDGE_HPP_

#include <string>

#include "knowledge_graph/graph/properties_container.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"

namespace knowledge_graph {
namespace graph {

/**
 * @class Edge
 * @brief Class representing an edge in the knowledge graph.
 */
class Edge : public PropertiesContainer {
public:
  /**
   * @brief Construct an Edge with type, source node, and target node
   * @param type Edge type
   * @param source_node Source node ID
   * @param target_node Target node ID
   */
  Edge(const std::string &type, const std::string &source_node,
       const std::string &target_node);

  /**
   * @brief Construct an Edge from a ROS message
   * @param msg ROS message representing the edge
   */
  Edge(const knowledge_graph_msgs::msg::Edge &msg);

  /**
   * @brief Destructor
   */
  ~Edge() = default;

  /**
   * @brief Get edge type
   * @return Edge type
   */
  std::string get_type() const;

  /**
   * @brief Get source node ID
   * @return Source node ID
   */
  std::string get_source_node() const;

  /**
   * @brief Get target node ID
   * @return Target node ID
   */
  std::string get_target_node() const;

  /**
   * @brief Convert edge to ROS message
   * @return ROS message representation of the edge
   */
  knowledge_graph_msgs::msg::Edge to_msg() const;

  /**
   * @brief Convert edge to string representation
   * @return String representation of the edge
   */
  std::string to_string() const;

private:
  /// @brief Edge type
  std::string type_;
  /// @brief  Source node ID
  std::string source_node_;
  /// @brief  Target node ID
  std::string target_node_;
};

} // namespace graph
} // namespace knowledge_graph

#endif // KNOWLEDGE__GRAPH__EDGE_HPP_