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

#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace knowledge_graph {

/// @brief Type alias for edge filter predicates.
using EdgePredicate =
    std::function<bool(const knowledge_graph_msgs::msg::Edge &)>;

/**
 * @brief A distributed knowledge graph implementation for ROS 2.
 *
 * This class provides a synchronized knowledge graph that can be shared
 * across multiple ROS 2 nodes. It supports adding, updating, and removing
 * nodes and edges, with automatic synchronization between instances.
 *
 */
class KnowledgeGraph {
public:
  /**
   * @brief Constructs a KnowledgeGraph with a raw pointer to a ROS 2 node.
   * @param provided_node Pointer to the ROS 2 node that owns this graph.
   */
  explicit KnowledgeGraph(rclcpp::Node *provided_node);

  /**
   * @brief Constructs a KnowledgeGraph with a shared pointer to a ROS 2 node.
   * @param provided_node Shared pointer to the ROS 2 node that owns this graph.
   */
  explicit KnowledgeGraph(rclcpp::Node::SharedPtr provided_node)
      : KnowledgeGraph(provided_node.get()) {}

  /**
   * @brief Constructs a KnowledgeGraph with a lifecycle node.
   * @param provided_node Shared pointer to the lifecycle node that owns this
   * graph.
   */
  explicit KnowledgeGraph(
      rclcpp_lifecycle::LifecycleNode::SharedPtr provided_node)
      : KnowledgeGraph((rclcpp::Node *)provided_node.get()) {}

  /**
   * @brief Default destructor.
   */
  ~KnowledgeGraph() = default;

  /**
   * @brief Removes a node from the graph.
   * @param node The name of the node to remove.
   * @param sync If true, synchronizes the removal with other graph instances.
   * @return True if the node was successfully removed, false otherwise.
   * @note This also removes all edges connected to the node.
   */
  bool remove_node(const std::string &node, bool sync = true);

  /**
   * @brief Checks if a node exists in the graph.
   * @param node The name of the node to check.
   * @return True if the node exists, false otherwise.
   */
  bool exist_node(const std::string &node);

  /**
   * @brief Retrieves a node from the graph.
   * @param node The name of the node to retrieve.
   * @return An optional containing the node if found, empty otherwise.
   */
  std::optional<knowledge_graph_msgs::msg::Node>
  get_node(const std::string &node);

  /**
   * @brief Removes an edge from the graph.
   * @param edge The edge to remove.
   * @param sync If true, synchronizes the removal with other graph instances.
   * @return True if the edge was successfully removed, false otherwise.
   */
  bool remove_edge(const knowledge_graph_msgs::msg::Edge &edge,
                   bool sync = true);

  /**
   * @brief Gets all edges between two nodes.
   * @param source The source node name.
   * @param target The target node name.
   * @return Vector of edges connecting the source and target nodes.
   */
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_edges(const std::string &source, const std::string &target);

  /**
   * @brief Gets all edges of a specific class.
   * @param edge_class The class/type of edges to retrieve.
   * @return Vector of edges matching the specified class.
   */
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_edges(const std::string &edge_class);

  /**
   * @brief Gets all outgoing edges from a node.
   * @param source The source node name.
   * @return Vector of edges originating from the source node.
   */
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_out_edges(const std::string &source);

  /**
   * @brief Gets all incoming edges to a node.
   * @param target The target node name.
   * @return Vector of edges pointing to the target node.
   */
  std::vector<knowledge_graph_msgs::msg::Edge>
  get_in_edges(const std::string &target);

  /**
   * @brief Gets all nodes in the graph.
   * @return Vector copy of all nodes (thread-safe).
   */
  std::vector<knowledge_graph_msgs::msg::Node> get_nodes() const;

  /**
   * @brief Gets all edges in the graph.
   * @return Vector copy of all edges (thread-safe).
   */
  std::vector<knowledge_graph_msgs::msg::Edge> get_edges() const;

  /**
   * @brief Gets the names of all nodes in the graph.
   * @return Vector of node names.
   */
  const std::vector<std::string> get_node_names();

  /**
   * @brief Gets the number of edges in the graph.
   * @return The number of edges.
   */
  size_t get_num_edges() const;

  /**
   * @brief Gets the number of nodes in the graph.
   * @return The number of nodes.
   */
  size_t get_num_nodes() const;

  /**
   * @brief Updates or adds a node in the graph.
   * @param node The node to update or add.
   * @param sync If true, synchronizes the update with other graph instances.
   * @return True if the operation was successful.
   */
  bool update_node(const knowledge_graph_msgs::msg::Node &node,
                   bool sync = true);

  /**
   * @brief Updates or adds an edge in the graph.
   * @param edge The edge to update or add.
   * @param sync If true, synchronizes the update with other graph instances.
   * @return True if the operation was successful, false if source or target
   * nodes don't exist.
   */
  bool update_edge(const knowledge_graph_msgs::msg::Edge &edge,
                   bool sync = true);

protected:
  /// @brief Pointer to the ROS 2 node that owns this graph.
  rclcpp::Node *provided_node;

  /// @brief The internal graph data structure.
  knowledge_graph_msgs::msg::Graph::UniquePtr graph;

  /// @brief Unique identifier for this graph instance.
  std::string graph_id;

  /// @brief Timestamp of the last graph modification.
  rclcpp::Time last_ts;

  /// @brief Shared mutex for thread-safe graph read/write operations.
  mutable std::shared_mutex graph_mutex;

  /**
   * @brief Updates the local graph with data from another graph.
   * @param msg The graph message containing updates.
   */
  void update_graph(knowledge_graph_msgs::msg::Graph msg);

  /**
   * @brief Callback for handling incoming graph update messages.
   * @param msg The graph update message.
   */
  void update_callback(knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg);

  /**
   * @brief Timer callback for requesting graph synchronization.
   */
  void reqsync_timer_callback();

  /**
   * @brief Publishes a graph update message.
   * @param operation The operation type (UPDATE, REMOVE).
   * @param element The element type (NODE, EDGE).
   * @param node Optional node data for node operations.
   * @param edge Optional edge data for edge operations.
   * @param removed_node Optional name of removed node.
   */
  void publish_update(
      uint8_t operation, uint8_t element,
      const std::optional<knowledge_graph_msgs::msg::Node> &node = std::nullopt,
      const std::optional<knowledge_graph_msgs::msg::Edge> &edge = std::nullopt,
      const std::string &removed_node = "");

  /**
   * @brief Filters edges based on a predicate.
   * @param predicate Function to test each edge.
   * @return Vector of edges matching the predicate.
   */
  std::vector<knowledge_graph_msgs::msg::Edge>
  filter_edges(const EdgePredicate &predicate) const;

private:
  /// @brief Publisher for graph update messages.
  rclcpp::Publisher<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_pub;

  /// @brief Subscriber for graph update messages.
  rclcpp::Subscription<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_sub;

  /// @brief Timer for periodic synchronization requests.
  rclcpp::TimerBase::SharedPtr reqsync_timer;

  /// @brief Start time for synchronization timeout.
  rclcpp::Time start_time;
};

} // namespace knowledge_graph

#endif
