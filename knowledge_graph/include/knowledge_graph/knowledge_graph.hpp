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

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "knowledge_graph/graph/graph.hpp"

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace knowledge_graph {

/**
 * @brief A distributed knowledge graph implementation for ROS 2.
 *
 * This class provides a synchronized knowledge graph that can be shared
 * across multiple ROS 2 nodes. It supports adding, updating, and removing
 * nodes and edges, with automatic synchronization between instances.
 *
 */
class KnowledgeGraph : public graph::Graph {
public:
  /**
   * @brief Provides access to the singleton instance of KnowledgeGraph.
   *
   * This method ensures there is only one instance of KnowledgeGraph running.
   *
   * @return A shared pointer to the singleton instance of KnowledgeGraph.
   */
  static std::shared_ptr<KnowledgeGraph> get_instance() {
    // Ensure ROS 2 is initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create the singleton instance if it doesn't exist
    static std::shared_ptr<KnowledgeGraph> instance =
        std::shared_ptr<KnowledgeGraph>(new KnowledgeGraph());
    return instance;
  }

  /**
   * @brief Deleted copy constructor to prevent copying of the singleton
   * instance.
   *
   * @param other Another instance of KnowledgeGraph (unused).
   */
  KnowledgeGraph(KnowledgeGraph &other) = delete;

  /**
   * @brief Default destructor.
   */
  ~KnowledgeGraph() = default;

  /************************************************************
   * Graph Management Functions
   ************************************************************/
  /**
   * @brief Update the graph with another graph
   * @param graph The graph to update from
   */
  void update_graph(const graph::Graph &graph) override;

  /**
   * @brief Convert the graph to a Graph message
   * @return The Graph message representation of the graph
   */
  knowledge_graph_msgs::msg::Graph to_msg() const;

  /************************************************************
   * Node Management Functions
   ************************************************************/
  /**
   * @brief Create a new node in the graph
   * @param name The name of the node
   * @param type The type of the node
   * @return The created Node
   */
  graph::Node create_node(const std::string &name,
                          const std::string &type) override;

  /**
   * @brief Update a node in the graph
   * @param node The node to update
   */
  void update_node(const graph::Node &node) override;

  /**
   * @brief Update nodes in the graph
   * @param nodes The nodes to update
   */
  void update_nodes(const std::vector<graph::Node> &nodes) override;

  /**
   * @brief Remove a node from the graph
   * @param node The node to remove
   * @return True if the node was removed, false if it was not found
   */
  bool remove_node(const graph::Node &node) override;

  /**
   * @brief Remove nodes from the graph
   * @param nodes The nodes to remove
   */
  const std::vector<graph::Node>
  remove_nodes(const std::vector<graph::Node> &nodes) override;

  /***
   * @brief Check if a node exists in the graph
   * @param name The name of the node
   * @return True if the node exists, false otherwise
   */
  bool has_node(const std::string &name) const;

  /**
   * @brief Get the number of nodes in the graph
   * @return Number of nodes in the graph
   */
  int get_num_nodes() const;

  /**
   * @brief Get all nodes in the graph
   * @return Vector of all nodes in the graph
   */
  std::vector<graph::Node> get_nodes() const;

  /**
   * @brief Get a node by name
   * @param name The name of the node
   * @return The Node with the given name
   * @throws std::runtime_error if the node does not exist
   */
  graph::Node get_node(const std::string &name) const;

  /************************************************************
   * Edge Management Functions
   ************************************************************/
  /**
   * @brief Create a new edge in the graph
   * @param type The type of the edge
   * @param source_node The source node name
   * @param target_node The target node name
   * @return The created Edge
   */
  graph::Edge create_edge(const std::string &type,
                          const std::string &source_node,
                          const std::string &target_node) override;

  /**
   * @brief Check if an edge exists in the graph
   * @param type The type of the edge
   * @param source_node The source node name
   * @param target_node The target node name
   * @return True if the edge exists, false otherwise
   */
  bool has_edge(const std::string &type, const std::string &source_node,
                const std::string &target_node) const;

  /**
   * @brief Update an edge in the graph
   * @param edge The edge to update
   */
  void update_edge(const graph::Edge &edge) override;

  /**
   * @brief Update edges in the graph
   * @param edges The edges to update
   */
  void update_edges(const std::vector<graph::Edge> &edges) override;

  /**
   * @brief Remove an edge from the graph
   * @param edge The edge to remove
   * @return True if the edge was removed, false if it was not found
   */
  bool remove_edge(const graph::Edge &edge) override;

  /**
   * @brief Remove edges from the graph
   * @param edges The edges to remove
   */
  const std::vector<graph::Edge>
  remove_edges(const std::vector<graph::Edge> &edges) override;

  /**
   * @brief Get the number of edges in the graph
   * @return Number of edges in the graph
   */
  int get_num_edges() const;

  /**
   * @brief Get all edges in the graph
   * @return Vector of all edges in the graph
   */
  std::vector<graph::Edge> get_edges() const;

  /**
   * @brief Get edges from a specific source node
   * @param source_node The source node name
   * @return Vector of edges from the specified source node
   */
  std::vector<graph::Edge>
  get_edges_from_node(const std::string &source_node) const;

  /**
   * @brief Get edges to a specific target node
   * @param target_node The target node name
   * @return Vector of edges to the specified target node
   */
  std::vector<graph::Edge>
  get_edges_to_node(const std::string &target_node) const;

  /**
   * @brief Get edges between a specific source and target node
   * @param source_node The source node name
   * @param target_node The target node name
   * @return Vector of edges between the specified source and target nodes
   */
  std::vector<graph::Edge>
  get_edges_between_nodes(const std::string &source_node,
                          const std::string &target_node) const;

  /**
   * @brief Get edges of a specific type
   * @param type The edge type
   * @return Vector of edges of the specified type
   */
  std::vector<graph::Edge> get_edges_by_type(const std::string &type) const;

  /**
   * @brief Get edges from a specific source node of a specific type
   * @param type The edge type
   * @param source_node The source node name
   * @return Vector of edges from the specified source node of the specified
   * type
   */
  std::vector<graph::Edge>
  get_edges_from_node_by_type(const std::string &type,
                              const std::string &source_node) const;

  /**
   * @brief Get edges to a specific target node of a specific type
   * @param type The edge type
   * @param target_node The target node name
   * @return Vector of edges to the specified target node of the specified type
   */
  std::vector<graph::Edge>
  get_edges_to_node_by_type(const std::string &type,
                            const std::string &target_node) const;

  /**
   * @brief Get an edge by type, source node, and target node
   * @param type The edge type
   * @param source_node The source node name
   * @param target_node The target node name
   * @return The Edge with the specified type, source node, and target node
   * @throws std::runtime_error if the edge does not exist
   */
  graph::Edge get_edge(const std::string &type, const std::string &source_node,
                       const std::string &target_node) const;

protected:
  /// @brief Pointer to the ROS 2 node that owns this graph.
  rclcpp::Node::SharedPtr node;

  /// @brief Unique identifier for this graph instance.
  std::string graph_id;

  /// @brief Timestamp of the last graph modification.
  rclcpp::Time last_ts;

  /**
   * @brief Constructor initializing the KnowledgeGraph.
   */
  explicit KnowledgeGraph();

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
   */
  void
  publish_update(uint8_t operation, uint8_t element,
                 const std::vector<knowledge_graph_msgs::msg::Node> &nodes,
                 const std::vector<knowledge_graph_msgs::msg::Edge> &edges);

private:
  /// Executor for managing multiple threads.
#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 1, 1) // Jazzy, Kilted and Rolling
  rclcpp::experimental::executors::EventsExecutor executor;
#else // Humble, Iron and Jazzy
  rclcpp::executors::MultiThreadedExecutor executor;
#endif
#else // Foxy and Galactic
  rclcpp::executors::MultiThreadedExecutor executor;
#endif
  /// Thread for spinning the node.
  std::unique_ptr<std::thread> spin_thread;

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

  /// @brief Mutex for thread-safe graph operations.
  mutable std::recursive_mutex graph_mutex_;
};

} // namespace knowledge_graph

#endif
