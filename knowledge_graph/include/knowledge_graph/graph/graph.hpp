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

#ifndef KNOWLEDGE__GRAPH__GRAPH_HPP_
#define KNOWLEDGE__GRAPH__GRAPH_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"

namespace knowledge_graph {
namespace graph {

/**
 * @class Graph
 * @brief Class representing a knowledge graph with nodes and edges.
 */
class Graph {
public:
  /**
   * @brief Default constructor
   */
  Graph() = default;

  /**
   * @brief Constructor from a Graph message
   * @param msg The Graph message to initialize the graph from
   */
  Graph(const knowledge_graph_msgs::msg::Graph &msg);

  /**
   * @brief Destructor
   */
  ~Graph() = default;

  /**
   * @brief Update the graph with another graph
   * @param graph The graph to update from
   */
  virtual void update_graph(const Graph &graph);

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
  virtual Node create_node(const std::string &name, const std::string &type);

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
  std::vector<Node> get_nodes() const;

  /**
   * @brief Get a node by name
   * @param name The name of the node
   * @return The Node with the given name
   * @throws std::runtime_error if the node does not exist
   */
  Node get_node(const std::string &name) const;

  /**
   * @brief Update a node in the graph
   * @param node The node to update
   */
  virtual void update_node(const Node &node);

  /**
   * @brief Update nodes in the graph
   * @param nodes The nodes to update
   */
  virtual void update_nodes(const std::vector<Node> &nodes);

  /**
   * @brief Remove a node from the graph
   * @param node The node to remove
   * @return True if the node was removed, false if it was not found
   */
  virtual bool remove_node(const Node &node);

  /**
   * @brief Remove nodes from the graph
   * @param nodes The nodes to remove
   */
  virtual void remove_nodes(const std::vector<Node> &nodes);

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
  virtual Edge create_edge(const std::string &type,
                           const std::string &source_node,
                           const std::string &target_node);

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
   * @brief Get the number of edges in the graph
   * @return Number of edges in the graph
   */
  int get_num_edges() const;

  /**
   * @brief Get all edges in the graph
   * @return Vector of all edges in the graph
   */
  std::vector<Edge> get_edges() const;

  /**
   * @brief Get edges from a specific source node
   * @param source_node The source node name
   * @return Vector of edges from the specified source node
   */
  std::vector<Edge> get_edges_from_node(const std::string &source_node) const;

  /**
   * @brief Get edges to a specific target node
   * @param target_node The target node name
   * @return Vector of edges to the specified target node
   */
  std::vector<Edge> get_edges_to_node(const std::string &target_node) const;

  /**
   * @brief Get edges between a specific source and target node
   * @param source_node The source node name
   * @param target_node The target node name
   * @return Vector of edges between the specified source and target nodes
   */
  std::vector<Edge>
  get_edges_between_nodes(const std::string &source_node,
                          const std::string &target_node) const;

  /**
   * @brief Get edges of a specific type
   * @param type The edge type
   * @return Vector of edges of the specified type
   */
  std::vector<Edge> get_edges_by_type(const std::string &type) const;

  /**
   * @brief Get edges from a specific source node of a specific type
   * @param type The edge type
   * @param source_node The source node name
   * @return Vector of edges from the specified source node of the specified
   * type
   */
  std::vector<Edge>
  get_edges_from_node_by_type(const std::string &type,
                              const std::string &source_node) const;

  /**
   * @brief Get edges to a specific target node of a specific type
   * @param type The edge type
   * @param target_node The target node name
   * @return Vector of edges to the specified target node of the specified type
   */
  std::vector<Edge>
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
  Edge get_edge(const std::string &type, const std::string &source_node,
                const std::string &target_node) const;

  /**
   * @brief Update an edge in the graph
   * @param edge The edge to update
   */
  virtual void update_edge(const Edge &edge);

  /**
   * @brief Update edges in the graph
   * @param edges The edges to update
   */
  virtual void update_edges(const std::vector<Edge> &edges);

  /**
   * @brief Remove an edge from the graph
   * @param edge The edge to remove
   * @return True if the edge was removed, false if it was not found
   */
  virtual bool remove_edge(const Edge &edge);

  /**
   * @brief Remove edges from the graph
   * @param edges The edges to remove
   */
  virtual void remove_edges(const std::vector<Edge> &edges);

protected:
  /**
   * @brief Filter edges based on a predicate
   * @tparam Predicate The type of the predicate function
   * @param pred The predicate function to filter edges
   * @return Vector of edges that satisfy the predicate
   */
  template <typename Predicate>
  std::vector<Edge> filter_edges(Predicate pred) const;

  /**
   * @brief Remove edges based on a predicate
   * @tparam Predicate The type of the predicate function
   * @param pred The predicate function to identify edges to remove
   * @return True if any edges were removed, false otherwise
   */
  template <typename Predicate> bool remove_edges_if(Predicate pred);

private:
  /// @brief Vector of nodes in the graph
  std::vector<Node> nodes_;
  /// @brief Vector of edges in the graph
  std::vector<Edge> edges_;
};

} // namespace graph
} // namespace knowledge_graph

#endif // KNOWLEDGE__GRAPH__GRAPH_HPP_