# Copyright 2023 Miguel Ángel González Santamarta

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
@file knowledge_graph.py
@brief Knowledge Graph implementation for ROS 2.

This module provides a singleton KnowledgeGraph class that manages
a distributed graph of nodes and edges with synchronization capabilities.
"""

from typing import List, Optional, Union
from threading import Lock, RLock

from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from knowledge_graph_msgs.msg import Node as NodeMsg
from knowledge_graph_msgs.msg import Edge as EdgeMsg
from knowledge_graph_msgs.msg import Graph as GraphMsg
from knowledge_graph_msgs.msg import GraphUpdate


class KnowledgeGraph:
    """
    @brief Singleton class for managing a distributed knowledge graph.

    The KnowledgeGraph class provides functionality for managing nodes and edges
    in a graph structure with support for distributed synchronization across
    multiple ROS 2 nodes.
    """

    _instance: "KnowledgeGraph" = None
    _lock: Lock = Lock()

    # QoS profile for graph updates
    _UPDATE_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    @staticmethod
    def get_instance(node: Node) -> "KnowledgeGraph":
        """
        @brief Gets the singleton instance of KnowledgeGraph.

        @param node The ROS 2 node to use for communication.
        @return The singleton KnowledgeGraph instance.
        """
        with KnowledgeGraph._lock:
            if KnowledgeGraph._instance is None:
                KnowledgeGraph._instance = KnowledgeGraph(node)
            return KnowledgeGraph._instance

    def __init__(self, provided_node: Node) -> None:
        """
        @brief Initializes the KnowledgeGraph.

        @param provided_node The ROS 2 node to use for communication.
        @raise Exception If an instance already exists (Singleton pattern).
        """
        if KnowledgeGraph._instance is not None:
            raise Exception("This class is a Singleton")

        self.node = provided_node
        self.graph = GraphMsg()
        self.graph_id = self.node.get_name()
        self.last_ts = self.node.get_clock().now()
        self._start_time = self.node.get_clock().now()

        # Mutex for thread-safe graph operations (reentrant to allow nested calls)
        self._graph_mutex = RLock()

        # Create publisher and subscriber for graph updates
        self._update_pub = self.node.create_publisher(
            GraphUpdate, "graph_update", self._UPDATE_QOS
        )
        self._update_sub = self.node.create_subscription(
            GraphUpdate, "graph_update", self._update_callback, self._UPDATE_QOS
        )

        # Start sync timer
        self._reqsync_timer = self.node.create_timer(0.1, self._reqsync_timer_callback)
        self._reqsync_timer_callback()

    # =========================================================================
    # Private Methods
    # =========================================================================

    def _reqsync_timer_callback(self) -> None:
        """@brief Timer callback for requesting graph synchronization."""
        elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds
        if Time(nanoseconds=elapsed).seconds_nanoseconds()[0] > 1.0:
            self._reqsync_timer.cancel()

        with self._graph_mutex:
            self._publish_update(
                GraphUpdate.REQSYNC, self.graph, element_type=GraphUpdate.GRAPH
            )

    def _publish_update(
        self,
        operation_type: int,
        element: Union[NodeMsg, EdgeMsg, GraphMsg],
        target_node: str = "",
        element_type: Optional[int] = None,
    ) -> None:
        """
        @brief Publishes a graph update message.

        @param operation_type The type of operation (UPDATE, REMOVE, etc.).
        @param element The graph element being updated.
        @param target_node Optional target node for directed updates.
        @param element_type Optional explicit element type.
        """
        update_msg = GraphUpdate()
        update_msg.stamp = self.node.get_clock().now().to_msg()
        update_msg.node_id = self.graph_id
        update_msg.target_node = target_node
        update_msg.operation_type = operation_type

        if element_type is not None:
            update_msg.element_type = element_type
        elif isinstance(element, NodeMsg):
            update_msg.element_type = GraphUpdate.NODE
        elif isinstance(element, EdgeMsg):
            update_msg.element_type = GraphUpdate.EDGE
        elif isinstance(element, GraphMsg):
            update_msg.element_type = GraphUpdate.GRAPH

        # Set the appropriate field based on element type
        if isinstance(element, NodeMsg):
            if operation_type == GraphUpdate.REMOVE:
                update_msg.removed_node = element.node_name
            else:
                update_msg.node = element
        elif isinstance(element, EdgeMsg):
            update_msg.edge = element
        elif isinstance(element, GraphMsg):
            update_msg.graph = element

        self._update_pub.publish(update_msg)

    def _find_node_index(self, node_name: str) -> int:
        """
        @brief Finds the index of a node by name.

        @param node_name The name of the node to find.
        @return The index of the node, or -1 if not found.
        """
        for idx, n in enumerate(self.graph.nodes):
            if n.node_name == node_name:
                return idx
        return -1

    def _find_edge_index(self, edge: EdgeMsg) -> int:
        """
        @brief Finds the index of an edge.

        @param edge The edge to find.
        @return The index of the edge, or -1 if not found.
        """
        for idx, e in enumerate(self.graph.edges):
            if (
                e.source_node == edge.source_node
                and e.target_node == edge.target_node
                and e.edge_class == edge.edge_class
            ):
                return idx
        return -1

    def _update_timestamp(self) -> None:
        """@brief Updates the last modification timestamp."""
        self.last_ts = self.node.get_clock().now()

    def _update_callback(self, msg: GraphUpdate) -> None:
        """
        @brief Callback for processing incoming graph updates.

        @param msg The received GraphUpdate message.
        """
        author_id = msg.node_id
        element = msg.element_type
        operation = msg.operation_type
        ts = Time.from_msg(msg.stamp)

        # Ignore our own updates
        if author_id == self.graph_id:
            return

        with self._graph_mutex:
            # Check for out-of-order updates
            if ts < self.last_ts and operation not in (
                GraphUpdate.REQSYNC,
                GraphUpdate.SYNC,
            ):
                self.node.get_logger().error(
                    f"UNORDERED UPDATE [{operation}] "
                    f"{self.last_ts.seconds_nanoseconds()[0]} > {ts.seconds_nanoseconds()[0]}"
                )

            # Process update based on element type
            if element == GraphUpdate.NODE:
                if operation == GraphUpdate.UPDATE:
                    self._update_node_internal(msg.node, sync=False)
                elif operation == GraphUpdate.REMOVE:
                    self._remove_node_internal(msg.removed_node, sync=False)

            elif element == GraphUpdate.EDGE:
                if operation == GraphUpdate.UPDATE:
                    self._update_edge_internal(msg.edge, sync=False)
                elif operation == GraphUpdate.REMOVE:
                    self._remove_edge_internal(msg.edge, sync=False)

            elif element == GraphUpdate.GRAPH:
                if operation == GraphUpdate.SYNC and msg.target_node == self.graph_id:
                    self._reqsync_timer.cancel()
                    self._update_graph(msg.graph)
                elif operation == GraphUpdate.REQSYNC and msg.node_id != self.graph_id:
                    self._publish_update(
                        GraphUpdate.SYNC,
                        self.graph,
                        target_node=msg.node_id,
                        element_type=GraphUpdate.GRAPH,
                    )
                    self._update_graph(msg.graph)

    def _update_graph(self, msg: GraphMsg) -> None:
        """
        @brief Updates the graph with data from another graph message.

        @param msg The GraphMsg containing nodes and edges to merge.
        """
        with self._graph_mutex:
            for n in msg.nodes:
                self._update_node_internal(n, sync=False)

            for e in msg.edges:
                self._update_edge_internal(e, sync=False)

            self._update_timestamp()

    # =========================================================================
    # Internal Methods (no lock, assumes caller holds the mutex)
    # =========================================================================

    def _update_node_internal(self, node: NodeMsg, sync: bool = True) -> bool:
        """
        @brief Internal method to add or update a node (assumes lock is held).

        @param node The node message to add or update.
        @param sync Whether to synchronize this update with other graphs.
        @return True if the operation was successful.
        """
        idx = self._find_node_index(node.node_name)

        if idx == -1:
            self.graph.nodes.append(node)
        else:
            self.graph.nodes[idx] = node

        if sync:
            self._publish_update(GraphUpdate.UPDATE, node)

        self._update_timestamp()
        return True

    def _remove_node_internal(self, node_name: str, sync: bool = True) -> bool:
        """
        @brief Internal method to remove a node (assumes lock is held).

        @param node_name The name of the node to remove.
        @param sync Whether to synchronize this update with other graphs.
        @return True if the node was removed, False if it didn't exist.
        """
        idx = self._find_node_index(node_name)
        if idx == -1:
            return False

        node = self.graph.nodes[idx]

        # Remove the node
        del self.graph.nodes[idx]

        # Remove all edges connected to this node
        self.graph.edges = [
            e
            for e in self.graph.edges
            if e.source_node != node_name and e.target_node != node_name
        ]

        if sync:
            self._publish_update(GraphUpdate.REMOVE, node)

        self._update_timestamp()
        return True

    def _update_edge_internal(self, edge: EdgeMsg, sync: bool = True) -> bool:
        """
        @brief Internal method to add or update an edge (assumes lock is held).

        @param edge The edge message to add or update.
        @param sync Whether to synchronize this update with other graphs.
        @return True if successful, False if source or target node doesn't exist.
        """
        if self._find_node_index(edge.source_node) == -1:
            self.node.get_logger().error(
                f"Node source [{edge.source_node}] doesn't exist adding edge"
            )
            return False

        if self._find_node_index(edge.target_node) == -1:
            self.node.get_logger().error(
                f"Node target [{edge.target_node}] doesn't exist adding edge"
            )
            return False

        idx = self._find_edge_index(edge)

        if idx == -1:
            self.graph.edges.append(edge)
        else:
            self.graph.edges[idx] = edge

        if sync:
            self._publish_update(GraphUpdate.UPDATE, edge)

        self._update_timestamp()
        return True

    def _remove_edge_internal(self, edge: EdgeMsg, sync: bool = True) -> bool:
        """
        @brief Internal method to remove an edge (assumes lock is held).

        @param edge The edge to remove (matched by source, target, and class).
        @param sync Whether to synchronize this update with other graphs.
        @return True if the edge was removed, False if it didn't exist.
        """
        idx = self._find_edge_index(edge)

        if idx == -1:
            return False

        del self.graph.edges[idx]

        if sync:
            self._publish_update(GraphUpdate.REMOVE, edge)

        self._update_timestamp()
        return True

    # =========================================================================
    # Node Operations
    # =========================================================================

    def update_node(self, node: NodeMsg, sync: bool = True) -> bool:
        """
        @brief Adds or updates a node in the graph.

        @param node The node message to add or update.
        @param sync Whether to synchronize this update with other graphs.
        @return True if the operation was successful.
        """
        with self._graph_mutex:
            return self._update_node_internal(node, sync)

    def remove_node(self, node_name: str, sync: bool = True) -> bool:
        """
        @brief Removes a node and all its connected edges from the graph.

        @param node_name The name of the node to remove.
        @param sync Whether to synchronize this update with other graphs.
        @return True if the node was removed, False if it didn't exist.
        """
        with self._graph_mutex:
            return self._remove_node_internal(node_name, sync)

    def exist_node(self, node_name: str) -> bool:
        """
        @brief Checks if a node exists in the graph.

        @param node_name The name of the node to check.
        @return True if the node exists, False otherwise.
        """
        with self._graph_mutex:
            return self._find_node_index(node_name) != -1

    def get_node(self, node_name: str) -> Optional[NodeMsg]:
        """
        @brief Retrieves a node by name.

        @param node_name The name of the node to retrieve.
        @return The node message if found, None otherwise.
        """
        with self._graph_mutex:
            idx = self._find_node_index(node_name)
            return self.graph.nodes[idx] if idx != -1 else None

    def get_node_names(self) -> List[str]:
        """
        @brief Gets the names of all nodes in the graph.

        @return A list of node names.
        """
        with self._graph_mutex:
            return [n.node_name for n in self.graph.nodes]

    def get_nodes(self) -> List[NodeMsg]:
        """
        @brief Gets all nodes in the graph.

        @return A list of all node messages.
        """
        with self._graph_mutex:
            return list(self.graph.nodes)

    def get_num_nodes(self) -> int:
        """
        @brief Gets the number of nodes in the graph.

        @return The number of nodes.
        """
        with self._graph_mutex:
            return len(self.graph.nodes)

    # =========================================================================
    # Edge Operations
    # =========================================================================

    def update_edge(self, edge: EdgeMsg, sync: bool = True) -> bool:
        """
        @brief Adds or updates an edge in the graph.

        @param edge The edge message to add or update.
        @param sync Whether to synchronize this update with other graphs.
        @return True if successful, False if source or target node doesn't exist.
        """
        with self._graph_mutex:
            return self._update_edge_internal(edge, sync)

    def remove_edge(self, edge: EdgeMsg, sync: bool = True) -> bool:
        """
        @brief Removes an edge from the graph.

        @param edge The edge to remove (matched by source, target, and class).
        @param sync Whether to synchronize this update with other graphs.
        @return True if the edge was removed, False if it didn't exist.
        """
        with self._graph_mutex:
            return self._remove_edge_internal(edge, sync)

    def get_edges(
        self,
        source: Optional[str] = None,
        target: Optional[str] = None,
        edge_class: Optional[str] = None,
    ) -> List[EdgeMsg]:
        """
        @brief Gets edges matching the specified criteria.

        @param source The source node name (optional).
        @param target The target node name (optional).
        @param edge_class The edge class to filter by (optional).
        @return A list of matching edges.

        If no parameters are provided, returns all edges.
        If edge_class is provided, filters by class.
        If source and target are provided, filters by endpoints.
        """
        with self._graph_mutex:
            if (source is None or target is None) and edge_class is None:
                return list(self.graph.edges)

            if edge_class is not None:
                return [e for e in self.graph.edges if e.edge_class == edge_class]

            return [
                e
                for e in self.graph.edges
                if e.source_node == source and e.target_node == target
            ]

    def get_out_edges(self, source: str) -> List[EdgeMsg]:
        """
        @brief Gets all outgoing edges from a node.

        @param source The source node name.
        @return A list of edges originating from the source node.
        """
        with self._graph_mutex:
            return [e for e in self.graph.edges if e.source_node == source]

    def get_in_edges(self, target: str) -> List[EdgeMsg]:
        """
        @brief Gets all incoming edges to a node.

        @param target The target node name.
        @return A list of edges targeting the specified node.
        """
        with self._graph_mutex:
            return [e for e in self.graph.edges if e.target_node == target]

    def get_num_edges(self) -> int:
        """
        @brief Gets the number of edges in the graph.

        @return The number of edges.
        """
        with self._graph_mutex:
            return len(self.graph.edges)
