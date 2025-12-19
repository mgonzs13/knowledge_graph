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
"""

from typing import Optional, Union, List
from threading import RLock

from rclpy.node import Node as ROSNode
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time

from knowledge_graph.graph import Graph, Node, Edge
from knowledge_graph_msgs.msg import GraphUpdate


class KnowledgeGraph(Graph):
    """
    @brief Class for managing a distributed knowledge graph.

    The KnowledgeGraph class provides functionality for managing nodes and edges
    in a graph structure with support for distributed synchronization across
    multiple ROS 2 nodes.
    """

    # QoS profile for graph updates
    _UPDATE_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    def __init__(self, provided_node: ROSNode) -> None:
        """
        @brief Initializes the KnowledgeGraph.

        @param provided_node The ROS 2 node to use for communication.
        """
        super().__init__()
        self._node = provided_node
        self._graph_id = self._node.get_name()
        self._last_ts = self._node.get_clock().now()
        self._start_time = self._node.get_clock().now()

        # Mutex for thread-safe graph operations (reentrant to allow nested calls)
        self._graph_mutex = RLock()

        # Create publisher and subscriber for graph updates
        self._update_pub = self._node.create_publisher(
            GraphUpdate, "graph_update", self._UPDATE_QOS
        )
        self._update_sub = self._node.create_subscription(
            GraphUpdate, "graph_update", self._update_callback, self._UPDATE_QOS
        )

        # Start sync timer
        self._reqsync_timer = self._node.create_timer(0.1, self._reqsync_timer_callback)
        self._reqsync_timer_callback()

    # =========================================================================
    # Private Methods
    # =========================================================================
    def _reqsync_timer_callback(self) -> None:
        """@brief Timer callback for requesting graph synchronization."""
        elapsed = (self._node.get_clock().now() - self._start_time).nanoseconds
        if Time(nanoseconds=elapsed).seconds_nanoseconds()[0] > 1.0:
            self._reqsync_timer.cancel()

        self._publish_update(
            GraphUpdate.REQSYNC,
            self,
            element_type=GraphUpdate.GRAPH,
        )

    def _publish_update(
        self,
        operation_type: int,
        element: Union[Node, Edge, Graph, List[Node], List[Edge]],
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
        update_msg.stamp = self._node.get_clock().now().to_msg()
        update_msg.node_id = self._graph_id
        update_msg.target_node = target_node
        update_msg.operation_type = operation_type

        if isinstance(element, (Node, Edge, Graph)):
            if isinstance(element, Node):
                update_msg.element_type = GraphUpdate.NODE
                update_msg.nodes = [element.to_msg()]
            elif isinstance(element, Edge):
                update_msg.element_type = GraphUpdate.EDGE
                update_msg.edges = [element.to_msg()]
            elif isinstance(element, Graph):
                update_msg.element_type = GraphUpdate.GRAPH
                update_msg.graph = element.to_msg()
        elif isinstance(element, list):
            if element and isinstance(element[0], Node):
                update_msg.element_type = GraphUpdate.NODE
                update_msg.nodes = [n.to_msg() for n in element]
            elif element and isinstance(element[0], Edge):
                update_msg.element_type = GraphUpdate.EDGE
                update_msg.edges = [e.to_msg() for e in element]

        self._update_pub.publish(update_msg)

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
        if author_id == self._graph_id:
            return

            # Check for out-of-order updates
        if ts < self._last_ts and operation not in (
            GraphUpdate.REQSYNC,
            GraphUpdate.SYNC,
        ):
            self._node.get_logger().warn(
                f"UNORDERED UPDATE [{operation}] "
                f"{self._last_ts.seconds_nanoseconds()[0]} > {ts.seconds_nanoseconds()[0]}"
            )

        with self._graph_mutex:
            # Process update based on element type
            if element == GraphUpdate.NODE:
                if operation == GraphUpdate.UPDATE:
                    for node_msg in msg.nodes:
                        node = Node(msg=node_msg)
                        Graph.update_node(self, node)
                elif operation == GraphUpdate.REMOVE:
                    for node_msg in msg.nodes:
                        node = Node(msg=node_msg)
                        Graph.remove_node(self, node)

            elif element == GraphUpdate.EDGE:
                if operation == GraphUpdate.UPDATE:
                    for edge_msg in msg.edges:
                        edge = Edge(msg=edge_msg)
                        Graph.update_edge(self, edge)
                elif operation == GraphUpdate.REMOVE:
                    for edge_msg in msg.edges:
                        edge = Edge(msg=edge_msg)
                        Graph.remove_edge(self, edge)

            elif element == GraphUpdate.GRAPH:
                remote_graph = Graph(msg=msg.graph)
                if operation == GraphUpdate.SYNC and msg.target_node == self._graph_id:
                    self._reqsync_timer.cancel()
                    self.update_graph(remote_graph)
                elif operation == GraphUpdate.REQSYNC and msg.node_id != self._graph_id:
                    self._publish_update(
                        GraphUpdate.SYNC,
                        self,
                        target_node=msg.node_id,
                        element_type=GraphUpdate.GRAPH,
                    )
                    self.update_graph(remote_graph)

            # Update timestamp to the latest received
            self._last_ts = max(self._last_ts, ts)

    # =========================================================================
    # Graph Management Functions
    # =========================================================================
    def update_graph(self, graph: Graph) -> None:
        with self._graph_mutex:
            for node in graph.get_nodes():
                Graph.update_node(self, node)
            for edge in graph.get_edges():
                Graph.update_edge(self, edge)

    def to_msg(self):
        with self._graph_mutex:
            return super().to_msg()

    # =========================================================================
    # Node Management Functions
    # =========================================================================
    def create_node(self, name: str, type_: str) -> Node:
        with self._graph_mutex:
            node = super().create_node(name, type_)
            self._publish_update(GraphUpdate.UPDATE, node)
            return node

    def has_node(self, name: str) -> bool:
        with self._graph_mutex:
            return super().has_node(name)

    def get_num_nodes(self) -> int:
        with self._graph_mutex:
            return super().get_num_nodes()

    def get_nodes(self) -> List[Node]:
        with self._graph_mutex:
            return super().get_nodes()

    def get_node(self, name: str) -> Node:
        with self._graph_mutex:
            return super().get_node(name)

    def update_node(self, node: Node) -> None:
        with self._graph_mutex:
            super().update_node(node)
            self._publish_update(GraphUpdate.UPDATE, node)

    def update_nodes(self, nodes: List[Node]) -> None:
        with self._graph_mutex:
            for node in nodes:
                Graph.update_node(self, node)
            self._publish_update(GraphUpdate.UPDATE, nodes)

    def remove_node(self, node: Node) -> bool:
        with self._graph_mutex:
            removed = super().remove_node(node)
            if removed:
                self._publish_update(GraphUpdate.REMOVE, node)
            return removed

    def remove_nodes(self, nodes: List[Node]) -> None:
        with self._graph_mutex:
            removed_nodes = []
            for node in nodes:
                if Graph.remove_node(self, node):
                    removed_nodes.append(node)
            if removed_nodes:
                self._publish_update(GraphUpdate.REMOVE, removed_nodes)

    # =========================================================================
    # Edge Management Functions
    # =========================================================================
    def create_edge(self, type_: str, source_node: str, target_node: str) -> Edge:
        with self._graph_mutex:
            edge = super().create_edge(type_, source_node, target_node)
            self._publish_update(GraphUpdate.UPDATE, edge)
            return edge

    def has_edge(self, type: str, source_node: str, target_node: str) -> bool:
        with self._graph_mutex:
            return super().has_edge(type, source_node, target_node)

    def get_num_edges(self) -> int:
        with self._graph_mutex:
            return super().get_num_edges()

    def get_edges(self) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges()

    def get_edges_from_node(self, source_node: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_from_node(source_node)

    def get_edges_to_node(self, target_node: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_to_node(target_node)

    def get_edges_between_nodes(self, source_node: str, target_node: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_between_nodes(source_node, target_node)

    def get_edges_by_type(self, type: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_by_type(type)

    def get_edges_from_node_by_type(self, type: str, source_node: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_from_node_by_type(type, source_node)

    def get_edges_to_node_by_type(self, type: str, target_node: str) -> List[Edge]:
        with self._graph_mutex:
            return super().get_edges_to_node_by_type(type, target_node)

    def get_edge(self, type: str, source_node: str, target_node: str) -> Edge:
        with self._graph_mutex:
            return super().get_edge(type, source_node, target_node)

    def update_edge(self, edge: Edge) -> None:
        with self._graph_mutex:
            super().update_edge(edge)
            self._publish_update(GraphUpdate.UPDATE, edge)

    def update_edges(self, edges: List[Edge]) -> None:
        with self._graph_mutex:
            for edge in edges:
                Graph.update_edge(self, edge)
            self._publish_update(GraphUpdate.UPDATE, edges)

    def remove_edge(self, edge: Edge) -> bool:
        with self._graph_mutex:
            removed = super().remove_edge(edge)
            if removed:
                self._publish_update(GraphUpdate.REMOVE, edge)
            return removed

    def remove_edges(self, edges: List[Edge]) -> None:
        with self._graph_mutex:
            removed_edges = []
            for edge in edges:
                if Graph.remove_edge(self, edge):
                    removed_edges.append(edge)
            if removed_edges:
                self._publish_update(GraphUpdate.REMOVE, removed_edges)
