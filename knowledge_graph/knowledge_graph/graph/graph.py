# Copyright 2025 Miguel Ángel González Santamarta

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List, Callable, Union
from knowledge_graph.graph.node import Node
from knowledge_graph.graph.edge import Edge
from knowledge_graph_msgs.msg import Graph as GraphMsg


class Graph:
    """
    Class representing a knowledge graph with nodes and edges.
    """

    def __init__(self, msg: GraphMsg = None) -> None:
        """
        Default constructor or from a Graph message.
        :param msg: The Graph message to initialize the graph from.
        """
        self._nodes: List[Node] = []
        self._edges: List[Edge] = []
        self._callbacks: List[Callable[[str, str, List[Union[Node, Edge]]], None]] = []
        if msg:
            for node_msg in msg.nodes:
                node = Node(msg=node_msg)
                self._nodes.append(node)
            for edge_msg in msg.edges:
                edge = Edge(msg=edge_msg)
                self._edges.append(edge)

    def update_graph(self, graph: "Graph") -> None:
        """
        Update the graph with another graph.
        :param graph: The graph to update from.
        """
        for node in graph.get_nodes():
            self.update_node(node)
        for edge in graph.get_edges():
            self.update_edge(edge)

    def add_callback(
        self, callback: Callable[[str, str, List[Union[Node, Edge]]], None]
    ) -> None:
        """
        Add a callback to be called when the graph is updated.
        :param callback: The callback function with signature (operation, element_type, elements).
        """
        self._callbacks.append(callback)

    def clear_callbacks(self) -> None:
        """
        Clear all registered callbacks.
        """
        self._callbacks.clear()

    def _notify_callbacks(
        self, operation: str, element_type: str, elements: List[Union[Node, Edge]]
    ) -> None:
        """
        Notify all registered callbacks.
        :param operation: 'add', 'update', or 'remove'.
        :param element_type: 'node' or 'edge'.
        :param elements: The list of Node or Edge objects.
        """
        for callback in self._callbacks:
            callback(operation, element_type, elements)

    def to_msg(self) -> GraphMsg:
        """
        Convert the graph to a Graph message.
        :return: The Graph message representation of the graph.
        """
        graph_msg = GraphMsg()
        for node in self._nodes:
            graph_msg.nodes.append(node.to_msg())
        for edge in self._edges:
            graph_msg.edges.append(edge.to_msg())
        return graph_msg

    # =========================================================================
    # Node methods
    # =========================================================================
    def create_node(self, name: str, type: str) -> Node:
        """
        Create a new node in the graph.
        :param name: The name of the node.
        :param type_: The type of the node.
        :return: The created Node.
        """
        if self.has_node(name):
            raise RuntimeError(f"Node already exists: {name}")
        node = Node(name, type)
        self._nodes.append(node)
        self._notify_callbacks("add", "node", [node])
        return node

    def has_node(self, name: str) -> bool:
        """
        Check if a node exists in the graph.
        :param name: The name of the node.
        :return: True if the node exists, false otherwise.
        """
        for node in self._nodes:
            if node.get_name() == name:
                return True
        return False

    def get_num_nodes(self) -> int:
        """
        Get the number of nodes in the graph.
        :return: Number of nodes in the graph.
        """
        return len(self._nodes)

    def get_nodes(self) -> List[Node]:
        """
        Get all nodes in the graph.
        :return: List of all nodes in the graph.
        """
        return self._nodes[:]

    def get_node(self, name: str) -> Node:
        """
        Get a node by name.
        :param name: The name of the node.
        :return: The Node with the given name.
        """
        for node in self._nodes:
            if node.get_name() == name:
                return node
        raise RuntimeError(f"Node not found: {name}")

    def _update_node_internal(self, node: Node) -> bool:
        """
        Internal update node without notification.
        :param node: The node to update.
        :return: True if the node was updated (existed), False if it was added (new).
        """
        existing = False
        for i, existing_node in enumerate(self._nodes):
            if existing_node.get_name() == node.get_name():
                self._nodes[i] = node
                existing = True
                break
        if not existing:
            self._nodes.append(node)
        return existing

    def update_node(self, node: Node) -> None:
        """
        Update a node in the graph.
        :param node: The node to update.
        """
        existing = self._update_node_internal(node)
        self._notify_callbacks("update" if existing else "add", "node", [node])

    def update_nodes(self, nodes: List[Node]) -> None:
        """
        Update nodes in the graph.
        :param nodes: The nodes to update.
        """
        added = []
        updated = []
        for node in nodes:
            existing = self._update_node_internal(node)
            if existing:
                updated.append(node)
            else:
                added.append(node)
        if added:
            self._notify_callbacks("add", "node", added)
        if updated:
            self._notify_callbacks("update", "node", updated)

    def _remove_node_internal(self, node: Node) -> bool:
        """
        Internal remove node without notification.
        :param node: The node to remove.
        :return: True if the node was removed, false if it was not found.
        """
        for i, n in enumerate(self._nodes):
            if n.get_name() == node.get_name():
                del self._nodes[i]
                return True
        return False

    def remove_node(self, node: Node) -> bool:
        """
        Remove a node from the graph.
        :param node: The node to remove.
        :return: True if the node was removed, false if it was not found.
        """
        removed = self._remove_node_internal(node)
        if removed:
            self._notify_callbacks("remove", "node", [node])
        return removed

    def remove_nodes(self, nodes: List[Node]) -> List[Node]:
        """
        Remove nodes from the graph.
        :param nodes: The nodes to remove.
        """
        removed = []
        for node in nodes:
            if self._remove_node_internal(node):
                removed.append(node)
        if removed:
            self._notify_callbacks("remove", "node", removed)
        return removed

    # =========================================================================
    # Edge methods
    # =========================================================================
    def create_edge(self, type: str, source_node: str, target_node: str) -> Edge:
        """
        Create a new edge in the graph.
        :param type_: The type of the edge.
        :param source_node: The source node name.
        :param target_node: The target node name.
        :return: The created Edge.
        """
        if not self.has_node(source_node):
            raise RuntimeError(f"Source node does not exist: {source_node}")
        if not self.has_node(target_node):
            raise RuntimeError(f"Target node does not exist: {target_node}")
        if self.has_edge(type, source_node, target_node):
            raise RuntimeError(
                f"Edge already exists: {type} from {source_node} to {target_node}"
            )
        edge = Edge(type, source_node, target_node)
        self._edges.append(edge)
        self._notify_callbacks("add", "edge", [edge])
        return edge

    def has_edge(self, type: str, source_node: str, target_node: str) -> bool:
        """
        Check if an edge exists in the graph.
        :param type: The type of the edge.
        :param source_node: The source node name.
        :param target_node: The target node name.
        :return: True if the edge exists, false otherwise.
        """
        for edge in self._edges:
            if (
                edge.get_type() == type
                and edge.get_source_node() == source_node
                and edge.get_target_node() == target_node
            ):
                return True
        return False

    def get_num_edges(self) -> int:
        """
        Get the number of edges in the graph.
        :return: Number of edges in the graph.
        """
        return len(self._edges)

    def get_edges(self) -> List[Edge]:
        """
        Get all edges in the graph.
        :return: List of all edges in the graph.
        """
        return self._edges[:]

    def get_edges_from_node(self, source_node: str) -> List[Edge]:
        """
        Get edges from a specific source node.
        :param source_node: The source node name.
        :return: List of edges from the specified source node.
        """
        return [e for e in self._edges if e.get_source_node() == source_node]

    def get_edges_to_node(self, target_node: str) -> List[Edge]:
        """
        Get edges to a specific target node.
        :param target_node: The target node name.
        :return: List of edges to the specified target node.
        """
        return [e for e in self._edges if e.get_target_node() == target_node]

    def get_edges_between_nodes(self, source_node: str, target_node: str) -> List[Edge]:
        """
        Get edges between a specific source and target node.
        :param source_node: The source node name.
        :param target_node: The target node name.
        :return: List of edges between the specified source and target nodes.
        """
        return [
            e
            for e in self._edges
            if e.get_source_node() == source_node and e.get_target_node() == target_node
        ]

    def get_edges_by_type(self, type: str) -> List[Edge]:
        """
        Get edges of a specific type.
        :param type: The edge type.
        :return: List of edges of the specified type.
        """
        return [e for e in self._edges if e.get_type() == type]

    def get_edges_from_node_by_type(self, type: str, source_node: str) -> List[Edge]:
        """
        Get edges from a specific source node of a specific type.
        :param type: The edge type.
        :param source_node: The source node name.
        :return: List of edges from the specified source node of the specified type.
        """
        return [
            e
            for e in self._edges
            if e.get_source_node() == source_node and e.get_type() == type
        ]

    def get_edges_to_node_by_type(self, type: str, target_node: str) -> List[Edge]:
        """
        Get edges to a specific target node of a specific type.
        :param type: The edge type.
        :param target_node: The target node name.
        :return: List of edges to the specified target node of the specified type.
        """
        return [
            e
            for e in self._edges
            if e.get_target_node() == target_node and e.get_type() == type
        ]

    def get_edge(self, type: str, source_node: str, target_node: str) -> Edge:
        """
        Get an edge by type, source node, and target node.
        :param type_: The edge type.
        :param source_node: The source node name.
        :param target_node: The target node name.
        :return: The Edge with the specified type, source node, and target node.
        """
        for edge in self._edges:
            if (
                edge.get_type() == type
                and edge.get_source_node() == source_node
                and edge.get_target_node() == target_node
            ):
                return edge
        raise RuntimeError(f"Edge not found: {type} from {source_node} to {target_node}")

    def _update_edge_internal(self, edge: Edge) -> bool:
        """
        Internal update edge without notification.
        :param edge: The edge to update.
        :return: True if the edge was updated (existed), False if it was added (new).
        """
        existing = False
        for i, existing_edge in enumerate(self._edges):
            if (
                existing_edge.get_type() == edge.get_type()
                and existing_edge.get_source_node() == edge.get_source_node()
                and existing_edge.get_target_node() == edge.get_target_node()
            ):
                self._edges[i] = edge
                existing = True
                break
        if not existing:
            self._edges.append(edge)
        return existing

    def update_edge(self, edge: Edge) -> None:
        """
        Update an edge in the graph.
        :param edge: The edge to update.
        """
        existing = self._update_edge_internal(edge)
        self._notify_callbacks("update" if existing else "add", "edge", [edge])

    def update_edges(self, edges: List[Edge]) -> None:
        """
        Update edges in the graph.
        :param edges: The edges to update.
        """
        added = []
        updated = []
        for edge in edges:
            existing = self._update_edge_internal(edge)
            if existing:
                updated.append(edge)
            else:
                added.append(edge)
        if added:
            self._notify_callbacks("add", "edge", added)
        if updated:
            self._notify_callbacks("update", "edge", updated)

    def _remove_edge_internal(self, edge: Edge) -> bool:
        """
        Internal remove edge without notification.
        :param edge: The edge to remove.
        :return: True if the edge was removed, false if it was not found.
        """
        for i, e in enumerate(self._edges):
            if (
                e.get_type() == edge.get_type()
                and e.get_source_node() == edge.get_source_node()
                and e.get_target_node() == edge.get_target_node()
            ):
                del self._edges[i]
                return True
        return False

    def remove_edge(self, edge: Edge) -> bool:
        """
        Remove an edge from the graph.
        :param edge: The edge to remove.
        :return: True if the edge was removed, false if it was not found.
        """
        removed = self._remove_edge_internal(edge)
        if removed:
            self._notify_callbacks("remove", "edge", [edge])
        return removed

    def remove_edges(self, edges: List[Edge]) -> List[Edge]:
        """
        Remove edges from the graph.
        :param edges: The edges to remove.
        """
        removed = []
        for edge in edges:
            if self._remove_edge_internal(edge):
                removed.append(edge)
        if removed:
            self._notify_callbacks("remove", "edge", removed)
        return removed
