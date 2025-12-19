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

"""Unit tests for the Graph class."""

import pytest
from knowledge_graph.graph.graph import Graph
from knowledge_graph.graph.node import Node
from knowledge_graph.graph.edge import Edge


class TestGraphNodes:
    """Test suite for Graph node operations."""

    def test_constructor_default(self):
        """Test default constructor creates empty graph."""
        graph = Graph()
        assert graph.get_num_nodes() == 0
        assert graph.get_num_edges() == 0

    def test_create_node(self):
        """Test creating a node."""
        graph = Graph()
        node = graph.create_node("robot", "robot_type")
        assert node.get_name() == "robot"
        assert node.get_type() == "robot_type"
        assert graph.get_num_nodes() == 1

    def test_create_duplicate_node_raises(self):
        """Test creating a duplicate node raises error."""
        graph = Graph()
        graph.create_node("robot", "robot_type")
        with pytest.raises(RuntimeError):
            graph.create_node("robot", "different_type")

    def test_has_node(self):
        """Test checking if a node exists."""
        graph = Graph()
        assert not graph.has_node("robot")
        graph.create_node("robot", "robot_type")
        assert graph.has_node("robot")

    def test_get_node(self):
        """Test getting a node by name."""
        graph = Graph()
        graph.create_node("robot", "robot_type")
        node = graph.get_node("robot")
        assert node.get_name() == "robot"
        assert node.get_type() == "robot_type"

    def test_get_nonexistent_node_raises(self):
        """Test getting a nonexistent node raises error."""
        graph = Graph()
        with pytest.raises(RuntimeError):
            graph.get_node("nonexistent")

    def test_get_nodes(self):
        """Test getting all nodes."""
        graph = Graph()
        graph.create_node("robot1", "type1")
        graph.create_node("robot2", "type2")
        nodes = graph.get_nodes()
        assert len(nodes) == 2
        names = {n.get_name() for n in nodes}
        assert names == {"robot1", "robot2"}

    def test_get_num_nodes(self):
        """Test getting the number of nodes."""
        graph = Graph()
        assert graph.get_num_nodes() == 0
        graph.create_node("robot1", "type1")
        assert graph.get_num_nodes() == 1
        graph.create_node("robot2", "type2")
        assert graph.get_num_nodes() == 2

    def test_update_node_existing(self):
        """Test updating an existing node."""
        graph = Graph()
        graph.create_node("robot", "robot_type")

        updated_node = Node("robot", "new_type")
        updated_node.set_property("speed", 10.0)
        graph.update_node(updated_node)

        node = graph.get_node("robot")
        assert node.get_type() == "new_type"
        assert node.get_property("speed") == 10.0

    def test_update_node_new(self):
        """Test updating a nonexistent node adds it."""
        graph = Graph()
        node = Node("robot", "robot_type")
        graph.update_node(node)
        assert graph.has_node("robot")

    def test_update_nodes(self):
        """Test updating multiple nodes."""
        graph = Graph()
        nodes = [
            Node("robot1", "type1"),
            Node("robot2", "type2"),
        ]
        graph.update_nodes(nodes)
        assert graph.get_num_nodes() == 2

    def test_remove_node(self):
        """Test removing a node."""
        graph = Graph()
        node = graph.create_node("robot", "robot_type")
        assert graph.remove_node(node)
        assert not graph.has_node("robot")

    def test_remove_nonexistent_node(self):
        """Test removing a nonexistent node returns False."""
        graph = Graph()
        node = Node("nonexistent", "type")
        assert not graph.remove_node(node)

    def test_remove_nodes(self):
        """Test removing multiple nodes."""
        graph = Graph()
        node1 = graph.create_node("robot1", "type1")
        node2 = graph.create_node("robot2", "type2")
        graph.create_node("robot3", "type3")

        graph.remove_nodes([node1, node2])
        assert graph.get_num_nodes() == 1
        assert graph.has_node("robot3")


class TestGraphEdges:
    """Test suite for Graph edge operations."""

    def test_create_edge(self):
        """Test creating an edge."""
        graph = Graph()
        graph.create_node("node_a", "type")
        graph.create_node("node_b", "type")
        edge = graph.create_edge("connects", "node_a", "node_b")

        assert edge.get_type() == "connects"
        assert edge.get_source_node() == "node_a"
        assert edge.get_target_node() == "node_b"
        assert graph.get_num_edges() == 1

    def test_create_edge_without_source_node_raises(self):
        """Test creating an edge without source node raises error."""
        graph = Graph()
        graph.create_node("node_b", "type")
        with pytest.raises(RuntimeError):
            graph.create_edge("connects", "node_a", "node_b")

    def test_create_edge_without_target_node_raises(self):
        """Test creating an edge without target node raises error."""
        graph = Graph()
        graph.create_node("node_a", "type")
        with pytest.raises(RuntimeError):
            graph.create_edge("connects", "node_a", "node_b")

    def test_create_duplicate_edge_raises(self):
        """Test creating a duplicate edge raises error."""
        graph = Graph()
        graph.create_node("node_a", "type")
        graph.create_node("node_b", "type")
        graph.create_edge("connects", "node_a", "node_b")
        with pytest.raises(RuntimeError):
            graph.create_edge("connects", "node_a", "node_b")

    def test_has_edge(self):
        """Test checking if an edge exists."""
        graph = Graph()
        graph.create_node("node_a", "type")
        graph.create_node("node_b", "type")
        assert not graph.has_edge("connects", "node_a", "node_b")
        graph.create_edge("connects", "node_a", "node_b")
        assert graph.has_edge("connects", "node_a", "node_b")

    def test_get_edge(self):
        """Test getting an edge by type and nodes."""
        graph = Graph()
        graph.create_node("node_a", "type")
        graph.create_node("node_b", "type")
        graph.create_edge("connects", "node_a", "node_b")

        edge = graph.get_edge("connects", "node_a", "node_b")
        assert edge.get_type() == "connects"

    def test_get_nonexistent_edge_raises(self):
        """Test getting a nonexistent edge raises error."""
        graph = Graph()
        with pytest.raises(RuntimeError):
            graph.get_edge("connects", "node_a", "node_b")

    def test_get_edges(self):
        """Test getting all edges."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("e1", "a", "b")
        graph.create_edge("e2", "b", "c")

        edges = graph.get_edges()
        assert len(edges) == 2

    def test_get_num_edges(self):
        """Test getting the number of edges."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        assert graph.get_num_edges() == 0
        graph.create_edge("connects", "a", "b")
        assert graph.get_num_edges() == 1

    def test_get_edges_from_node(self):
        """Test getting edges from a source node."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("e1", "a", "b")
        graph.create_edge("e2", "a", "c")
        graph.create_edge("e3", "b", "c")

        edges = graph.get_edges_from_node("a")
        assert len(edges) == 2

    def test_get_edges_to_node(self):
        """Test getting edges to a target node."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("e1", "a", "c")
        graph.create_edge("e2", "b", "c")
        graph.create_edge("e3", "a", "b")

        edges = graph.get_edges_to_node("c")
        assert len(edges) == 2

    def test_get_edges_between_nodes(self):
        """Test getting edges between specific nodes."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_edge("e1", "a", "b")
        graph.create_edge("e2", "a", "b")

        edges = graph.get_edges_between_nodes("a", "b")
        assert len(edges) == 2

    def test_get_edges_by_type(self):
        """Test getting edges by type."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("connects", "a", "b")
        graph.create_edge("relates", "b", "c")
        graph.create_edge("connects", "a", "c")

        edges = graph.get_edges_by_type("connects")
        assert len(edges) == 2

    def test_get_edges_from_node_by_type(self):
        """Test getting edges from node by type."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("connects", "a", "b")
        graph.create_edge("relates", "a", "c")

        edges = graph.get_edges_from_node_by_type("connects", "a")
        assert len(edges) == 1
        assert edges[0].get_target_node() == "b"

    def test_get_edges_to_node_by_type(self):
        """Test getting edges to node by type."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        graph.create_edge("connects", "a", "c")
        graph.create_edge("relates", "b", "c")

        edges = graph.get_edges_to_node_by_type("connects", "c")
        assert len(edges) == 1
        assert edges[0].get_source_node() == "a"

    def test_update_edge_existing(self):
        """Test updating an existing edge."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_edge("connects", "a", "b")

        updated_edge = Edge("connects", "a", "b")
        updated_edge.set_property("weight", 0.5)
        graph.update_edge(updated_edge)

        edge = graph.get_edge("connects", "a", "b")
        assert abs(edge.get_property("weight") - 0.5) < 0.001

    def test_update_edge_new(self):
        """Test updating a nonexistent edge adds it."""
        graph = Graph()
        edge = Edge("connects", "a", "b")
        graph.update_edge(edge)
        assert graph.get_num_edges() == 1

    def test_update_edges(self):
        """Test updating multiple edges."""
        graph = Graph()
        edges = [
            Edge("e1", "a", "b"),
            Edge("e2", "b", "c"),
        ]
        graph.update_edges(edges)
        assert graph.get_num_edges() == 2

    def test_remove_edge(self):
        """Test removing an edge."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        edge = graph.create_edge("connects", "a", "b")

        assert graph.remove_edge(edge)
        assert graph.get_num_edges() == 0

    def test_remove_nonexistent_edge(self):
        """Test removing a nonexistent edge returns False."""
        graph = Graph()
        edge = Edge("connects", "a", "b")
        assert not graph.remove_edge(edge)

    def test_remove_edges(self):
        """Test removing multiple edges."""
        graph = Graph()
        graph.create_node("a", "type")
        graph.create_node("b", "type")
        graph.create_node("c", "type")
        edge1 = graph.create_edge("e1", "a", "b")
        edge2 = graph.create_edge("e2", "b", "c")
        graph.create_edge("e3", "a", "c")

        graph.remove_edges([edge1, edge2])
        assert graph.get_num_edges() == 1


class TestGraphSerialization:
    """Test suite for Graph serialization."""

    def test_to_msg(self):
        """Test converting graph to message."""
        graph = Graph()
        graph.create_node("a", "type1")
        graph.create_node("b", "type2")
        graph.create_edge("connects", "a", "b")

        msg = graph.to_msg()
        assert len(msg.nodes) == 2
        assert len(msg.edges) == 1

    def test_constructor_from_message(self):
        """Test constructing graph from message."""
        # Create original graph
        original = Graph()
        original.create_node("a", "type1")
        original.create_node("b", "type2")
        original.create_edge("connects", "a", "b")

        # Convert to message and back
        msg = original.to_msg()
        restored = Graph(msg)

        assert restored.get_num_nodes() == 2
        assert restored.get_num_edges() == 1
        assert restored.has_node("a")
        assert restored.has_node("b")
        assert restored.has_edge("connects", "a", "b")

    def test_update_graph(self):
        """Test updating graph with another graph."""
        graph1 = Graph()
        graph1.create_node("a", "type")

        graph2 = Graph()
        graph2.create_node("b", "type")
        edge = Edge("connects", "a", "b")
        graph2.update_edge(edge)

        graph1.update_graph(graph2)
        assert graph1.has_node("a")
        assert graph1.has_node("b")
        assert graph1.has_edge("connects", "a", "b")
