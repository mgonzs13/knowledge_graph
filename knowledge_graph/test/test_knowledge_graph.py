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

import pytest
import rclpy
from knowledge_graph.knowledge_graph import KnowledgeGraph
from knowledge_graph.graph.node import Node
from knowledge_graph.graph.edge import Edge
from knowledge_graph.graph.graph import Graph


@pytest.fixture(scope="module", autouse=True)
def rclpy_init():
    """Initialize rclpy for the test module."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def knowledge_graph():
    """Provide a KnowledgeGraph instance for tests."""
    kg = KnowledgeGraph.get_instance()
    # Clear all callbacks first to avoid dangling references from previous tests
    kg.clear_callbacks()
    # Clear the graph before each test
    for edge in kg.get_edges():
        kg.remove_edge(edge)
    for node in kg.get_nodes():
        kg.remove_node(node)
    return kg


class TestKnowledgeGraphSingleton:
    """Test suite for KnowledgeGraph singleton behavior."""

    def test_get_instance_returns_same_instance(self, knowledge_graph):
        """Test that get_instance always returns the same instance."""
        kg1 = KnowledgeGraph.get_instance()
        kg2 = KnowledgeGraph.get_instance()
        assert kg1 is kg2

    def test_direct_instantiation_raises(self, knowledge_graph):
        """Test that direct instantiation raises RuntimeError."""
        with pytest.raises(RuntimeError):
            KnowledgeGraph()


class TestKnowledgeGraphNodes:
    """Test suite for KnowledgeGraph node operations."""

    def test_create_node(self, knowledge_graph):
        """Test creating a node."""
        node = knowledge_graph.create_node("robot", "robot_type")
        assert node.get_name() == "robot"
        assert node.get_type() == "robot_type"
        assert knowledge_graph.get_num_nodes() == 1

    def test_has_node(self, knowledge_graph):
        """Test checking if a node exists."""
        assert not knowledge_graph.has_node("robot")
        knowledge_graph.create_node("robot", "robot_type")
        assert knowledge_graph.has_node("robot")

    def test_get_node(self, knowledge_graph):
        """Test getting a node by name."""
        knowledge_graph.create_node("robot", "robot_type")
        node = knowledge_graph.get_node("robot")
        assert node.get_name() == "robot"
        assert node.get_type() == "robot_type"

    def test_get_nonexistent_node_raises(self, knowledge_graph):
        """Test getting a nonexistent node raises error."""
        with pytest.raises(RuntimeError):
            knowledge_graph.get_node("nonexistent")

    def test_get_nodes(self, knowledge_graph):
        """Test getting all nodes."""
        knowledge_graph.create_node("robot1", "type1")
        knowledge_graph.create_node("robot2", "type2")
        nodes = knowledge_graph.get_nodes()
        assert len(nodes) == 2
        names = {n.get_name() for n in nodes}
        assert names == {"robot1", "robot2"}

    def test_get_num_nodes(self, knowledge_graph):
        """Test getting the number of nodes."""
        assert knowledge_graph.get_num_nodes() == 0
        knowledge_graph.create_node("robot1", "type1")
        assert knowledge_graph.get_num_nodes() == 1
        knowledge_graph.create_node("robot2", "type2")
        assert knowledge_graph.get_num_nodes() == 2

    def test_update_node_existing(self, knowledge_graph):
        """Test updating an existing node."""
        knowledge_graph.create_node("robot", "robot_type")

        updated_node = Node("robot", "new_type")
        updated_node.set_property("speed", 10.0)
        knowledge_graph.update_node(updated_node)

        node = knowledge_graph.get_node("robot")
        assert node.get_type() == "new_type"
        assert node.get_property("speed") == 10.0

    def test_update_node_new(self, knowledge_graph):
        """Test updating a nonexistent node adds it."""
        node = Node("robot", "robot_type")
        knowledge_graph.update_node(node)
        assert knowledge_graph.has_node("robot")

    def test_update_nodes(self, knowledge_graph):
        """Test updating multiple nodes."""
        nodes = [
            Node("robot1", "type1"),
            Node("robot2", "type2"),
        ]
        knowledge_graph.update_nodes(nodes)
        assert knowledge_graph.get_num_nodes() == 2

    def test_remove_node(self, knowledge_graph):
        """Test removing a node."""
        node = knowledge_graph.create_node("robot", "robot_type")
        assert knowledge_graph.remove_node(node)
        assert not knowledge_graph.has_node("robot")

    def test_remove_nonexistent_node(self, knowledge_graph):
        """Test removing a nonexistent node returns False."""
        node = Node("nonexistent", "type")
        assert not knowledge_graph.remove_node(node)

    def test_remove_nodes(self, knowledge_graph):
        """Test removing multiple nodes."""
        node1 = knowledge_graph.create_node("robot1", "type1")
        node2 = knowledge_graph.create_node("robot2", "type2")
        knowledge_graph.create_node("robot3", "type3")

        knowledge_graph.remove_nodes([node1, node2])
        assert knowledge_graph.get_num_nodes() == 1
        assert knowledge_graph.has_node("robot3")


class TestKnowledgeGraphEdges:
    """Test suite for KnowledgeGraph edge operations."""

    def test_create_edge(self, knowledge_graph):
        """Test creating an edge."""
        knowledge_graph.create_node("node_a", "type")
        knowledge_graph.create_node("node_b", "type")
        edge = knowledge_graph.create_edge("connects", "node_a", "node_b")
        assert edge.get_type() == "connects"
        assert edge.get_source_node() == "node_a"
        assert edge.get_target_node() == "node_b"
        assert knowledge_graph.get_num_edges() == 1

    def test_has_edge(self, knowledge_graph):
        """Test checking if an edge exists."""
        knowledge_graph.create_node("node_a", "type")
        knowledge_graph.create_node("node_b", "type")
        assert not knowledge_graph.has_edge("connects", "node_a", "node_b")
        knowledge_graph.create_edge("connects", "node_a", "node_b")
        assert knowledge_graph.has_edge("connects", "node_a", "node_b")

    def test_get_num_edges(self, knowledge_graph):
        """Test getting the number of edges."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        assert knowledge_graph.get_num_edges() == 0
        knowledge_graph.create_edge("connects", "a", "b")
        assert knowledge_graph.get_num_edges() == 1

    def test_get_edges(self, knowledge_graph):
        """Test getting all edges."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_node("c", "type")
        knowledge_graph.create_edge("connects", "a", "b")
        knowledge_graph.create_edge("connects", "b", "c")
        edges = knowledge_graph.get_edges()
        assert len(edges) == 2

    def test_get_edges_from_node(self, knowledge_graph):
        """Test getting edges from a specific node."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_node("c", "type")
        knowledge_graph.create_edge("connects", "a", "b")
        knowledge_graph.create_edge("connects", "a", "c")
        knowledge_graph.create_edge("connects", "b", "c")
        edges = knowledge_graph.get_edges_from_node("a")
        assert len(edges) == 2

    def test_get_edges_to_node(self, knowledge_graph):
        """Test getting edges to a specific node."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_node("c", "type")
        knowledge_graph.create_edge("connects", "a", "c")
        knowledge_graph.create_edge("connects", "b", "c")
        edges = knowledge_graph.get_edges_to_node("c")
        assert len(edges) == 2

    def test_get_edges_between_nodes(self, knowledge_graph):
        """Test getting edges between two specific nodes."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("connects1", "a", "b")
        knowledge_graph.create_edge("connects2", "a", "b")
        edges = knowledge_graph.get_edges_between_nodes("a", "b")
        assert len(edges) == 2

    def test_get_edges_by_type(self, knowledge_graph):
        """Test getting edges by type."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("type1", "a", "b")
        knowledge_graph.create_edge("type2", "a", "b")
        edges = knowledge_graph.get_edges_by_type("type1")
        assert len(edges) == 1
        assert edges[0].get_type() == "type1"

    def test_get_edges_from_node_by_type(self, knowledge_graph):
        """Test getting edges from node by type."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_node("c", "type")
        knowledge_graph.create_edge("type1", "a", "b")
        knowledge_graph.create_edge("type1", "a", "c")
        knowledge_graph.create_edge("type2", "a", "b")
        edges = knowledge_graph.get_edges_from_node_by_type("type1", "a")
        assert len(edges) == 2

    def test_get_edges_to_node_by_type(self, knowledge_graph):
        """Test getting edges to node by type."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_node("c", "type")
        knowledge_graph.create_edge("type1", "a", "c")
        knowledge_graph.create_edge("type1", "b", "c")
        knowledge_graph.create_edge("type2", "a", "c")
        edges = knowledge_graph.get_edges_to_node_by_type("type1", "c")
        assert len(edges) == 2

    def test_get_edge(self, knowledge_graph):
        """Test getting a specific edge."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("connects", "a", "b")
        edge = knowledge_graph.get_edge("connects", "a", "b")
        assert edge.get_type() == "connects"
        assert edge.get_source_node() == "a"
        assert edge.get_target_node() == "b"

    def test_get_nonexistent_edge_raises(self, knowledge_graph):
        """Test getting a nonexistent edge raises error."""
        with pytest.raises(RuntimeError):
            knowledge_graph.get_edge("connects", "a", "b")

    def test_update_edge(self, knowledge_graph):
        """Test updating an edge."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("connects", "a", "b")

        updated_edge = Edge("connects", "a", "b")
        updated_edge.set_property("weight", 5.0)
        knowledge_graph.update_edge(updated_edge)

        edge = knowledge_graph.get_edge("connects", "a", "b")
        assert edge.get_property("weight") == 5.0

    def test_update_edges(self, knowledge_graph):
        """Test updating multiple edges."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edges = [
            Edge("type1", "a", "b"),
            Edge("type2", "a", "b"),
        ]
        knowledge_graph.update_edges(edges)
        assert knowledge_graph.get_num_edges() == 2

    def test_remove_edge(self, knowledge_graph):
        """Test removing an edge."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edge = knowledge_graph.create_edge("connects", "a", "b")
        assert knowledge_graph.remove_edge(edge)
        assert not knowledge_graph.has_edge("connects", "a", "b")

    def test_remove_nonexistent_edge(self, knowledge_graph):
        """Test removing a nonexistent edge returns False."""
        edge = Edge("connects", "a", "b")
        assert not knowledge_graph.remove_edge(edge)

    def test_remove_edges(self, knowledge_graph):
        """Test removing multiple edges."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edge1 = knowledge_graph.create_edge("type1", "a", "b")
        edge2 = knowledge_graph.create_edge("type2", "a", "b")
        knowledge_graph.create_edge("type3", "a", "b")

        knowledge_graph.remove_edges([edge1, edge2])
        assert knowledge_graph.get_num_edges() == 1
        assert knowledge_graph.has_edge("type3", "a", "b")


class TestKnowledgeGraphGraphOperations:
    """Test suite for KnowledgeGraph graph-level operations."""

    def test_update_graph(self, knowledge_graph):
        """Test updating the graph from another graph."""
        other_graph = Graph()
        other_graph.create_node("external_node1", "type1")
        other_graph.create_node("external_node2", "type2")
        other_graph.create_edge("connects", "external_node1", "external_node2")

        knowledge_graph.update_graph(other_graph)

        assert knowledge_graph.has_node("external_node1")
        assert knowledge_graph.has_node("external_node2")
        assert knowledge_graph.has_edge("connects", "external_node1", "external_node2")

    def test_to_msg(self, knowledge_graph):
        """Test converting the graph to a message."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("connects", "a", "b")

        msg = knowledge_graph.to_msg()

        assert len(msg.nodes) == 2
        assert len(msg.edges) == 1


class TestKnowledgeGraphNodeProperties:
    """Test suite for KnowledgeGraph node properties."""

    def test_node_with_properties(self, knowledge_graph):
        """Test creating and retrieving node with properties."""
        node = knowledge_graph.create_node("robot", "robot_type")
        node.set_property("battery", 100.0)
        node.set_property("active", True)
        node.set_property("name", "RobotOne")
        knowledge_graph.update_node(node)

        retrieved = knowledge_graph.get_node("robot")
        assert retrieved.get_property("battery") == 100.0
        assert retrieved.get_property("active") is True
        assert retrieved.get_property("name") == "RobotOne"


class TestKnowledgeGraphEdgeProperties:
    """Test suite for KnowledgeGraph edge properties."""

    def test_edge_with_properties(self, knowledge_graph):
        """Test creating and retrieving edge with properties."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edge = knowledge_graph.create_edge("path", "a", "b")
        edge.set_property("distance", 10.5)
        edge.set_property("traversable", True)
        knowledge_graph.update_edge(edge)

        retrieved = knowledge_graph.get_edge("path", "a", "b")
        assert retrieved.get_property("distance") == 10.5
        assert retrieved.get_property("traversable") is True


class TestKnowledgeGraphComplexScenarios:
    """Test suite for KnowledgeGraph complex scenarios."""

    def test_graph_with_multiple_edge_types(self, knowledge_graph):
        """Test graph with multiple edge types between same nodes."""
        knowledge_graph.create_node("robot", "robot_type")
        knowledge_graph.create_node("location", "location_type")

        knowledge_graph.create_edge("at", "robot", "location")
        knowledge_graph.create_edge("visited", "robot", "location")
        knowledge_graph.create_edge("can_reach", "robot", "location")

        edges = knowledge_graph.get_edges_between_nodes("robot", "location")
        assert len(edges) == 3

        types = {e.get_type() for e in edges}
        assert types == {"at", "visited", "can_reach"}

    def test_self_loop_edge(self, knowledge_graph):
        """Test creating a self-loop edge (node connected to itself)."""
        knowledge_graph.create_node("robot", "robot_type")
        edge = knowledge_graph.create_edge("monitors", "robot", "robot")

        assert edge.get_source_node() == "robot"
        assert edge.get_target_node() == "robot"
        assert knowledge_graph.has_edge("monitors", "robot", "robot")


class TestKnowledgeGraphCallbacks:
    """Test suite for KnowledgeGraph callback functionality."""

    def test_add_callback(self, knowledge_graph):
        """Test adding a callback to the knowledge graph."""
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.create_node("robot", "robot_type")

        assert len(callback_data) == 1
        assert callback_data[0][0] == "add"
        assert callback_data[0][1] == "node"
        assert len(callback_data[0][2]) == 1
        assert callback_data[0][2][0].get_name() == "robot"

    def test_callback_on_node_create(self, knowledge_graph):
        """Test callback is invoked when a node is created."""
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.create_node("robot", "robot_type")

        assert len(callback_data) == 1
        assert callback_data[0][0] == "add"
        assert callback_data[0][1] == "node"

    def test_callback_on_node_update_existing(self, knowledge_graph):
        """Test callback is invoked when an existing node is updated."""
        knowledge_graph.create_node("robot", "robot_type")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        updated_node = Node("robot", "new_type")
        knowledge_graph.update_node(updated_node)

        assert len(callback_data) == 1
        assert callback_data[0][0] == "update"
        assert callback_data[0][1] == "node"

    def test_callback_on_node_update_new(self, knowledge_graph):
        """Test callback is invoked when a new node is added via update."""
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        new_node = Node("robot", "robot_type")
        knowledge_graph.update_node(new_node)

        assert len(callback_data) == 1
        assert callback_data[0][0] == "add"
        assert callback_data[0][1] == "node"

    def test_callback_on_update_nodes(self, knowledge_graph):
        """Test callback is invoked when multiple nodes are updated."""
        knowledge_graph.create_node("robot1", "type1")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        nodes = [
            Node("robot1", "new_type1"),  # existing - should trigger update
            Node("robot2", "type2"),  # new - should trigger add
        ]
        knowledge_graph.update_nodes(nodes)

        # Should have two callbacks: one for add, one for update
        assert len(callback_data) == 2
        operations = {cb[0] for cb in callback_data}
        assert operations == {"add", "update"}

    def test_callback_on_node_remove(self, knowledge_graph):
        """Test callback is invoked when a node is removed."""
        node = knowledge_graph.create_node("robot", "robot_type")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.remove_node(node)

        assert len(callback_data) == 1
        assert callback_data[0][0] == "remove"
        assert callback_data[0][1] == "node"

    def test_callback_on_remove_nodes(self, knowledge_graph):
        """Test callback is invoked when multiple nodes are removed."""
        node1 = knowledge_graph.create_node("robot1", "type1")
        node2 = knowledge_graph.create_node("robot2", "type2")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.remove_nodes([node1, node2])

        assert len(callback_data) == 1
        assert callback_data[0][0] == "remove"
        assert callback_data[0][1] == "node"
        assert len(callback_data[0][2]) == 2

    def test_callback_on_edge_create(self, knowledge_graph):
        """Test callback is invoked when an edge is created."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.create_edge("connects", "a", "b")

        assert len(callback_data) == 1
        assert callback_data[0][0] == "add"
        assert callback_data[0][1] == "edge"

    def test_callback_on_edge_update_existing(self, knowledge_graph):
        """Test callback is invoked when an existing edge is updated."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("connects", "a", "b")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        updated_edge = Edge("connects", "a", "b")
        updated_edge.set_property("weight", 1.0)
        knowledge_graph.update_edge(updated_edge)

        assert len(callback_data) == 1
        assert callback_data[0][0] == "update"
        assert callback_data[0][1] == "edge"

    def test_callback_on_update_edges(self, knowledge_graph):
        """Test callback is invoked when multiple edges are updated."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        knowledge_graph.create_edge("type1", "a", "b")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        edges = [
            Edge("type1", "a", "b"),  # existing - should trigger update
            Edge("type2", "a", "b"),  # new - should trigger add
        ]
        knowledge_graph.update_edges(edges)

        # Should have two callbacks: one for add, one for update
        assert len(callback_data) == 2
        operations = {cb[0] for cb in callback_data}
        assert operations == {"add", "update"}

    def test_callback_on_edge_remove(self, knowledge_graph):
        """Test callback is invoked when an edge is removed."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edge = knowledge_graph.create_edge("connects", "a", "b")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.remove_edge(edge)

        assert len(callback_data) == 1
        assert callback_data[0][0] == "remove"
        assert callback_data[0][1] == "edge"

    def test_callback_on_remove_edges(self, knowledge_graph):
        """Test callback is invoked when multiple edges are removed."""
        knowledge_graph.create_node("a", "type")
        knowledge_graph.create_node("b", "type")
        edge1 = knowledge_graph.create_edge("type1", "a", "b")
        edge2 = knowledge_graph.create_edge("type2", "a", "b")
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        knowledge_graph.remove_edges([edge1, edge2])

        assert len(callback_data) == 1
        assert callback_data[0][0] == "remove"
        assert callback_data[0][1] == "edge"
        assert len(callback_data[0][2]) == 2

    def test_multiple_callbacks(self, knowledge_graph):
        """Test multiple callbacks can be registered and all are invoked."""
        callback_data1 = []
        callback_data2 = []

        def callback1(operation, element_type, elements):
            callback_data1.append((operation, element_type, elements))

        def callback2(operation, element_type, elements):
            callback_data2.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback1)
        knowledge_graph.add_callback(callback2)
        knowledge_graph.create_node("robot", "robot_type")

        assert len(callback_data1) == 1
        assert len(callback_data2) == 1

    def test_no_callback_on_failed_remove_node(self, knowledge_graph):
        """Test callback is not invoked when remove_node fails."""
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        non_existent_node = Node("nonexistent", "type")
        knowledge_graph.remove_node(non_existent_node)

        assert len(callback_data) == 0

    def test_no_callback_on_failed_remove_edge(self, knowledge_graph):
        """Test callback is not invoked when remove_edge fails."""
        callback_data = []

        def callback(operation, element_type, elements):
            callback_data.append((operation, element_type, elements))

        knowledge_graph.add_callback(callback)
        non_existent_edge = Edge("connects", "a", "b")
        knowledge_graph.remove_edge(non_existent_edge)

        assert len(callback_data) == 0
