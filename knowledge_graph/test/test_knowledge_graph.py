# Copyright 2023 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import rclpy
from rclpy.node import Node

from knowledge_graph.knowledge_graph import KnowledgeGraph
from knowledge_graph.graph_utils import new_node, new_edge, add_property, get_property


class TestKnowledgeGraph(unittest.TestCase):
    """Test fixture for KnowledgeGraph tests."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Reset singleton for each test
        KnowledgeGraph._instance = None
        self.node = Node("test_node")
        self.graph = KnowledgeGraph(self.node)

    def tearDown(self):
        self.graph = None
        KnowledgeGraph._instance = None
        self.node.destroy_node()

    # =========================================================================
    # Node Operations Tests
    # =========================================================================

    def test_initial_graph_is_empty(self):
        self.assertEqual(self.graph.get_num_nodes(), 0)
        self.assertEqual(self.graph.get_num_edges(), 0)
        self.assertEqual(len(self.graph.get_nodes()), 0)
        self.assertEqual(len(self.graph.get_edges()), 0)

    def test_add_single_node(self):
        robot = new_node("robot1", "robot")
        result = self.graph.update_node(robot, False)

        self.assertTrue(result)
        self.assertEqual(self.graph.get_num_nodes(), 1)
        self.assertTrue(self.graph.exist_node("robot1"))

    def test_add_multiple_nodes(self):
        self.graph.update_node(new_node("robot1", "robot"), False)
        self.graph.update_node(new_node("room1", "room"), False)
        self.graph.update_node(new_node("object1", "object"), False)

        self.assertEqual(self.graph.get_num_nodes(), 3)
        self.assertTrue(self.graph.exist_node("robot1"))
        self.assertTrue(self.graph.exist_node("room1"))
        self.assertTrue(self.graph.exist_node("object1"))

    def test_update_existing_node(self):
        robot = new_node("robot1", "robot")
        add_property(robot, "speed", 10)
        self.graph.update_node(robot, False)

        # Update with new properties
        robot_updated = new_node("robot1", "robot")
        add_property(robot_updated, "speed", 20)
        self.graph.update_node(robot_updated, False)

        self.assertEqual(self.graph.get_num_nodes(), 1)

        retrieved = self.graph.get_node("robot1")
        self.assertIsNotNone(retrieved)

        speed = get_property(retrieved, "speed")
        self.assertIsNotNone(speed)
        self.assertEqual(speed, 20)

    def test_get_node_that_exists(self):
        robot = new_node("robot1", "robot")
        add_property(robot, "model", "TurtleBot")
        self.graph.update_node(robot, False)

        retrieved = self.graph.get_node("robot1")

        self.assertIsNotNone(retrieved)
        self.assertEqual(retrieved.node_name, "robot1")
        self.assertEqual(retrieved.node_class, "robot")

        model = get_property(retrieved, "model")
        self.assertIsNotNone(model)
        self.assertEqual(model, "TurtleBot")

    def test_get_node_that_does_not_exist(self):
        retrieved = self.graph.get_node("nonexistent")

        self.assertIsNone(retrieved)

    def test_exist_node_positive(self):
        self.graph.update_node(new_node("robot1", "robot"), False)

        self.assertTrue(self.graph.exist_node("robot1"))

    def test_exist_node_negative(self):
        self.assertFalse(self.graph.exist_node("nonexistent"))

    def test_remove_existing_node(self):
        self.graph.update_node(new_node("robot1", "robot"), False)
        self.assertTrue(self.graph.exist_node("robot1"))

        result = self.graph.remove_node("robot1", False)

        self.assertTrue(result)
        self.assertFalse(self.graph.exist_node("robot1"))
        self.assertEqual(self.graph.get_num_nodes(), 0)

    def test_remove_non_existing_node(self):
        result = self.graph.remove_node("nonexistent", False)

        self.assertFalse(result)

    def test_remove_node_removes_connected_edges(self):
        # Create nodes
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)
        self.graph.update_node(new_node("node_c", "type"), False)

        # Create edges
        self.graph.update_edge(new_edge("connects", "node_a", "node_b"), False)
        self.graph.update_edge(new_edge("connects", "node_b", "node_c"), False)
        self.graph.update_edge(new_edge("connects", "node_a", "node_c"), False)

        self.assertEqual(self.graph.get_num_edges(), 3)

        # Remove node_b
        self.graph.remove_node("node_b", False)

        # Only edge from node_a to node_c should remain
        self.assertEqual(self.graph.get_num_edges(), 1)
        self.assertEqual(self.graph.get_num_nodes(), 2)

        remaining_edges = self.graph.get_edges("node_a", "node_c")
        self.assertEqual(len(remaining_edges), 1)

    def test_get_node_names(self):
        self.graph.update_node(new_node("alpha", "type"), False)
        self.graph.update_node(new_node("beta", "type"), False)
        self.graph.update_node(new_node("gamma", "type"), False)

        names = self.graph.get_node_names()

        self.assertEqual(len(names), 3)
        self.assertIn("alpha", names)
        self.assertIn("beta", names)
        self.assertIn("gamma", names)

    def test_get_all_nodes(self):
        self.graph.update_node(new_node("node1", "type_a"), False)
        self.graph.update_node(new_node("node2", "type_b"), False)

        nodes = self.graph.get_nodes()

        self.assertEqual(len(nodes), 2)

    # =========================================================================
    # Edge Operations Tests
    # =========================================================================

    def test_add_edge_between_existing_nodes(self):
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph.update_edge(edge, False)

        self.assertTrue(result)
        self.assertEqual(self.graph.get_num_edges(), 1)

    def test_add_edge_with_non_existent_source(self):
        self.graph.update_node(new_node("node_b", "type"), False)

        edge = new_edge("connects", "nonexistent", "node_b")
        result = self.graph.update_edge(edge, False)

        self.assertFalse(result)
        self.assertEqual(self.graph.get_num_edges(), 0)

    def test_add_edge_with_non_existent_target(self):
        self.graph.update_node(new_node("node_a", "type"), False)

        edge = new_edge("connects", "node_a", "nonexistent")
        result = self.graph.update_edge(edge, False)

        self.assertFalse(result)
        self.assertEqual(self.graph.get_num_edges(), 0)

    def test_add_multiple_edges_same_nodes(self):
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)

        self.graph.update_edge(new_edge("edge_type_1", "node_a", "node_b"), False)
        self.graph.update_edge(new_edge("edge_type_2", "node_a", "node_b"), False)

        self.assertEqual(self.graph.get_num_edges(), 2)

        edges = self.graph.get_edges("node_a", "node_b")
        self.assertEqual(len(edges), 2)

    def test_update_existing_edge(self):
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)

        edge1 = new_edge("connects", "node_a", "node_b")
        add_property(edge1, "weight", 10)
        self.graph.update_edge(edge1, False)

        edge2 = new_edge("connects", "node_a", "node_b")
        add_property(edge2, "weight", 20)
        self.graph.update_edge(edge2, False)

        self.assertEqual(self.graph.get_num_edges(), 1)

        edges = self.graph.get_edges("node_a", "node_b")
        self.assertEqual(len(edges), 1)

        weight = get_property(edges[0], "weight")
        self.assertIsNotNone(weight)
        self.assertEqual(weight, 20)

    def test_remove_existing_edge(self):
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)
        self.graph.update_edge(new_edge("connects", "node_a", "node_b"), False)

        self.assertEqual(self.graph.get_num_edges(), 1)

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph.remove_edge(edge, False)

        self.assertTrue(result)
        self.assertEqual(self.graph.get_num_edges(), 0)

    def test_remove_non_existing_edge(self):
        self.graph.update_node(new_node("node_a", "type"), False)
        self.graph.update_node(new_node("node_b", "type"), False)

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph.remove_edge(edge, False)

        self.assertFalse(result)

    def test_get_edges_by_source_and_target(self):
        self.graph.update_node(new_node("a", "type"), False)
        self.graph.update_node(new_node("b", "type"), False)
        self.graph.update_node(new_node("c", "type"), False)

        self.graph.update_edge(new_edge("e1", "a", "b"), False)
        self.graph.update_edge(new_edge("e2", "a", "b"), False)
        self.graph.update_edge(new_edge("e3", "a", "c"), False)

        edges_ab = self.graph.get_edges("a", "b")
        edges_ac = self.graph.get_edges("a", "c")
        edges_bc = self.graph.get_edges("b", "c")

        self.assertEqual(len(edges_ab), 2)
        self.assertEqual(len(edges_ac), 1)
        self.assertEqual(len(edges_bc), 0)

    def test_get_edges_by_class(self):
        self.graph.update_node(new_node("a", "type"), False)
        self.graph.update_node(new_node("b", "type"), False)
        self.graph.update_node(new_node("c", "type"), False)

        self.graph.update_edge(new_edge("connects", "a", "b"), False)
        self.graph.update_edge(new_edge("connects", "b", "c"), False)
        self.graph.update_edge(new_edge("contains", "a", "c"), False)

        connects_edges = self.graph.get_edges(edge_class="connects")
        contains_edges = self.graph.get_edges(edge_class="contains")
        other_edges = self.graph.get_edges(edge_class="other")

        self.assertEqual(len(connects_edges), 2)
        self.assertEqual(len(contains_edges), 1)
        self.assertEqual(len(other_edges), 0)

    def test_get_out_edges(self):
        self.graph.update_node(new_node("center", "type"), False)
        self.graph.update_node(new_node("target1", "type"), False)
        self.graph.update_node(new_node("target2", "type"), False)
        self.graph.update_node(new_node("source", "type"), False)

        self.graph.update_edge(new_edge("out1", "center", "target1"), False)
        self.graph.update_edge(new_edge("out2", "center", "target2"), False)
        self.graph.update_edge(new_edge("in1", "source", "center"), False)

        out_edges = self.graph.get_out_edges("center")

        self.assertEqual(len(out_edges), 2)
        for edge in out_edges:
            self.assertEqual(edge.source_node, "center")

    def test_get_in_edges(self):
        self.graph.update_node(new_node("center", "type"), False)
        self.graph.update_node(new_node("source1", "type"), False)
        self.graph.update_node(new_node("source2", "type"), False)
        self.graph.update_node(new_node("target", "type"), False)

        self.graph.update_edge(new_edge("in1", "source1", "center"), False)
        self.graph.update_edge(new_edge("in2", "source2", "center"), False)
        self.graph.update_edge(new_edge("out1", "center", "target"), False)

        in_edges = self.graph.get_in_edges("center")

        self.assertEqual(len(in_edges), 2)
        for edge in in_edges:
            self.assertEqual(edge.target_node, "center")

    def test_get_all_edges(self):
        self.graph.update_node(new_node("a", "type"), False)
        self.graph.update_node(new_node("b", "type"), False)

        self.graph.update_edge(new_edge("e1", "a", "b"), False)
        self.graph.update_edge(new_edge("e2", "b", "a"), False)

        all_edges = self.graph.get_edges()

        self.assertEqual(len(all_edges), 2)

    def test_self_referencing_edge(self):
        self.graph.update_node(new_node("node", "type"), False)

        edge = new_edge("self_loop", "node", "node")
        result = self.graph.update_edge(edge, False)

        self.assertTrue(result)
        self.assertEqual(self.graph.get_num_edges(), 1)

        out_edges = self.graph.get_out_edges("node")
        in_edges = self.graph.get_in_edges("node")

        self.assertEqual(len(out_edges), 1)
        self.assertEqual(len(in_edges), 1)

    # =========================================================================
    # Graph Size Tests
    # =========================================================================

    def test_get_num_nodes_after_operations(self):
        self.assertEqual(self.graph.get_num_nodes(), 0)

        self.graph.update_node(new_node("n1", "type"), False)
        self.assertEqual(self.graph.get_num_nodes(), 1)

        self.graph.update_node(new_node("n2", "type"), False)
        self.assertEqual(self.graph.get_num_nodes(), 2)

        self.graph.remove_node("n1", False)
        self.assertEqual(self.graph.get_num_nodes(), 1)

        self.graph.remove_node("n2", False)
        self.assertEqual(self.graph.get_num_nodes(), 0)

    def test_get_num_edges_after_operations(self):
        self.graph.update_node(new_node("a", "type"), False)
        self.graph.update_node(new_node("b", "type"), False)

        self.assertEqual(self.graph.get_num_edges(), 0)

        self.graph.update_edge(new_edge("e1", "a", "b"), False)
        self.assertEqual(self.graph.get_num_edges(), 1)

        self.graph.update_edge(new_edge("e2", "a", "b"), False)
        self.assertEqual(self.graph.get_num_edges(), 2)

        self.graph.remove_edge(new_edge("e1", "a", "b"), False)
        self.assertEqual(self.graph.get_num_edges(), 1)

    # =========================================================================
    # Complex Graph Tests
    # =========================================================================

    def test_build_complex_graph(self):
        # Create a more complex graph structure
        # Rooms
        self.graph.update_node(new_node("living_room", "room"), False)
        self.graph.update_node(new_node("kitchen", "room"), False)
        self.graph.update_node(new_node("bedroom", "room"), False)

        # Objects
        self.graph.update_node(new_node("table", "object"), False)
        self.graph.update_node(new_node("chair", "object"), False)
        self.graph.update_node(new_node("fridge", "object"), False)

        # Robot
        robot = new_node("turtlebot", "robot")
        add_property(robot, "battery", 85.5)
        add_property(robot, "status", "idle")
        self.graph.update_node(robot, False)

        # Room connections
        self.graph.update_edge(new_edge("connected_to", "living_room", "kitchen"), False)
        self.graph.update_edge(new_edge("connected_to", "living_room", "bedroom"), False)

        # Object locations
        self.graph.update_edge(new_edge("is_in", "table", "living_room"), False)
        self.graph.update_edge(new_edge("is_in", "chair", "living_room"), False)
        self.graph.update_edge(new_edge("is_in", "fridge", "kitchen"), False)

        # Robot location
        self.graph.update_edge(new_edge("is_at", "turtlebot", "living_room"), False)

        # Verify structure
        self.assertEqual(self.graph.get_num_nodes(), 7)
        self.assertEqual(self.graph.get_num_edges(), 6)

        # Check robot properties
        retrieved_robot = self.graph.get_node("turtlebot")
        self.assertIsNotNone(retrieved_robot)

        battery = get_property(retrieved_robot, "battery")
        self.assertIsNotNone(battery)
        self.assertAlmostEqual(battery, 85.5)

        # Check connections from living room
        living_room_connections = self.graph.get_out_edges("living_room")
        self.assertEqual(len(living_room_connections), 2)

        # Check objects in living room
        objects_in_living = self.graph.get_in_edges("living_room")
        self.assertGreaterEqual(len(objects_in_living), 2)  # table, chair, and robot


if __name__ == "__main__":
    unittest.main()
