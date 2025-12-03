# Copyright 2025 Miguel Ángel González Santamarta
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
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

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
        self.node1 = Node("test_node1")
        self.node2 = Node("test_node2")
        self.graph1 = KnowledgeGraph(self.node1)
        self.graph2 = KnowledgeGraph(self.node2)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node1)
        self.executor.add_node(self.node2)

        self.spinning = True
        self.spin_thread = threading.Thread(target=self._spin_executor)
        self.spin_thread.start()

    def tearDown(self):
        self.spinning = False
        if self.spin_thread.is_alive():
            self.spin_thread.join()
        self.executor.shutdown()
        self.graph1 = None
        self.graph2 = None
        self.node1.destroy_node()
        self.node2.destroy_node()

    def _spin_executor(self):
        while self.spinning and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.01)

    # =========================================================================
    # Node Operations Tests
    # =========================================================================

    def test_initial_graph_is_empty(self):
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_nodes(), 0)
        self.assertEqual(self.graph1.get_num_edges(), 0)
        self.assertEqual(len(self.graph1.get_nodes()), 0)
        self.assertEqual(len(self.graph1.get_edges()), 0)
        self.assertEqual(self.graph2.get_num_nodes(), 0)
        self.assertEqual(self.graph2.get_num_edges(), 0)
        self.assertEqual(len(self.graph2.get_nodes()), 0)
        self.assertEqual(len(self.graph2.get_edges()), 0)

    def test_add_single_node(self):
        robot = new_node("robot1", "robot")
        result = self.graph1.update_node(robot, True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(result)
        self.assertEqual(self.graph1.get_num_nodes(), 1)
        self.assertTrue(self.graph1.exist_node("robot1"))
        self.assertEqual(self.graph2.get_num_nodes(), 1)
        self.assertTrue(self.graph2.exist_node("robot1"))

    def test_add_multiple_nodes(self):
        self.graph1.update_node(new_node("robot1", "robot"), True)
        self.graph1.update_node(new_node("room1", "room"), True)
        self.graph1.update_node(new_node("object1", "object"), True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_nodes(), 3)
        self.assertTrue(self.graph1.exist_node("robot1"))
        self.assertTrue(self.graph1.exist_node("room1"))
        self.assertTrue(self.graph1.exist_node("object1"))
        self.assertEqual(self.graph2.get_num_nodes(), 3)
        self.assertTrue(self.graph2.exist_node("robot1"))
        self.assertTrue(self.graph2.exist_node("room1"))
        self.assertTrue(self.graph2.exist_node("object1"))

    def test_update_existing_node(self):
        robot = new_node("robot1", "robot")
        add_property(robot, "speed", 10)
        self.graph1.update_node(robot, True)

        # Update with new properties
        robot_updated = new_node("robot1", "robot")
        add_property(robot_updated, "speed", 20)
        self.graph1.update_node(robot_updated, True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_nodes(), 1)

        retrieved = self.graph1.get_node("robot1")
        self.assertIsNotNone(retrieved)

        speed = get_property(retrieved, "speed")
        self.assertIsNotNone(speed)
        self.assertEqual(speed, 20)

        retrieved2 = self.graph2.get_node("robot1")
        self.assertIsNotNone(retrieved2)

        speed2 = get_property(retrieved2, "speed")
        self.assertIsNotNone(speed2)
        self.assertEqual(speed2, 20)

    def test_get_node_that_exists(self):
        robot = new_node("robot1", "robot")
        add_property(robot, "model", "TurtleBot")
        self.graph1.update_node(robot, True)

        time.sleep(0.05)  # Allow sync

        retrieved = self.graph1.get_node("robot1")

        self.assertIsNotNone(retrieved)
        self.assertEqual(retrieved.node_name, "robot1")
        self.assertEqual(retrieved.node_class, "robot")

        model = get_property(retrieved, "model")
        self.assertIsNotNone(model)
        self.assertEqual(model, "TurtleBot")

        retrieved2 = self.graph2.get_node("robot1")

        self.assertIsNotNone(retrieved2)
        self.assertEqual(retrieved2.node_name, "robot1")
        self.assertEqual(retrieved2.node_class, "robot")

        model2 = get_property(retrieved2, "model")
        self.assertIsNotNone(model2)
        self.assertEqual(model2, "TurtleBot")

    def test_get_node_that_does_not_exist(self):
        time.sleep(0.05)  # Allow sync
        retrieved = self.graph1.get_node("nonexistent")

        self.assertIsNone(retrieved)

        retrieved2 = self.graph2.get_node("nonexistent")

        self.assertIsNone(retrieved2)

    def test_exist_node_positive(self):
        self.graph1.update_node(new_node("robot1", "robot"), True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(self.graph1.exist_node("robot1"))
        self.assertTrue(self.graph2.exist_node("robot1"))

    def test_exist_node_negative(self):
        time.sleep(0.05)  # Allow sync
        self.assertFalse(self.graph1.exist_node("nonexistent"))
        self.assertFalse(self.graph2.exist_node("nonexistent"))

    def test_remove_existing_node(self):
        self.graph1.update_node(new_node("robot1", "robot"), True)
        time.sleep(0.05)  # Allow sync
        self.assertTrue(self.graph1.exist_node("robot1"))
        self.assertTrue(self.graph2.exist_node("robot1"))

        result = self.graph1.remove_node("robot1", True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(result)
        self.assertFalse(self.graph1.exist_node("robot1"))
        self.assertEqual(self.graph1.get_num_nodes(), 0)
        self.assertFalse(self.graph2.exist_node("robot1"))
        self.assertEqual(self.graph2.get_num_nodes(), 0)

    def test_remove_non_existing_node(self):
        result = self.graph1.remove_node("nonexistent", True)

        self.assertFalse(result)

    def test_remove_node_removes_connected_edges(self):
        # Create nodes
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)
        self.graph1.update_node(new_node("node_c", "type"), True)

        # Create edges
        self.graph1.update_edge(new_edge("connects", "node_a", "node_b"), True)
        self.graph1.update_edge(new_edge("connects", "node_b", "node_c"), True)
        self.graph1.update_edge(new_edge("connects", "node_a", "node_c"), True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_edges(), 3)
        self.assertEqual(self.graph2.get_num_edges(), 3)

        # Remove node_b
        self.graph1.remove_node("node_b", True)

        time.sleep(0.05)  # Allow sync

        # Only edge from node_a to node_c should remain
        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph1.get_num_nodes(), 2)
        self.assertEqual(self.graph2.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_nodes(), 2)

        remaining_edges = self.graph1.get_edges("node_a", "node_c")
        self.assertEqual(len(remaining_edges), 1)
        remaining_edges2 = self.graph2.get_edges("node_a", "node_c")
        self.assertEqual(len(remaining_edges2), 1)

    def test_get_node_names(self):
        self.graph1.update_node(new_node("alpha", "type"), True)
        self.graph1.update_node(new_node("beta", "type"), True)
        self.graph1.update_node(new_node("gamma", "type"), True)

        time.sleep(0.05)  # Allow sync

        names = self.graph1.get_node_names()

        self.assertEqual(len(names), 3)
        self.assertIn("alpha", names)
        self.assertIn("beta", names)
        self.assertIn("gamma", names)

        names2 = self.graph2.get_node_names()

        self.assertEqual(len(names2), 3)
        self.assertIn("alpha", names2)
        self.assertIn("beta", names2)
        self.assertIn("gamma", names2)

    def test_get_all_nodes(self):
        self.graph1.update_node(new_node("node1", "type_a"), True)
        self.graph1.update_node(new_node("node2", "type_b"), True)

        time.sleep(0.05)  # Allow sync

        nodes = self.graph1.get_nodes()

        self.assertEqual(len(nodes), 2)

        nodes2 = self.graph2.get_nodes()

        self.assertEqual(len(nodes2), 2)

    # =========================================================================
    # Edge Operations Tests
    # =========================================================================

    def test_add_edge_between_existing_nodes(self):
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph1.update_edge(edge, True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(result)
        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

    def test_add_edge_with_non_existent_source(self):
        self.graph1.update_node(new_node("node_b", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge = new_edge("connects", "nonexistent", "node_b")
        result = self.graph1.update_edge(edge, True)

        self.assertFalse(result)
        self.assertEqual(self.graph1.get_num_edges(), 0)
        self.assertEqual(self.graph2.get_num_edges(), 0)

    def test_add_edge_with_non_existent_target(self):
        self.graph1.update_node(new_node("node_a", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge = new_edge("connects", "node_a", "nonexistent")
        result = self.graph1.update_edge(edge, True)

        self.assertFalse(result)
        self.assertEqual(self.graph1.get_num_edges(), 0)
        self.assertEqual(self.graph2.get_num_edges(), 0)

    def test_add_multiple_edges_same_nodes(self):
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)

        time.sleep(0.05)  # Allow sync

        self.graph1.update_edge(new_edge("edge_type_1", "node_a", "node_b"), True)
        self.graph1.update_edge(new_edge("edge_type_2", "node_a", "node_b"), True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_edges(), 2)
        self.assertEqual(self.graph2.get_num_edges(), 2)

        edges = self.graph1.get_edges("node_a", "node_b")
        self.assertEqual(len(edges), 2)
        edges2 = self.graph2.get_edges("node_a", "node_b")
        self.assertEqual(len(edges2), 2)

    def test_update_existing_edge(self):
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge1 = new_edge("connects", "node_a", "node_b")
        add_property(edge1, "weight", 10)
        self.graph1.update_edge(edge1, True)

        edge2 = new_edge("connects", "node_a", "node_b")
        add_property(edge2, "weight", 20)
        self.graph1.update_edge(edge2, True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

        edges = self.graph1.get_edges("node_a", "node_b")
        self.assertEqual(len(edges), 1)

        weight = get_property(edges[0], "weight")
        self.assertIsNotNone(weight)
        self.assertEqual(weight, 20)

        edges2 = self.graph2.get_edges("node_a", "node_b")
        self.assertEqual(len(edges2), 1)

        weight2 = get_property(edges2[0], "weight")
        self.assertIsNotNone(weight2)
        self.assertEqual(weight2, 20)

    def test_remove_existing_edge(self):
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)
        self.graph1.update_edge(new_edge("connects", "node_a", "node_b"), True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph1.remove_edge(edge, True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(result)
        self.assertEqual(self.graph1.get_num_edges(), 0)
        self.assertEqual(self.graph2.get_num_edges(), 0)

    def test_remove_non_existing_edge(self):
        self.graph1.update_node(new_node("node_a", "type"), True)
        self.graph1.update_node(new_node("node_b", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge = new_edge("connects", "node_a", "node_b")
        result = self.graph1.remove_edge(edge, True)

        self.assertFalse(result)

    def test_get_edges_by_source_and_target(self):
        self.graph1.update_node(new_node("a", "type"), True)
        self.graph1.update_node(new_node("b", "type"), True)
        self.graph1.update_node(new_node("c", "type"), True)

        self.graph1.update_edge(new_edge("e1", "a", "b"), True)
        self.graph1.update_edge(new_edge("e2", "a", "b"), True)
        self.graph1.update_edge(new_edge("e3", "a", "c"), True)

        time.sleep(0.05)  # Allow sync

        edges_ab = self.graph1.get_edges("a", "b")
        edges_ac = self.graph1.get_edges("a", "c")
        edges_bc = self.graph1.get_edges("b", "c")

        self.assertEqual(len(edges_ab), 2)
        self.assertEqual(len(edges_ac), 1)
        self.assertEqual(len(edges_bc), 0)

        edges_ab2 = self.graph2.get_edges("a", "b")
        edges_ac2 = self.graph2.get_edges("a", "c")
        edges_bc2 = self.graph2.get_edges("b", "c")

        self.assertEqual(len(edges_ab2), 2)
        self.assertEqual(len(edges_ac2), 1)
        self.assertEqual(len(edges_bc2), 0)

    def test_get_edges_by_class(self):
        self.graph1.update_node(new_node("a", "type"), True)
        self.graph1.update_node(new_node("b", "type"), True)
        self.graph1.update_node(new_node("c", "type"), True)

        self.graph1.update_edge(new_edge("connects", "a", "b"), True)
        self.graph1.update_edge(new_edge("connects", "b", "c"), True)
        self.graph1.update_edge(new_edge("contains", "a", "c"), True)

        time.sleep(0.05)  # Allow sync

        connects_edges = self.graph1.get_edges(edge_class="connects")
        contains_edges = self.graph1.get_edges(edge_class="contains")
        other_edges = self.graph1.get_edges(edge_class="other")

        self.assertEqual(len(connects_edges), 2)
        self.assertEqual(len(contains_edges), 1)
        self.assertEqual(len(other_edges), 0)

        connects_edges2 = self.graph2.get_edges(edge_class="connects")
        contains_edges2 = self.graph2.get_edges(edge_class="contains")
        other_edges2 = self.graph2.get_edges(edge_class="other")

        self.assertEqual(len(connects_edges2), 2)
        self.assertEqual(len(contains_edges2), 1)
        self.assertEqual(len(other_edges2), 0)

    def test_get_out_edges(self):
        self.graph1.update_node(new_node("center", "type"), True)
        self.graph1.update_node(new_node("target1", "type"), True)
        self.graph1.update_node(new_node("target2", "type"), True)
        self.graph1.update_node(new_node("source", "type"), True)

        self.graph1.update_edge(new_edge("out1", "center", "target1"), True)
        self.graph1.update_edge(new_edge("out2", "center", "target2"), True)
        self.graph1.update_edge(new_edge("in1", "source", "center"), True)

        time.sleep(0.05)  # Allow sync

        out_edges = self.graph1.get_out_edges("center")

        self.assertEqual(len(out_edges), 2)
        for edge in out_edges:
            self.assertEqual(edge.source_node, "center")

        out_edges2 = self.graph2.get_out_edges("center")

        self.assertEqual(len(out_edges2), 2)
        for edge in out_edges2:
            self.assertEqual(edge.source_node, "center")

    def test_get_in_edges(self):
        self.graph1.update_node(new_node("center", "type"), True)
        self.graph1.update_node(new_node("source1", "type"), True)
        self.graph1.update_node(new_node("source2", "type"), True)
        self.graph1.update_node(new_node("target", "type"), True)

        self.graph1.update_edge(new_edge("in1", "source1", "center"), True)
        self.graph1.update_edge(new_edge("in2", "source2", "center"), True)
        self.graph1.update_edge(new_edge("out1", "center", "target"), True)

        time.sleep(0.05)  # Allow sync

        in_edges = self.graph1.get_in_edges("center")

        self.assertEqual(len(in_edges), 2)
        for edge in in_edges:
            self.assertEqual(edge.target_node, "center")

        in_edges2 = self.graph2.get_in_edges("center")

        self.assertEqual(len(in_edges2), 2)
        for edge in in_edges2:
            self.assertEqual(edge.target_node, "center")

    def test_get_all_edges(self):
        self.graph1.update_node(new_node("a", "type"), True)
        self.graph1.update_node(new_node("b", "type"), True)

        self.graph1.update_edge(new_edge("e1", "a", "b"), True)
        self.graph1.update_edge(new_edge("e2", "b", "a"), True)

        time.sleep(0.05)  # Allow sync

        all_edges = self.graph1.get_edges()

        self.assertEqual(len(all_edges), 2)

        all_edges2 = self.graph2.get_edges()

        self.assertEqual(len(all_edges2), 2)

    def test_self_referencing_edge(self):
        self.graph1.update_node(new_node("node", "type"), True)

        time.sleep(0.05)  # Allow sync

        edge = new_edge("self_loop", "node", "node")
        result = self.graph1.update_edge(edge, True)

        time.sleep(0.05)  # Allow sync

        self.assertTrue(result)
        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

        out_edges = self.graph1.get_out_edges("node")
        in_edges = self.graph1.get_in_edges("node")

        self.assertEqual(len(out_edges), 1)
        self.assertEqual(len(in_edges), 1)

        out_edges2 = self.graph2.get_out_edges("node")
        in_edges2 = self.graph2.get_in_edges("node")

        self.assertEqual(len(out_edges2), 1)
        self.assertEqual(len(in_edges2), 1)

    # =========================================================================
    # Graph Size Tests
    # =========================================================================

    def test_get_num_nodes_after_operations(self):
        self.assertEqual(self.graph1.get_num_nodes(), 0)
        self.assertEqual(self.graph2.get_num_nodes(), 0)

        self.graph1.update_node(new_node("n1", "type"), True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_nodes(), 1)
        self.assertEqual(self.graph2.get_num_nodes(), 1)

        self.graph1.update_node(new_node("n2", "type"), True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_nodes(), 2)
        self.assertEqual(self.graph2.get_num_nodes(), 2)

        self.graph1.remove_node("n1", True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_nodes(), 1)
        self.assertEqual(self.graph2.get_num_nodes(), 1)

        self.graph1.remove_node("n2", True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_nodes(), 0)
        self.assertEqual(self.graph2.get_num_nodes(), 0)

    def test_get_num_edges_after_operations(self):
        self.graph1.update_node(new_node("a", "type"), True)
        self.graph1.update_node(new_node("b", "type"), True)

        time.sleep(0.05)  # Allow sync

        self.assertEqual(self.graph1.get_num_edges(), 0)
        self.assertEqual(self.graph2.get_num_edges(), 0)

        self.graph1.update_edge(new_edge("e1", "a", "b"), True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

        self.graph1.update_edge(new_edge("e2", "a", "b"), True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_edges(), 2)
        self.assertEqual(self.graph2.get_num_edges(), 2)

        self.graph1.remove_edge(new_edge("e1", "a", "b"), True)
        time.sleep(0.05)  # Allow sync
        self.assertEqual(self.graph1.get_num_edges(), 1)
        self.assertEqual(self.graph2.get_num_edges(), 1)

    # =========================================================================
    # Complex Graph Tests
    # =========================================================================

    def test_build_complex_graph(self):
        # Create a more complex graph structure
        # Rooms
        self.graph1.update_node(new_node("living_room", "room"), True)
        self.graph1.update_node(new_node("kitchen", "room"), True)
        self.graph1.update_node(new_node("bedroom", "room"), True)

        # Objects
        self.graph1.update_node(new_node("table", "object"), True)
        self.graph1.update_node(new_node("chair", "object"), True)
        self.graph1.update_node(new_node("fridge", "object"), True)

        # Robot
        robot = new_node("turtlebot", "robot")
        add_property(robot, "battery", 85.5)
        add_property(robot, "status", "idle")
        self.graph1.update_node(robot, True)

        # Room connections
        self.graph1.update_edge(new_edge("connected_to", "living_room", "kitchen"), True)
        self.graph1.update_edge(new_edge("connected_to", "living_room", "bedroom"), True)

        # Object locations
        self.graph1.update_edge(new_edge("is_in", "table", "living_room"), True)
        self.graph1.update_edge(new_edge("is_in", "chair", "living_room"), True)
        self.graph1.update_edge(new_edge("is_in", "fridge", "kitchen"), True)

        # Robot location
        self.graph1.update_edge(new_edge("is_at", "turtlebot", "living_room"), True)

        time.sleep(0.1)  # Allow sync

        # Verify structure
        self.assertEqual(self.graph1.get_num_nodes(), 7)
        self.assertEqual(self.graph1.get_num_edges(), 6)
        self.assertEqual(self.graph2.get_num_nodes(), 7)
        self.assertEqual(self.graph2.get_num_edges(), 6)

        # Check robot properties
        retrieved_robot = self.graph1.get_node("turtlebot")
        self.assertIsNotNone(retrieved_robot)

        battery = get_property(retrieved_robot, "battery")
        self.assertIsNotNone(battery)
        self.assertAlmostEqual(battery, 85.5)

        retrieved_robot2 = self.graph2.get_node("turtlebot")
        self.assertIsNotNone(retrieved_robot2)

        battery2 = get_property(retrieved_robot2, "battery")
        self.assertIsNotNone(battery2)
        self.assertAlmostEqual(battery2, 85.5)

        # Check connections from living room
        living_room_connections = self.graph1.get_out_edges("living_room")
        self.assertEqual(len(living_room_connections), 2)

        living_room_connections2 = self.graph2.get_out_edges("living_room")
        self.assertEqual(len(living_room_connections2), 2)

        # Check objects in living room
        objects_in_living = self.graph1.get_in_edges("living_room")
        self.assertGreaterEqual(len(objects_in_living), 2)  # table, chair, and robot

        objects_in_living2 = self.graph2.get_in_edges("living_room")
        self.assertGreaterEqual(len(objects_in_living2), 2)  # table, chair, and robot


if __name__ == "__main__":
    unittest.main()
