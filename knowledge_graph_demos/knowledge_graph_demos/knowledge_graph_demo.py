#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node

from knowledge_graph import KnowledgeGraph


class KnowledgeGraphDemo(Node):
    def __init__(self):
        super().__init__("knowledge_graph_demo")
        # Initialize the knowledge graph
        self.graph_ = KnowledgeGraph(self)

        # Create instances (nodes)
        self.create_instances()

        # Create predicates (edges)
        self.create_predicates()

        # Print the graph
        self.print_graph()

    def create_instances(self):
        # Create robot instance
        self.graph_.create_node("leia", "robot")

        # Create room instances
        self.graph_.create_node("entrance", "room")
        self.graph_.create_node("kitchen", "room")
        self.graph_.create_node("bedroom", "room")
        self.graph_.create_node("dinning", "room")
        self.graph_.create_node("bathroom", "room")
        self.graph_.create_node("chargingroom", "room")

    def create_predicates(self):
        # Connected predicates (bidirectional)
        self.graph_.create_edge("connected", "entrance", "dinning")
        self.graph_.create_edge("connected", "dinning", "entrance")

        self.graph_.create_edge("connected", "dinning", "kitchen")
        self.graph_.create_edge("connected", "kitchen", "dinning")

        self.graph_.create_edge("connected", "dinning", "bedroom")
        self.graph_.create_edge("connected", "bedroom", "dinning")

        self.graph_.create_edge("connected", "bathroom", "bedroom")
        self.graph_.create_edge("connected", "bedroom", "bathroom")

        self.graph_.create_edge("connected", "chargingroom", "kitchen")
        self.graph_.create_edge("connected", "kitchen", "chargingroom")

        # Other predicates
        self.graph_.create_edge("charging_point_at", "chargingroom", "chargingroom")
        self.graph_.create_edge("battery_low", "leia", "leia")

        self.graph_.create_edge("robot_at", "leia", "entrance")

        # Goal predicate
        goal_edge = self.graph_.create_edge("robot_at", "leia", "bathroom")
        goal_edge.set_property("is_goal", True)
        self.graph_.update_edge(goal_edge)

    def print_graph(self):
        self.get_logger().info("Knowledge Graph Demo")
        self.get_logger().info("====================")

        nodes = self.graph_.get_nodes()
        self.get_logger().info(f"Nodes ({len(nodes)}):")
        for node in nodes:
            self.get_logger().info(f"  {node.to_string()}")

        edges = self.graph_.get_edges()
        self.get_logger().info(f"Edges ({len(edges)}):")
        for edge in edges:
            self.get_logger().info(f"  {edge.to_string()}")


def main(args=None):
    rclpy.init(args=args)
    node = KnowledgeGraphDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
