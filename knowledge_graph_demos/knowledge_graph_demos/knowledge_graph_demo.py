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
from knowledge_graph import KnowledgeGraph


def main():
    # Initialize rclpy
    rclpy.init()

    # Initialize the knowledge graph
    graph = KnowledgeGraph.get_instance()

    # Create robot instance
    graph.create_node("leia", "robot")

    # Create room instances
    graph.create_node("entrance", "room")
    graph.create_node("kitchen", "room")
    graph.create_node("bedroom", "room")
    graph.create_node("dinning", "room")
    graph.create_node("bathroom", "room")
    graph.create_node("chargingroom", "room")

    # Connected predicates (bidirectional)
    graph.create_edge("connected", "entrance", "dinning")
    graph.create_edge("connected", "dinning", "entrance")

    graph.create_edge("connected", "dinning", "kitchen")
    graph.create_edge("connected", "kitchen", "dinning")

    graph.create_edge("connected", "dinning", "bedroom")
    graph.create_edge("connected", "bedroom", "dinning")

    graph.create_edge("connected", "bathroom", "bedroom")
    graph.create_edge("connected", "bedroom", "bathroom")

    graph.create_edge("connected", "chargingroom", "kitchen")
    graph.create_edge("connected", "kitchen", "chargingroom")

    # Other predicates
    graph.create_edge("charging_point_at", "chargingroom", "chargingroom")
    graph.create_edge("battery_low", "leia", "leia")

    graph.create_edge("robot_at", "leia", "entrance")

    # Goal predicate
    goal_edge = graph.create_edge("robot_at", "leia", "bathroom")
    goal_edge.set_property("is_goal", True)
    graph.update_edge(goal_edge)

    # Print the graph
    print("Knowledge Graph Demo")
    print("====================")

    nodes = graph.get_nodes()
    print(f"Nodes ({len(nodes)}):")
    for node in nodes:
        print(f"  {node.to_string()}")

    edges = graph.get_edges()
    print(f"Edges ({len(edges)}):")
    for edge in edges:
        print(f"  {edge.to_string()}")

    # Shutdown rclpy
    rclpy.shutdown()


if __name__ == "__main__":
    main()
