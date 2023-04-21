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


from typing import List
from typing import Union
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from knowledge_graph_msgs.msg import Node as NodeMsg
from knowledge_graph_msgs.msg import Edge as EdgeMsg
from knowledge_graph_msgs.msg import Graph as GraphMsg
from knowledge_graph_msgs.msg import GraphUpdate
from threading import Lock


class KnowledgeGraph:

    _instance: "KnowledgeGraph" = None
    _lock: Lock = Lock()

    @staticmethod
    def get_instance(node: Node):
        with KnowledgeGraph._lock:
            if KnowledgeGraph._instance == None:
                KnowledgeGraph._instance = KnowledgeGraph(node)
            return KnowledgeGraph._instance

    def __init__(self, provided_node: Node) -> None:

        if not KnowledgeGraph._instance is None:
            raise Exception("This class is a Singleton")

        self.node = provided_node

        self.graph = GraphMsg()
        self.graph_id = self.node.get_name()

        self.update_pub = self.node.create_publisher(
            GraphUpdate, "graph_update", QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RELIABLE
            ))
        self.update_sub = self.node.create_subscription(
            GraphUpdate, "graph_update", self.update_callback, QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RELIABLE
            ))

        self.last_ts = self.node.get_clock().now()
        self.start_time = self.node.get_clock().now()

        self.reqsync_timer = self.node.create_timer(
            0.1, self.reqsync_timer_callback)
        self.reqsync_timer_callback()

    def reqsync_timer_callback(self) -> None:
        if Time(nanoseconds=(self.node.get_clock().now() - self.start_time).nanoseconds).seconds_nanoseconds()[0] > 1.0:
            self.reqsync_timer.cancel()

        hello_msg = GraphUpdate()
        hello_msg.stamp = self.node.get_clock().now().to_msg()
        hello_msg.node_id = self.graph_id
        hello_msg.operation_type = GraphUpdate.REQSYNC
        hello_msg.element_type = GraphUpdate.GRAPH
        hello_msg.graph = self.graph

        self.update_pub.publish(hello_msg)

    def publish_update(self, operation_type: int,
                       element: Union[NodeMsg, EdgeMsg, GraphMsg],
                       target_node: str = "") -> None:

        update_msg = GraphUpdate()

        if isinstance(element, NodeMsg):
            update_msg.element_type = GraphUpdate.NODE
            if operation_type == GraphUpdate.REMOVE:
                update_msg.removed_node = element.node_name
            else:
                update_msg.node = element
        elif isinstance(element, EdgeMsg):
            update_msg.element_type = GraphUpdate.EDGE
            update_msg.edge = element
        elif isinstance(element, GraphMsg):
            update_msg.element_type = GraphUpdate.GRAPH
            update_msg.graph = element

        update_msg.stamp = self.node.get_clock().now().to_msg()
        update_msg.node_id = self.graph_id
        update_msg.target_node = target_node
        update_msg.operation_type = operation_type
        self.update_pub.publish(update_msg)

    def remove_node(self, node: str, sync: bool = True) -> bool:
        removed = False

        n = self.get_node(node)
        if not n is None:
            self.graph.nodes.remove(n)
            removed = True

        if removed:
            for e in self.graph.edges:
                if e.source_node == node or e.target_node == node:
                    self.graph.edges.remove(e)

        if removed:
            if sync:
                self.publish_update(GraphUpdate.REMOVE, node)

            self.last_ts = self.node.get_clock().now()

        return removed

    def exist_node(self, node: str) -> bool:
        return not self.get_node(node) is None

    def get_node(self, node: str) -> NodeMsg:
        for n in self.graph.nodes:
            if n.node_name == node:
                return n
        return None

    def get_node_names(self) -> List[str]:
        ret = []
        for n in self.graph.nodes:
            ret.append(n.node_name)
        return ret

    def remove_edge(self, edge: EdgeMsg, sync: bool = True) -> bool:
        removed = False

        for e in self.graph.edges:
            if (e.source_node == edge.source_node and
                e.target_node == edge.target_node and
                    e.edge_class == edge.edge_class):

                self.graph.edges.remove(e)
                removed = True
                break

        if removed:
            if sync:
                self.publish_update(GraphUpdate.REMOVE, edge)

            self.last_ts = self.node.get_clock().now()

        return removed

    def get_nodes(self) -> List[Node]:
        return self.graph.nodes

    def get_edges(self,
                  source: str = None,
                  target: str = None,
                  edge_class: str = None) -> List[EdgeMsg]:

        if (source is None or target is None) and edge_class is None:
            return self.graph.edges

        ret = []
        for e in self.graph.edges:

            if not edge_class is None:
                if e.edge_class == edge_class:
                    ret.append(e)

            else:
                if e.source_node == source and e.target_node == target:
                    ret.append(e)
        return ret

    def get_out_edges(self, source: str) -> List[EdgeMsg]:

        ret = []
        for e in self.graph.edges:
            if e.source_node == source:
                ret.append(e)
        return ret

    def get_in_edges(self, target: str) -> List[EdgeMsg]:

        ret = []
        for e in self.graph.edges:
            if e.target_node == target:
                ret.append(e)
        return ret

    def get_num_edges(self) -> int:
        return len(self.graph.edges)

    def get_num_nodes(self) -> int:
        return len(self.graph.nodes)

    def update_node(self, node: NodeMsg, sync: bool = True) -> bool:

        node_msg = None
        idx = -1

        for idx, n in enumerate(self.graph.nodes):
            if n.node_name == node.node_name:
                node_msg = n
                break

        if node_msg is None:
            self.graph.nodes.append(node)
        else:
            self.graph.nodes[idx] = node

        if sync:
            self.publish_update(GraphUpdate.UPDATE, node)

        self.last_ts = self.node.get_clock().now()

        return True

    def update_edge(self, edge: EdgeMsg, sync: bool = True) -> bool:
        if not self.exist_node(edge.source_node):
            self.node.get_logger().error(
                f"Node source [{edge.source_node}] doesn't exist adding edge")
            return False

        if not self.exist_node(edge.target_node):
            self.node.get_logger().error(
                f"Node source [{edge.target_node}] doesn't exist adding edge")
            return False

        found = False
        for idx, e in enumerate(self.graph.edges):
            if (e.source_node == edge.source_node and
                e.target_node == edge.target_node and
                    e.edge_class == edge.edge_class):

                self.graph.edges[idx] = edge
                found = True
                break

        if not found:
            self.graph.edges.append(edge)

        if sync:
            self.publish_update(GraphUpdate.UPDATE, edge)
        self.last_ts = self.node.get_clock().now()

        return True

    def update_callback(self, msg: GraphUpdate) -> None:
        author_id = msg.node_id
        element = msg.element_type
        operation = msg.operation_type
        ts = Time.from_msg(msg.stamp)

        if author_id == self.graph_id:
            return

        if (ts < self.last_ts and
                operation != GraphUpdate.REQSYNC and operation != GraphUpdate.SYNC):
            self.node.get_logger().error(
                f"UNORDERER UPDATE [{operation}] {self.last_ts.seconds_nanoseconds()[0]} > {ts.seconds_nanoseconds()[0]}")

        if element == GraphUpdate.NODE:
            if author_id == self.graph_id:
                return

            if operation == GraphUpdate.UPDATE:
                self.update_node(msg.node, False)

            elif operation == GraphUpdate.REMOVE:
                self.remove_node(msg.removed_node, False)

        elif element == GraphUpdate.EDGE:
            if author_id == self.graph_id:
                return

            if operation == GraphUpdate.UPDATE:
                self.update_edge(msg.edge, False)

            elif operation == GraphUpdate.REMOVE:
                self.remove_edge(msg.edge, False)

        elif element == GraphUpdate.GRAPH:
            if operation == GraphUpdate.SYNC:
                if msg.target_node == self.graph_id:
                    self.reqsync_timer.cancel()
                    # self.graph = msg.graph

                    for n in msg.graph.nodes:
                        self.update_node(n, False)

                    for e in msg.graph.edges:
                        self.update_edge(e, False)

                    self.last_ts = ts

            elif operation == GraphUpdate.REQSYNC:
                if msg.node_id != self.graph_id:
                    self.publish_update(
                        GraphUpdate.SYNC, self.graph, msg.node_id)
