#!/usr/bin/env python3

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

import json
from typing import List

import rclpy
from rclpy.node import Node as ROSNode

from knowledge_graph import KnowledgeGraph
from knowledge_graph.graph import Node, Edge
from knowledge_graph_msgs.msg import Node as NodeMsg
from knowledge_graph_msgs.msg import Edge as EdgeMsg
from knowledge_graph_msgs.msg import Content
from knowledge_graph_msgs.msg import Property

import sqlite3
from sqlite3 import Error


class KnowledgeGraphDbNode(ROSNode):

    def __init__(self) -> None:
        super().__init__("knowledge_graph_db")

        self.declare_parameter("db_file", "knowledge_graph.db")
        db_file = self.get_parameter("db_file").get_parameter_value().string_value

        self.graph = KnowledgeGraph(self)

        self.sqlite_conn = self.create_connection(db_file)
        self.create_tables()
        self.load_db()

        self.graph.update_node = self.update_node
        self.graph.remove_node = self.remove_node
        self.graph.update_edge = self.update_edge
        self.graph.remove_edge = self.remove_edge

        self.graph._reqsync_timer_callback()
        self.get_logger().info("Knowledge Graph DB Node Started")

    def create_connection(self, db_file: str) -> sqlite3.Connection:
        try:
            return sqlite3.connect(db_file)
        except Error as e:
            self.get_logger().error(str(e))
            return None

    def load_db(self) -> None:
        c = self.sqlite_conn.cursor()

        c.execute("SELECT * FROM nodes")
        nodes_row = c.fetchall()
        for nr in nodes_row:
            node_msg = NodeMsg()

            node_msg.name = nr[0]
            node_msg.type = nr[1]
            node_msg.properties = self.parse_str_properties(nr[2])

            node = Node(msg=node_msg)
            self.graph.update_node(node)

        c.execute("SELECT * FROM edges")
        edges_rows = c.fetchall()
        for er in edges_rows:
            edge_msg = EdgeMsg()

            edge_msg.type = er[0]
            edge_msg.source_node = er[1]
            edge_msg.target_node = er[2]
            edge_msg.properties = self.parse_str_properties(er[3])

            edge = Edge(msg=edge_msg)
            self.graph.update_edge(edge)

    def create_tables(self) -> None:
        c = self.sqlite_conn.cursor()

        sql_create_nodes_table = """CREATE TABLE IF NOT EXISTS nodes (
                                        name text PRIMARY KEY,
                                        type text NOT NULL,
                                        properties text
                                    );"""
        c.execute(sql_create_nodes_table)

        sql_create_edges_table = """CREATE TABLE IF NOT EXISTS edges (
                                    type text NOT NULL,
                                    source_node text NOT NULL,
                                    target_node text NOT NULL,
                                    properties text,
                                    PRIMARY KEY (type, source_node, target_node),
                                    FOREIGN KEY (source_node) REFERENCES nodes (name)
                                    FOREIGN KEY (target_node) REFERENCES nodes (name)
                                );"""
        c.execute(sql_create_edges_table)

    def parse_str_properties(self, properties: str) -> List[Property]:
        properties_dict = json.loads(properties)
        properties_list = []

        for key in properties_dict:
            msg = Property()
            msg.key = key

            value = properties_dict[key]

            if isinstance(value, bool):
                msg.value.bool_value = value
                msg.value.type = Content.BOOL
            elif isinstance(value, int):
                msg.value.int_value = value
                msg.value.type = Content.INT
            elif isinstance(value, float):
                msg.value.float_value = value
                msg.value.type = Content.FLOAT
            elif isinstance(value, str):
                msg.value.string_value = value
                msg.value.type = Content.STRING
            elif isinstance(value, list):
                if isinstance(value[0], bool):
                    msg.value.bool_vector = value
                    msg.value.type = Content.VBOOL
                elif isinstance(value[0], int):
                    msg.value.int_vector = value
                    msg.value.type = Content.VINT
                elif isinstance(value[0], float):
                    msg.value.float_vector = value
                    msg.value.type = Content.VFLOAT
                elif isinstance(value[0], str):
                    msg.value.string_vector = value
                    msg.value.type = Content.VSTRING

            properties_list.append(msg)

        return properties_list

    def parse_properties(self, properties: List[Property]) -> str:
        properties_str = "{"

        for idx, p in enumerate(properties):
            properties_str += f'"{p.key}": '
            content = p.value

            if content.type == Content.BOOL:
                properties_str += (
                    str(content.bool_value).replace("T", "t").replace("F", "f")
                )
            elif content.type == Content.INT:
                properties_str += str(content.int_value)
            elif content.type == Content.FLOAT:
                properties_str += str(content.float_value)
            elif content.type == Content.DOUBLE:
                properties_str += str(content.double_value)
            elif content.type == Content.STRING:
                properties_str += '"' + content.string_value + '"'
            elif content.type == Content.VBOOL:
                properties_str += (
                    str(content.bool_value).replace("T", "t").replace("F", "f")
                )
            elif content.type == Content.VINT:
                properties_str += str(content.int_vector)
            elif content.type == Content.VFLOAT:
                properties_str += str(content.float_vector)
            elif content.type == Content.VDOUBLE:
                properties_str += str(content.double_vector)
            elif content.type == Content.VSTRING:
                properties_str += str(content.string_vector).replace("'", '"')

            if idx < len(properties) - 1:
                properties_str += ","

        properties_str += "}"

        return properties_str

    # =========================================================================
    # SQL nodes methods
    # =========================================================================
    def exist_sql_node(self, node: NodeMsg) -> bool:
        c = self.sqlite_conn.cursor()
        c.execute("SELECT count(*) FROM nodes WHERE name = ?;", (node.name,))
        data = c.fetchone()[0]
        return data != 0

    def parse_node(self, node: NodeMsg) -> List[str]:
        return (node.name, node.type, self.parse_properties(node.properties))

    def insert_sql_node(self, node: NodeMsg) -> None:
        sql = """INSERT INTO nodes(name, type, properties)
                 VALUES(?, ?, ?);"""
        c = self.sqlite_conn.cursor()
        c.execute(sql, self.parse_node(node))
        self.sqlite_conn.commit()

    def update_sql_node(self, node: NodeMsg) -> None:
        sql = """UPDATE nodes
              SET name = ?,
                  type = ?,
                  properties = ?
              WHERE name = ?;"""
        c = self.sqlite_conn.cursor()
        c.execute(sql, self.parse_node(node) + (node.name,))
        self.sqlite_conn.commit()

    def remove_sql_node(self, node: str) -> None:
        sql = """DELETE FROM nodes WHERE name = ?;"""
        c = self.sqlite_conn.cursor()
        c.execute(sql, (node,))
        self.sqlite_conn.commit()

    # =========================================================================
    # SQL edges methods
    # =========================================================================
    def exist_sql_edge(self, edge: EdgeMsg) -> bool:
        c = self.sqlite_conn.cursor()
        c.execute(
            "SELECT count(*) FROM edges WHERE type = ? AND source_node = ? AND target_node = ?;",
            (
                edge.type,
                edge.source_node,
                edge.target_node,
            ),
        )
        data = c.fetchone()[0]
        return data != 0

    def parse_edge(self, edge: EdgeMsg) -> List[str]:
        return (
            edge.type,
            edge.source_node,
            edge.target_node,
            self.parse_properties(edge.properties),
        )

    def insert_sql_edge(self, edge: EdgeMsg) -> None:
        sql = """INSERT INTO edges(type, source_node, target_node, properties)
                 VALUES(?, ?, ?, ?);"""
        c = self.sqlite_conn.cursor()
        c.execute(sql, self.parse_edge(edge))
        self.sqlite_conn.commit()

    def update_sql_edge(self, edge: EdgeMsg) -> None:
        sql = """UPDATE edges
              SET type = ?,
                  source_node = ?,
                  target_node = ?,
                  properties = ?
              WHERE type = ? AND source_node = ? AND target_node = ?;"""
        c = self.sqlite_conn.cursor()
        c.execute(
            sql,
            self.parse_edge(edge) + (edge.type, edge.source_node, edge.target_node),
        )
        self.sqlite_conn.commit()

    def remove_sql_edge(self, edge: EdgeMsg) -> None:
        sql = """DELETE FROM edges WHERE type = ? AND source_node = ? AND target_node = ?;"""
        c = self.sqlite_conn.cursor()
        c.execute(sql, (edge.type, edge.source_node, edge.target_node))
        self.sqlite_conn.commit()

    # =========================================================================
    # Overridden KnowledgeGraph Methods
    # =========================================================================
    def update_node(self, node: Node) -> None:
        KnowledgeGraph.update_node(self.graph, node)

        msg = node.to_msg()

        if self.exist_sql_node(msg):
            self.update_sql_node(msg)
        else:
            self.insert_sql_node(msg)

    def remove_node(self, node: Node) -> bool:
        edges = self.graph.get_edges().copy()
        removed = KnowledgeGraph.remove_node(self.graph, node)

        if removed:
            self.remove_sql_node(node.get_name())

            for e in edges:
                if e.get_source_node() == node or e.get_target_node() == node:
                    self.remove_sql_edge(e.to_msg())

        return removed

    def update_edge(self, edge: Edge) -> None:
        KnowledgeGraph.update_edge(self.graph, edge)

        msg = edge.to_msg()

        if self.exist_sql_edge(msg):
            self.update_sql_edge(msg)
        else:
            self.insert_sql_edge(msg)

    def remove_edge(self, edge: Edge) -> bool:
        removed = KnowledgeGraph.remove_edge(self.graph, edge)
        if removed:
            self.remove_sql_edge(edge.to_msg())
        return removed


def main(args=None):
    try:
        rclpy.init(args=args)
        node = KnowledgeGraphDbNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
