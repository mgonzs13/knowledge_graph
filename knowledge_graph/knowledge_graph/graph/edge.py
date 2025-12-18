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

from knowledge_graph_msgs.msg import Edge as EdgeMsg
from knowledge_graph.graph.properties_container import PropertiesContainer


class Edge(PropertiesContainer):
    """
    Class representing an edge in the knowledge graph.
    """

    def __init__(
        self,
        type_: str = None,
        source_node: str = None,
        target_node: str = None,
        msg: EdgeMsg = None,
    ) -> None:
        """
        Construct an Edge with type, source node, and target node, or from a ROS message.
        :param type: Edge type
        :param source_node: Source node ID
        :param target_node: Target node ID
        :param msg: ROS message representing the edge
        """
        if msg:
            super().__init__(msg.properties)
            self._type = msg.type
            self._source_node = msg.source_node
            self._target_node = msg.target_node
        else:
            super().__init__()
            self._type = type_
            self._source_node = source_node
            self._target_node = target_node

    def get_type(self) -> str:
        """
        Get edge type
        :return: Edge type
        """
        return self._type

    def get_source_node(self) -> str:
        """
        Get source node ID
        :return: Source node ID
        """
        return self._source_node

    def get_target_node(self) -> str:
        """
        Get target node ID
        :return: Target node ID
        """
        return self._target_node

    def to_msg(self) -> EdgeMsg:
        """
        Convert edge to ROS message
        :return: ROS message representation of the edge
        """
        edge_msg = EdgeMsg()
        edge_msg.type = self._type
        edge_msg.properties = self.properties_to_msg()
        edge_msg.source_node = self._source_node
        edge_msg.target_node = self._target_node
        return edge_msg

    def to_string(self) -> str:
        """
        Convert edge to string representation
        :return: String representation of the edge
        """
        return f"Edge(type: {self._type}, source: {self._source_node}, target: {self._target_node})"

    def __str__(self) -> str:
        return self.to_string()
