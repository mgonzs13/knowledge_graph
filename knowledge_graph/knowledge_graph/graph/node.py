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

from knowledge_graph_msgs.msg import Node as NodeMsg
from knowledge_graph.graph.properties_container import PropertiesContainer


class Node(PropertiesContainer):
    """
    Class representing a node in the knowledge graph.
    """

    def __init__(self, name: str = None, type: str = None, msg: NodeMsg = None) -> None:
        """
        Constructor initializing a Node with a name and type, or from a Node message.
        :param name: The name of the node.
        :param type: The type of the node.
        :param msg: The Node message to initialize from.
        """
        if msg:
            super().__init__(msg.properties)
            self._name = msg.name
            self._type = msg.type
        else:
            super().__init__()
            self._name = name
            self._type = type

    def get_name(self) -> str:
        """
        Get the name of the node.
        :return: The name of the node.
        """
        return self._name

    def get_type(self) -> str:
        """
        Get the type of the node.
        :return: The type of the node.
        """
        return self._type

    def to_msg(self) -> NodeMsg:
        """
        Convert the Node to a Node message.
        :return: The Node message.
        """
        node_msg = NodeMsg()
        node_msg.name = self._name
        node_msg.type = self._type
        node_msg.properties = self.properties_to_msg()
        return node_msg

    def to_string(self) -> str:
        """
        Get a string representation of the Node.
        :return: String representation of the Node.
        """
        return f"Node(name: {self._name}, type: {self._type})"

    def __str__(self) -> str:
        return self.to_string()
