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

from typing import Any
from typing import List
from typing import Union
from knowledge_graph_msgs.msg import Node
from knowledge_graph_msgs.msg import Edge
from knowledge_graph_msgs.msg import Content
from knowledge_graph_msgs.msg import Property


def new_node(node_name: str, node_class: str) -> Node:
    msg = Node()
    msg.node_name = node_name
    msg.node_class = node_class
    return msg


def new_content(content: Any) -> Content:
    msg = Content()

    if isinstance(content, bool):
        msg.bool_value = content
        msg.type = Content.BOOL

    elif isinstance(content, int):
        msg.int_value = content
        msg.type = Content.INT

    elif isinstance(content, float):
        msg.float_value = content
        msg.type = Content.FLOAT

    elif isinstance(content, str):
        msg.string_value = content
        msg.type = Content.STRING

    elif isinstance(content, list):

        if len(content) > 0:
            if isinstance(content[0], bool):
                msg.bool_vector = content
                msg.type = Content.VBOOL

            elif isinstance(content[0], int):
                msg.int_vector = content
                msg.type = Content.VINT

            elif isinstance(content[0], float):
                msg.float_vector = content
                msg.type = Content.VFLOAT

            elif isinstance(content[0], str):
                msg.string_vector = content
                msg.type = Content.VSTRING

        else:
            msg.type = Content.ERROR

    else:
        msg.type = Content.ERROR

    return msg


def new_edge(edge_class: str, edge_source: str, edge_target: str) -> Edge:
    msg = Edge()
    msg.edge_class = edge_class
    msg.source_node = edge_source
    msg.target_node = edge_target
    return msg


def get_content(content: Content) -> Any:

    if content.type == Content.BOOL:
        return content.bool_value
    elif content.type == Content.INT:
        return content.int_value
    elif content.type == Content.FLOAT:
        return content.float_value
    elif content.type == Content.STRING:
        return content.string_value
    elif content.type == Content.VBOOL:
        return content.bool_vector
    elif content.type == Content.VINT:
        return content.int_vector
    elif content.type == Content.VFLOAT:
        return content.float_vector
    elif content.type == Content.VSTRING:
        return content.string_vector
    else:
        return None


def add_property(element: Union[Node, Edge], key: str, content: Any) -> bool:
    found = False
    newc = new_content(content)

    if (newc.type == Content.ERROR):
        # "Adding a property of type ERROR"
        return False

    for p in element.properties:
        if (p.key == key):
            found = True
            p.content = newc
            break

    if not found:
        prop = Property()
        prop.key = key
        prop.value = newc
        element.properties.append(prop)

    return True


def get_property(element: Union[Node, Edge], key: str) -> Any:

    for p in element.properties:
        if p.key == key:
            return get_content(p.value)

    return None


def get_property_type(element: Union[Node, Edge], key: str) -> int:

    for p in element.properties:
        if p.key == key:
            return p.value.type

    return Content.ERROR
