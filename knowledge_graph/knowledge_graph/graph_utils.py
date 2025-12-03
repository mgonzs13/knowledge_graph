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

"""
@file graph_utils.py
@brief Utility functions for Knowledge Graph operations.

This module provides helper functions for creating and manipulating
graph nodes, edges, content, and properties.
"""

from typing import Any, Callable, Dict, List, Optional, Union

from knowledge_graph_msgs.msg import Node, Edge, Content, Property


# Type mappings for content types
_TYPE_TO_STRING: Dict[int, str] = {
    Content.BOOL: "bool",
    Content.INT: "int",
    Content.FLOAT: "float",
    Content.DOUBLE: "double",
    Content.STRING: "string",
    Content.VBOOL: "bool[]",
    Content.VINT: "int[]",
    Content.VFLOAT: "float[]",
    Content.VDOUBLE: "double[]",
    Content.VSTRING: "string[]",
    Content.ERROR: "error",
}

_STRING_TO_TYPE: Dict[str, int] = {v: k for k, v in _TYPE_TO_STRING.items()}


def new_node(node_name: str, node_class: str) -> Node:
    """
    @brief Creates a new graph node.

    @param node_name The name of the node.
    @param node_class The class/type of the node.
    @return A new Node message with the specified name and class.
    """
    msg = Node()
    msg.node_name = node_name
    msg.node_class = node_class
    return msg


def new_edge(edge_class: str, edge_source: str, edge_target: str) -> Edge:
    """
    @brief Creates a new graph edge.

    @param edge_class The class/type of the edge.
    @param edge_source The name of the source node.
    @param edge_target The name of the target node.
    @return A new Edge message with the specified properties.
    """
    msg = Edge()
    msg.edge_class = edge_class
    msg.source_node = edge_source
    msg.target_node = edge_target
    return msg


def new_content(content: Any) -> Content:
    """
    @brief Creates a new Content message from a value.

    @param content The content value (bool, int, float, str, or list of these).
    @return A Content message with the appropriate type set.

    @note For lists, the type is inferred from the first element.
          Empty lists result in ERROR type.
    """
    msg = Content()

    if isinstance(content, bool):
        msg.bool_value = content
        msg.type = Content.BOOL
    elif isinstance(content, int):
        msg.int_value = content
        msg.type = Content.INT
    elif isinstance(content, float):
        msg.double_value = content
        msg.type = Content.DOUBLE
    elif isinstance(content, str):
        msg.string_value = content
        msg.type = Content.STRING
    elif isinstance(content, list) and len(content) > 0:
        first = content[0]
        if isinstance(first, bool):
            msg.bool_vector = content
            msg.type = Content.VBOOL
        elif isinstance(first, int):
            msg.int_vector = content
            msg.type = Content.VINT
        elif isinstance(first, float):
            msg.double_vector = content
            msg.type = Content.VDOUBLE
        elif isinstance(first, str):
            msg.string_vector = content
            msg.type = Content.VSTRING
        else:
            msg.type = Content.ERROR
    else:
        msg.type = Content.ERROR

    return msg


# Content type to getter mapping
_CONTENT_GETTERS: Dict[int, Callable[[Content], Any]] = {
    Content.BOOL: lambda c: c.bool_value,
    Content.INT: lambda c: c.int_value,
    Content.DOUBLE: lambda c: c.double_value,
    Content.STRING: lambda c: c.string_value,
    Content.VBOOL: lambda c: c.bool_vector,
    Content.VINT: lambda c: c.int_vector,
    Content.VDOUBLE: lambda c: c.double_vector,
    Content.VSTRING: lambda c: c.string_vector,
}


def get_content(content: Content) -> Optional[Any]:
    """
    @brief Extracts the value from a Content message.

    @param content The Content message to extract from.
    @return The extracted value, or None if type is ERROR or unknown.
    """
    getter = _CONTENT_GETTERS.get(content.type)
    return getter(content) if getter else None


def _find_property(element: Union[Node, Edge], key: str) -> Optional[Property]:
    """
    @brief Finds a property by key in a node or edge.

    @param element The node or edge to search.
    @param key The property key to find.
    @return The Property if found, None otherwise.
    """
    for prop in element.properties:
        if prop.key == key:
            return prop
    return None


def add_property(element: Union[Node, Edge], key: str, content: Any) -> bool:
    """
    @brief Adds or updates a property in a node or edge.

    @param element The node or edge to modify.
    @param key The property key.
    @param content The property value.
    @return True if successful, False if the content type is ERROR.
    """
    new_content_msg = new_content(content)

    if new_content_msg.type == Content.ERROR:
        return False

    existing = _find_property(element, key)
    if existing:
        existing.value = new_content_msg
    else:
        prop = Property()
        prop.key = key
        prop.value = new_content_msg
        element.properties.append(prop)

    return True


def get_property(element: Union[Node, Edge], key: str) -> Optional[Any]:
    """
    @brief Retrieves a property value from a node or edge.

    @param element The node or edge to search.
    @param key The property key to find.
    @return The property value if found, None otherwise.
    """
    prop = _find_property(element, key)
    return get_content(prop.value) if prop else None


def get_property_type(element: Union[Node, Edge], key: str) -> int:
    """
    @brief Gets the type of a property from a node or edge.

    @param element The node or edge to search.
    @param key The property key to find.
    @return The Content type of the property, or ERROR if not found.
    """
    prop = _find_property(element, key)
    return prop.value.type if prop else Content.ERROR


def _vector_to_string(vec: List[Any], converter: Callable[[Any], str]) -> str:
    """
    @brief Helper function to convert a vector to string representation.

    @param vec The vector to convert.
    @param converter Function to convert each element to string.
    @return String representation of the vector.
    """
    items = " ".join(f" {converter(v)}" for v in vec)
    return f"[{items}]"


def type_to_string(content_type: int) -> str:
    """
    @brief Converts a Content type to its string representation.

    @param content_type The Content type value.
    @return String representation of the type (e.g., "bool", "int", "string[]").
    """
    return _TYPE_TO_STRING.get(content_type, "Unknown")


def content_to_string(content: Content) -> str:
    """
    @brief Converts a Content message to its string representation.

    @param content The Content message to convert.
    @return String representation of the content value.
    """
    converters: Dict[int, Callable[[], str]] = {
        Content.BOOL: lambda: "true" if content.bool_value else "false",
        Content.INT: lambda: str(content.int_value),
        Content.FLOAT: lambda: str(content.float_value),
        Content.DOUBLE: lambda: str(content.double_value),
        Content.STRING: lambda: content.string_value,
        Content.VBOOL: lambda: _vector_to_string(
            content.bool_vector, lambda v: "true" if v else "false"
        ),
        Content.VINT: lambda: _vector_to_string(content.int_vector, str),
        Content.VFLOAT: lambda: _vector_to_string(content.float_vector, str),
        Content.VDOUBLE: lambda: _vector_to_string(content.double_vector, str),
        Content.VSTRING: lambda: _vector_to_string(content.string_vector, str),
    }

    converter = converters.get(content.type)
    return converter() if converter else "error"


def node_to_string(node: Node) -> str:
    """
    @brief Converts a Node message to its string representation.

    @param node The Node message to convert.
    @return String representation of the node with its properties.
    """
    lines = [f"{node.node_name} ({node.node_class})"]
    for prop in node.properties:
        lines.append(f"\t{prop.key}: [{content_to_string(prop.value)}]")
    return "\n".join(lines)


def edge_to_string(edge: Edge) -> str:
    """
    @brief Converts an Edge message to its string representation.

    @param edge The Edge message to convert.
    @return String representation of the edge.
    """
    return f" [{edge.edge_class}]{edge.source_node} -> {edge.target_node}"


def type_from_string(type_str: str) -> int:
    """
    @brief Converts a type string to its Content type value.

    @param type_str The string representation of the type.
    @return The Content type value, or ERROR if not recognized.
    """
    return _STRING_TO_TYPE.get(type_str, Content.ERROR)
