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

from knowledge_graph.graph_utils import (
    new_node,
    new_edge,
    new_content,
    get_content,
    add_property,
    get_property,
    get_property_type,
    type_to_string,
    content_to_string,
    node_to_string,
    edge_to_string,
    type_from_string,
)
from knowledge_graph_msgs.msg import Content


# =============================================================================
# Node Creation Tests
# =============================================================================


class TestNodeCreation(unittest.TestCase):
    """Test cases for node creation functions."""

    def test_create_node_with_valid_parameters(self):
        node = new_node("robot1", "robot")

        self.assertEqual(node.node_name, "robot1")
        self.assertEqual(node.node_class, "robot")
        self.assertEqual(len(node.properties), 0)

    def test_create_node_with_empty_name(self):
        node = new_node("", "robot")

        self.assertEqual(node.node_name, "")
        self.assertEqual(node.node_class, "robot")

    def test_create_node_with_empty_class(self):
        node = new_node("robot1", "")

        self.assertEqual(node.node_name, "robot1")
        self.assertEqual(node.node_class, "")


# =============================================================================
# Edge Creation Tests
# =============================================================================


class TestEdgeCreation(unittest.TestCase):
    """Test cases for edge creation functions."""

    def test_create_edge_with_valid_parameters(self):
        edge = new_edge("connects", "node_a", "node_b")

        self.assertEqual(edge.edge_class, "connects")
        self.assertEqual(edge.source_node, "node_a")
        self.assertEqual(edge.target_node, "node_b")
        self.assertEqual(len(edge.properties), 0)

    def test_create_self_referencing_edge(self):
        edge = new_edge("self_loop", "node_a", "node_a")

        self.assertEqual(edge.edge_class, "self_loop")
        self.assertEqual(edge.source_node, "node_a")
        self.assertEqual(edge.target_node, "node_a")


# =============================================================================
# Content Creation Tests
# =============================================================================


class TestContentCreation(unittest.TestCase):
    """Test cases for content creation functions."""

    def test_create_bool_content(self):
        content_true = new_content(True)
        content_false = new_content(False)

        self.assertEqual(content_true.type, Content.BOOL)
        self.assertTrue(content_true.bool_value)

        self.assertEqual(content_false.type, Content.BOOL)
        self.assertFalse(content_false.bool_value)

    def test_create_int_content(self):
        content = new_content(42)

        self.assertEqual(content.type, Content.INT)
        self.assertEqual(content.int_value, 42)

    def test_create_int_content_negative(self):
        content = new_content(-100)

        self.assertEqual(content.type, Content.INT)
        self.assertEqual(content.int_value, -100)

    def test_create_double_content(self):
        content = new_content(3.14159265359)

        self.assertEqual(content.type, Content.DOUBLE)
        self.assertAlmostEqual(content.double_value, 3.14159265359)

    def test_create_string_content(self):
        content = new_content("hello world")

        self.assertEqual(content.type, Content.STRING)
        self.assertEqual(content.string_value, "hello world")

    def test_create_empty_string_content(self):
        content = new_content("")

        self.assertEqual(content.type, Content.STRING)
        self.assertEqual(content.string_value, "")

    def test_create_bool_vector_content(self):
        values = [True, False, True]
        content = new_content(values)

        self.assertEqual(content.type, Content.VBOOL)
        self.assertEqual(len(content.bool_vector), 3)
        self.assertTrue(content.bool_vector[0])
        self.assertFalse(content.bool_vector[1])
        self.assertTrue(content.bool_vector[2])

    def test_create_int_vector_content(self):
        values = [1, 2, 3, 4, 5]
        content = new_content(values)

        self.assertEqual(content.type, Content.VINT)
        self.assertEqual(list(content.int_vector), values)

    def test_create_double_vector_content(self):
        values = [1.1, 2.2, 3.3]
        content = new_content(values)

        self.assertEqual(content.type, Content.VDOUBLE)
        self.assertEqual(len(content.double_vector), 3)

    def test_create_string_vector_content(self):
        values = ["a", "b", "c"]
        content = new_content(values)

        self.assertEqual(content.type, Content.VSTRING)
        self.assertEqual(list(content.string_vector), values)

    def test_create_empty_vector_content(self):
        empty_values = []
        content = new_content(empty_values)

        self.assertEqual(content.type, Content.ERROR)


# =============================================================================
# Content Extraction Tests
# =============================================================================


class TestContentExtraction(unittest.TestCase):
    """Test cases for content extraction functions."""

    def test_get_bool_content(self):
        content = new_content(True)
        result = get_content(content)

        self.assertIsNotNone(result)
        self.assertTrue(result)

    def test_get_int_content(self):
        content = new_content(42)
        result = get_content(content)

        self.assertIsNotNone(result)
        self.assertEqual(result, 42)

    def test_get_double_content(self):
        content = new_content(3.14159)
        result = get_content(content)

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result, 3.14159)

    def test_get_string_content(self):
        content = new_content("test")
        result = get_content(content)

        self.assertIsNotNone(result)
        self.assertEqual(result, "test")

    def test_get_vector_content(self):
        values = [1, 2, 3]
        content = new_content(values)
        result = get_content(content)

        self.assertIsNotNone(result)
        self.assertEqual(list(result), values)

    def test_get_content_with_error_type(self):
        content = Content()
        content.type = Content.ERROR
        result = get_content(content)

        self.assertIsNone(result)


# =============================================================================
# Property Tests
# =============================================================================


class TestProperty(unittest.TestCase):
    """Test cases for property functions."""

    def setUp(self):
        self.node = new_node("test_node", "test_class")
        self.edge = new_edge("test_edge", "source", "target")

    def test_add_property_to_node(self):
        result = add_property(self.node, "count", 10)

        self.assertTrue(result)
        self.assertEqual(len(self.node.properties), 1)
        self.assertEqual(self.node.properties[0].key, "count")

    def test_add_multiple_properties_to_node(self):
        add_property(self.node, "count", 10)
        add_property(self.node, "name", "robot")
        add_property(self.node, "speed", 1.5)

        self.assertEqual(len(self.node.properties), 3)

    def test_update_existing_property(self):
        add_property(self.node, "count", 10)
        add_property(self.node, "count", 20)

        self.assertEqual(len(self.node.properties), 1)

        value = get_property(self.node, "count")
        self.assertIsNotNone(value)
        self.assertEqual(value, 20)

    def test_add_property_to_edge(self):
        result = add_property(self.edge, "weight", 0.5)

        self.assertTrue(result)
        self.assertEqual(len(self.edge.properties), 1)
        self.assertEqual(self.edge.properties[0].key, "weight")

    def test_get_property_from_node(self):
        add_property(self.node, "label", "my_label")

        result = get_property(self.node, "label")

        self.assertIsNotNone(result)
        self.assertEqual(result, "my_label")

    def test_get_property_from_edge(self):
        add_property(self.edge, "distance", 10.5)

        result = get_property(self.edge, "distance")

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result, 10.5)

    def test_get_non_existent_property(self):
        result = get_property(self.node, "nonexistent")

        self.assertIsNone(result)

    def test_get_property_type(self):
        add_property(self.node, "int_prop", 10)
        add_property(self.node, "string_prop", "test")
        add_property(self.node, "double_prop", 3.14)

        self.assertEqual(get_property_type(self.node, "int_prop"), Content.INT)
        self.assertEqual(get_property_type(self.node, "string_prop"), Content.STRING)
        self.assertEqual(get_property_type(self.node, "double_prop"), Content.DOUBLE)

    def test_get_property_type_non_existent(self):
        prop_type = get_property_type(self.node, "nonexistent")

        self.assertEqual(prop_type, Content.ERROR)


# =============================================================================
# String Conversion Tests
# =============================================================================


class TestToString(unittest.TestCase):
    """Test cases for string conversion functions."""

    def test_type_to_string(self):
        self.assertEqual(type_to_string(Content.BOOL), "bool")
        self.assertEqual(type_to_string(Content.INT), "int")
        self.assertEqual(type_to_string(Content.FLOAT), "float")
        self.assertEqual(type_to_string(Content.DOUBLE), "double")
        self.assertEqual(type_to_string(Content.STRING), "string")
        self.assertEqual(type_to_string(Content.VBOOL), "bool[]")
        self.assertEqual(type_to_string(Content.VINT), "int[]")
        self.assertEqual(type_to_string(Content.VFLOAT), "float[]")
        self.assertEqual(type_to_string(Content.VDOUBLE), "double[]")
        self.assertEqual(type_to_string(Content.VSTRING), "string[]")
        self.assertEqual(type_to_string(Content.ERROR), "error")

    def test_bool_content_to_string(self):
        content_true = new_content(True)
        content_false = new_content(False)

        self.assertEqual(content_to_string(content_true), "true")
        self.assertEqual(content_to_string(content_false), "false")

    def test_int_content_to_string(self):
        content = new_content(42)

        self.assertEqual(content_to_string(content), "42")

    def test_string_content_to_string(self):
        content = new_content("hello")

        self.assertEqual(content_to_string(content), "hello")

    def test_node_to_string(self):
        node = new_node("robot1", "robot")

        result = node_to_string(node)

        self.assertIn("robot1", result)
        self.assertIn("robot", result)

    def test_node_with_properties_to_string(self):
        node = new_node("robot1", "robot")
        add_property(node, "speed", 10)

        result = node_to_string(node)

        self.assertIn("robot1", result)
        self.assertIn("speed", result)
        self.assertIn("10", result)

    def test_edge_to_string(self):
        edge = new_edge("connects", "node_a", "node_b")

        result = edge_to_string(edge)

        self.assertIn("connects", result)
        self.assertIn("node_a", result)
        self.assertIn("node_b", result)
        self.assertIn("->", result)


# =============================================================================
# Type From String Tests
# =============================================================================


class TestTypeFromString(unittest.TestCase):
    """Test cases for type_from_string function."""

    def test_valid_type_strings(self):
        self.assertEqual(type_from_string("bool"), Content.BOOL)
        self.assertEqual(type_from_string("int"), Content.INT)
        self.assertEqual(type_from_string("float"), Content.FLOAT)
        self.assertEqual(type_from_string("double"), Content.DOUBLE)
        self.assertEqual(type_from_string("string"), Content.STRING)

    def test_vector_type_strings(self):
        self.assertEqual(type_from_string("bool[]"), Content.VBOOL)
        self.assertEqual(type_from_string("int[]"), Content.VINT)
        self.assertEqual(type_from_string("float[]"), Content.VFLOAT)
        self.assertEqual(type_from_string("double[]"), Content.VDOUBLE)
        self.assertEqual(type_from_string("string[]"), Content.VSTRING)

    def test_invalid_type_string(self):
        self.assertEqual(type_from_string("invalid"), Content.ERROR)
        self.assertEqual(type_from_string(""), Content.ERROR)
        self.assertEqual(type_from_string("Bool"), Content.ERROR)


if __name__ == "__main__":
    unittest.main()
