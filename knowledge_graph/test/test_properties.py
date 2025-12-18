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

"""Unit tests for the Properties class."""

import pytest
from knowledge_graph.graph.properties import Properties
from knowledge_graph_msgs.msg import Property, Content


class TestProperties:
    """Test suite for Properties class."""

    def test_constructor_default(self):
        """Test default constructor creates empty properties."""
        props = Properties()
        assert not props.has("nonexistent")

    def test_set_and_get_bool(self):
        """Test setting and getting boolean properties."""
        props = Properties()
        props.set("flag", True)
        assert props.has("flag")
        assert props.get("flag") is True
        assert props.type("flag") == "bool"

    def test_set_and_get_int(self):
        """Test setting and getting integer properties."""
        props = Properties()
        props.set("count", 42)
        assert props.has("count")
        assert props.get("count") == 42
        assert props.type("count") == "int"

    def test_set_and_get_float(self):
        """Test setting and getting float properties."""
        props = Properties()
        props.set("ratio", 3.14)
        assert props.has("ratio")
        assert abs(props.get("ratio") - 3.14) < 0.001
        assert props.type("ratio") == "float"

    def test_set_and_get_string(self):
        """Test setting and getting string properties."""
        props = Properties()
        props.set("name", "test_value")
        assert props.has("name")
        assert props.get("name") == "test_value"
        assert props.type("name") == "str"

    def test_set_and_get_bool_list(self):
        """Test setting and getting boolean list properties."""
        props = Properties()
        props.set("flags", [True, False, True])
        assert props.has("flags")
        assert props.get("flags") == [True, False, True]
        assert props.type("flags") == "list_bool"

    def test_set_and_get_int_list(self):
        """Test setting and getting integer list properties."""
        props = Properties()
        props.set("numbers", [1, 2, 3, 4, 5])
        assert props.has("numbers")
        assert props.get("numbers") == [1, 2, 3, 4, 5]
        assert props.type("numbers") == "list_int"

    def test_set_and_get_float_list(self):
        """Test setting and getting float list properties."""
        props = Properties()
        props.set("values", [1.1, 2.2, 3.3])
        assert props.has("values")
        assert props.get("values") == [1.1, 2.2, 3.3]
        assert props.type("values") == "list_float"

    def test_set_and_get_string_list(self):
        """Test setting and getting string list properties."""
        props = Properties()
        props.set("names", ["a", "b", "c"])
        assert props.has("names")
        assert props.get("names") == ["a", "b", "c"]
        assert props.type("names") == "list_str"

    def test_update_property_same_type(self):
        """Test updating a property with the same type."""
        props = Properties()
        props.set("value", 10)
        props.set("value", 20)
        assert props.get("value") == 20

    def test_update_property_different_type_raises(self):
        """Test updating a property with different type raises error."""
        props = Properties()
        props.set("value", 10)
        with pytest.raises(RuntimeError):
            props.set("value", "string")

    def test_get_nonexistent_property_raises(self):
        """Test getting a nonexistent property raises error."""
        props = Properties()
        with pytest.raises(RuntimeError):
            props.get("nonexistent")

    def test_type_nonexistent_property_raises(self):
        """Test getting type of nonexistent property raises error."""
        props = Properties()
        with pytest.raises(RuntimeError):
            props.type("nonexistent")

    def test_empty_list_raises(self):
        """Test setting an empty list raises error."""
        props = Properties()
        with pytest.raises(RuntimeError):
            props.set("empty", [])

    def test_mixed_type_list_raises(self):
        """Test setting a mixed type list raises error."""
        props = Properties()
        with pytest.raises(RuntimeError):
            props.set("mixed", [1, "two", 3])

    def test_to_msg_bool(self):
        """Test converting bool property to message."""
        props = Properties()
        props.set("flag", True)
        msg = props.to_msg("flag")
        assert isinstance(msg, Property)
        assert msg.key == "flag"
        assert msg.value.type == Content.BOOL
        assert msg.value.bool_value is True

    def test_to_msg_int(self):
        """Test converting int property to message."""
        props = Properties()
        props.set("count", 42)
        msg = props.to_msg("count")
        assert msg.key == "count"
        assert msg.value.type == Content.INT
        assert msg.value.int_value == 42

    def test_to_msg_float(self):
        """Test converting float property to message."""
        props = Properties()
        props.set("ratio", 3.14)
        msg = props.to_msg("ratio")
        assert msg.key == "ratio"
        assert msg.value.type == Content.DOUBLE
        assert abs(msg.value.double_value - 3.14) < 0.001

    def test_to_msg_string(self):
        """Test converting string property to message."""
        props = Properties()
        props.set("name", "test")
        msg = props.to_msg("name")
        assert msg.key == "name"
        assert msg.value.type == Content.STRING
        assert msg.value.string_value == "test"

    def test_to_msg_all_properties(self):
        """Test converting all properties to messages."""
        props = Properties()
        props.set("flag", True)
        props.set("count", 42)
        msgs = props.to_msg()
        assert len(msgs) == 2
        keys = {msg.key for msg in msgs}
        assert keys == {"flag", "count"}

    def test_constructor_from_messages(self):
        """Test constructing Properties from messages."""
        # Create property messages
        prop1 = Property()
        prop1.key = "flag"
        prop1.value.type = Content.BOOL
        prop1.value.bool_value = True

        prop2 = Property()
        prop2.key = "count"
        prop2.value.type = Content.INT
        prop2.value.int_value = 42

        prop3 = Property()
        prop3.key = "name"
        prop3.value.type = Content.STRING
        prop3.value.string_value = "test"

        props = Properties([prop1, prop2, prop3])
        assert props.get("flag") is True
        assert props.get("count") == 42
        assert props.get("name") == "test"

    def test_to_msg_nonexistent_raises(self):
        """Test converting nonexistent property raises error."""
        props = Properties()
        with pytest.raises(RuntimeError):
            props.to_msg("nonexistent")
