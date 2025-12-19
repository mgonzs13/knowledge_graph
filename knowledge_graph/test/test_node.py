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

from knowledge_graph.graph.node import Node
from knowledge_graph_msgs.msg import Node as NodeMsg, Property, Content


class TestNode:
    """Test suite for Node class."""

    def test_constructor_with_name_and_type(self):
        """Test constructor with name and type."""
        node = Node("robot", "robot_type")
        assert node.get_name() == "robot"
        assert node.get_type() == "robot_type"

    def test_constructor_from_message(self):
        """Test constructor from ROS message."""
        msg = NodeMsg()
        msg.name = "sensor"
        msg.type = "sensor_type"
        node = Node(msg=msg)
        assert node.get_name() == "sensor"
        assert node.get_type() == "sensor_type"

    def test_constructor_from_message_with_properties(self):
        """Test constructor from message with properties."""
        prop = Property()
        prop.key = "active"
        prop.value.type = Content.BOOL
        prop.value.bool_value = True

        msg = NodeMsg()
        msg.name = "robot"
        msg.type = "robot_type"
        msg.properties = [prop]

        node = Node(msg=msg)
        assert node.get_name() == "robot"
        assert node.has_property("active")
        assert node.get_property("active") is True

    def test_to_msg(self):
        """Test converting Node to ROS message."""
        node = Node("robot", "robot_type")
        node.set_property("speed", 1.5)

        msg = node.to_msg()
        assert msg.name == "robot"
        assert msg.type == "robot_type"
        assert len(msg.properties) == 1
        assert msg.properties[0].key == "speed"

    def test_to_string(self):
        """Test string representation of Node."""
        node = Node("robot", "robot_type")
        string_repr = node.to_string()
        assert "robot" in string_repr
        assert "robot_type" in string_repr

    def test_str_method(self):
        """Test __str__ method."""
        node = Node("robot", "robot_type")
        assert str(node) == node.to_string()

    def test_set_and_get_property(self):
        """Test setting and getting properties on a node."""
        node = Node("robot", "robot_type")
        node.set_property("speed", 10.5)
        node.set_property("name", "R2D2")
        node.set_property("active", True)

        assert node.get_property("speed") == 10.5
        assert node.get_property("name") == "R2D2"
        assert node.get_property("active") is True

    def test_has_property(self):
        """Test checking if a property exists."""
        node = Node("robot", "robot_type")
        assert not node.has_property("speed")
        node.set_property("speed", 10.5)
        assert node.has_property("speed")

    def test_property_roundtrip(self):
        """Test that properties survive a message roundtrip."""
        node = Node("robot", "robot_type")
        node.set_property("speed", 10.5)
        node.set_property("name", "R2D2")
        node.set_property("active", True)
        node.set_property("count", 42)

        msg = node.to_msg()
        restored_node = Node(msg=msg)

        assert restored_node.get_name() == "robot"
        assert restored_node.get_type() == "robot_type"
        assert restored_node.get_property("speed") == 10.5
        assert restored_node.get_property("name") == "R2D2"
        assert restored_node.get_property("active") is True
        assert restored_node.get_property("count") == 42
