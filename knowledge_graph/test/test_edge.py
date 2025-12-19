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

from knowledge_graph.graph.edge import Edge
from knowledge_graph_msgs.msg import Edge as EdgeMsg, Property, Content


class TestEdge:
    """Test suite for Edge class."""

    def test_constructor_with_parameters(self):
        """Test constructor with type, source, and target."""
        edge = Edge("connects", "node_a", "node_b")
        assert edge.get_type() == "connects"
        assert edge.get_source_node() == "node_a"
        assert edge.get_target_node() == "node_b"

    def test_constructor_from_message(self):
        """Test constructor from ROS message."""
        msg = EdgeMsg()
        msg.type = "relates"
        msg.source_node = "source"
        msg.target_node = "target"

        edge = Edge(msg=msg)
        assert edge.get_type() == "relates"
        assert edge.get_source_node() == "source"
        assert edge.get_target_node() == "target"

    def test_constructor_from_message_with_properties(self):
        """Test constructor from message with properties."""
        prop = Property()
        prop.key = "weight"
        prop.value.type = Content.DOUBLE
        prop.value.double_value = 0.75

        msg = EdgeMsg()
        msg.type = "connects"
        msg.source_node = "a"
        msg.target_node = "b"
        msg.properties = [prop]

        edge = Edge(msg=msg)
        assert edge.get_type() == "connects"
        assert edge.has_property("weight")
        assert abs(edge.get_property("weight") - 0.75) < 0.001

    def test_to_msg(self):
        """Test converting Edge to ROS message."""
        edge = Edge("connects", "node_a", "node_b")
        edge.set_property("weight", 1.5)

        msg = edge.to_msg()
        assert msg.type == "connects"
        assert msg.source_node == "node_a"
        assert msg.target_node == "node_b"
        assert len(msg.properties) == 1
        assert msg.properties[0].key == "weight"

    def test_to_string(self):
        """Test string representation of Edge."""
        edge = Edge("connects", "node_a", "node_b")
        string_repr = edge.to_string()
        assert "connects" in string_repr
        assert "node_a" in string_repr
        assert "node_b" in string_repr

    def test_str_method(self):
        """Test __str__ method."""
        edge = Edge("connects", "node_a", "node_b")
        assert str(edge) == edge.to_string()

    def test_set_and_get_property(self):
        """Test setting and getting properties on an edge."""
        edge = Edge("connects", "node_a", "node_b")
        edge.set_property("weight", 0.9)
        edge.set_property("label", "connection")
        edge.set_property("bidirectional", True)

        assert abs(edge.get_property("weight") - 0.9) < 0.001
        assert edge.get_property("label") == "connection"
        assert edge.get_property("bidirectional") is True

    def test_has_property(self):
        """Test checking if a property exists."""
        edge = Edge("connects", "node_a", "node_b")
        assert not edge.has_property("weight")
        edge.set_property("weight", 0.5)
        assert edge.has_property("weight")

    def test_property_roundtrip(self):
        """Test that properties survive a message roundtrip."""
        edge = Edge("connects", "node_a", "node_b")
        edge.set_property("weight", 0.75)
        edge.set_property("label", "test_edge")
        edge.set_property("enabled", False)

        msg = edge.to_msg()
        restored_edge = Edge(msg=msg)

        assert restored_edge.get_type() == "connects"
        assert restored_edge.get_source_node() == "node_a"
        assert restored_edge.get_target_node() == "node_b"
        assert abs(restored_edge.get_property("weight") - 0.75) < 0.001
        assert restored_edge.get_property("label") == "test_edge"
        assert restored_edge.get_property("enabled") is False
