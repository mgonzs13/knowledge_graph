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

from typing import List, Any, Union

from knowledge_graph_msgs.msg import Property, Content


class Properties:
    """
    Class for managing a collection of properties with various data types.
    """

    def __init__(self, msg: List[Property] = None) -> None:
        """
        Default constructor or initializes properties from a list of Property messages.
        :param msg: List of Property messages to initialize from.
        """
        self._properties = {}
        self._registry = {}

        if msg:
            for prop_msg in msg:
                key = prop_msg.key
                type_ = prop_msg.value.type
                if type_ == Content.BOOL:
                    self.set(key, prop_msg.value.bool_value)
                elif type_ == Content.INT:
                    self.set(key, prop_msg.value.int_value)
                elif type_ == Content.FLOAT:
                    self.set(key, prop_msg.value.float_value)
                elif type_ == Content.DOUBLE:
                    self.set(key, prop_msg.value.double_value)
                elif type_ == Content.STRING:
                    self.set(key, prop_msg.value.string_value)
                elif type_ == Content.VBOOL:
                    self.set(key, prop_msg.value.bool_vector)
                elif type_ == Content.VINT:
                    self.set(key, prop_msg.value.int_vector)
                elif type_ == Content.VFLOAT:
                    self.set(key, prop_msg.value.float_vector)
                elif type_ == Content.VDOUBLE:
                    self.set(key, prop_msg.value.double_vector)
                elif type_ == Content.VSTRING:
                    self.set(key, prop_msg.value.string_vector)
                else:
                    raise RuntimeError(f"Unsupported property type for key: {key}")

    def has(self, key: str) -> bool:
        """
        Check if a property with the given key exists.
        :param key: The key of the property.
        :return: True if the property exists, false otherwise.
        """
        return key in self._properties

    def type(self, key: str) -> str:
        """
        Get the type of the property with the given key.
        :param key: The key of the property.
        :return: The type of the property as a string.
        """
        if key in self._registry:
            return self._registry[key]
        raise RuntimeError(f"Property not found: {key}")

    def set(self, key: str, value: Any) -> None:
        """
        Set the value of a property with the given key.
        :param key: The key of the property to set.
        :param value: The value to set.
        """
        if isinstance(value, list):
            if not value:
                raise RuntimeError(f"Empty list not supported for key: {key}")
            elem_type = type(value[0]).__name__
            for v in value:
                if type(v).__name__ != elem_type:
                    raise RuntimeError(f"Mixed types in list for key: {key}")
            type_str = f"list_{elem_type}"
        else:
            supported_types = (bool, int, float, str)
            if not isinstance(value, supported_types):
                raise RuntimeError(f"Unsupported property type for key: {key}")
            type_str = type(value).__name__

        if self.has(key):
            if self._registry[key] != type_str:
                raise RuntimeError(f"Type mismatch for key: {key}")
            self._properties[key] = value
        else:
            self._properties[key] = value
            self._registry[key] = type_str

    def get(self, key: str) -> Any:
        """
        Get the value of a property with the given key.
        :param key: The key of the property to retrieve.
        :return: The value of the property.
        """
        if key in self._properties:
            return self._properties[key]
        raise RuntimeError(f"Property not found: {key}")

    def to_msg(self, key: str = None) -> Union[Property, List[Property]]:
        """
        Convert a property or all properties to Property messages.
        :param key: The key of the property to convert. If None, convert all.
        :return: The Property message or list of Property messages.
        """
        if key is not None:
            if not self.has(key):
                raise RuntimeError(f"Property not found: {key}")
            prop_msg = Property()
            prop_msg.key = key
            value = self._properties[key]
            type_str = self._registry[key]

            if type_str == "bool":
                prop_msg.value.bool_value = value
                prop_msg.value.type = Content.BOOL
            elif type_str == "int":
                prop_msg.value.int_value = value
                prop_msg.value.type = Content.INT
            elif type_str == "float":
                prop_msg.value.double_value = value
                prop_msg.value.type = Content.DOUBLE
            elif type_str == "str":
                prop_msg.value.string_value = value
                prop_msg.value.type = Content.STRING
            elif type_str.startswith("list_"):
                elem_type = type_str[5:]
                if elem_type == "bool":
                    prop_msg.value.bool_vector = value
                    prop_msg.value.type = Content.VBOOL
                elif elem_type == "int":
                    prop_msg.value.int_vector = value
                    prop_msg.value.type = Content.VINT
                elif elem_type == "float":
                    prop_msg.value.double_vector = value
                    prop_msg.value.type = Content.VDOUBLE
                elif elem_type == "str":
                    prop_msg.value.string_vector = value
                    prop_msg.value.type = Content.VSTRING
                else:
                    raise RuntimeError(f"Unsupported list element type for key: {key}")
            else:
                raise RuntimeError(f"Unsupported property type for key: {key}")
            return prop_msg
        else:
            msg_properties = []
            for k in self._properties:
                msg_properties.append(self.to_msg(k))
            return msg_properties
