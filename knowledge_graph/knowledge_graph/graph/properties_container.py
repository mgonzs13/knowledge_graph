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

from typing import List, Any
from knowledge_graph.graph.properties import Properties


class PropertiesContainer:
    """
    Class that contains a Properties object and provides methods to manage properties.
    """

    def __init__(self, msg: List[Any] = None) -> None:
        """
        Default constructor or initializes from a list of Property messages.
        :param msg: List of Property messages to initialize from.
        """
        self.properties = Properties(msg)

    def set_property(self, key: str, value: Any) -> None:
        """
        Set the value of a property with the given key.
        :param key: The key of the property to set.
        :param value: The value to set.
        """
        self.properties.set(key, value)

    def get_property(self, key: str) -> Any:
        """
        Get the value of a property with the given key.
        :param key: The key of the property to get.
        :return: The value of the property.
        """
        return self.properties.get(key)

    def has_property(self, key: str) -> bool:
        """
        Check if a property with the given key exists.
        :param key: The key of the property.
        :return: True if the property exists, false otherwise.
        """
        return self.properties.has(key)

    def properties_to_msg(self) -> List[Any]:
        """
        Convert properties to a list of Property messages.
        :return: List of Property messages.
        """
        return self.properties.to_msg()
