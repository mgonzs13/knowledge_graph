# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from qt_dotgraph.pydotfactory import PydotFactory


class KnowledgeGraphDotcodeGenerator:

    def __init__(self):
        # self.listener_ = tf.TransformListener()
        pass

    def generate_dotgraph(
        self,
        knowledge_graphinst,
        dotcode_factory: PydotFactory,
        orientation="LR",
        rank="same",  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        # direction of layout (TB top > bottom, LR left > right)
        rankdir="TB",
        simplify=True,  # remove double edges
        quiet=False,
        unreachable=False,
    ):
        """
        See generate_dotcode
        """

        # create the graph
        # result = "digraph G {\n
        # rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(
            rank=rank, ranksep=ranksep, simplify=simplify, rankdir=orientation
        )

        # Get all nodes and edges from the graph
        nodes = knowledge_graphinst.graph.get_nodes()
        edges = knowledge_graphinst.graph.get_edges()

        # If there are no nodes, return empty graph
        if not nodes:
            return dotgraph

        # First, collect all valid node names and add them to the graph
        node_names = set()
        for node in nodes:
            node_name = node.get_name()
            if node_name:
                node_names.add(node_name)
                dotcode_factory.add_node_to_graph(
                    dotgraph,
                    nodename=node_name,
                    nodelabel=node_name,
                    shape="ellipse",
                    url=node_name,
                )

        # Only add edges where both source and target nodes exist
        for edge in edges:
            source = edge.get_source_node()
            target = edge.get_target_node()
            label = edge.get_type()

            # Skip edges with invalid endpoints
            if not source or not target:
                continue

            # Only add edge if both nodes were added to the graph
            if source in node_names and target in node_names:
                dotcode_factory.add_edge_to_graph(
                    dotgraph,
                    source,
                    target,
                    label=label,
                    url=f"{source}, {target}, {label}",
                    penwidth=1,
                )

        return dotgraph

    def generate_dotcode(
        self,
        knowledge_graphinst,
        dotcode_factory,
        orientation="LR",
        rank="same",  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        # direction of layout (TB top > bottom, LR left > right)
        rankdir="TB",
        simplify=False,  # remove double edges
        quiet=False,
        unreachable=False,
    ):
        """
        @param knowledge_graphinst: KnowledgeGraph instance
        @param ns_filter: nodename filter
        @type  ns_filter: string
        @param topic_filter: topicname filter
        @type  ns_filter: string
        @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @param hide_single_connection_topics: if true remove topics with just one connection
        @param hide_dead_end_topics: if true remove topics with publishers only
        @param cluster_namespaces_level: if > 0 places box around members of same namespace
               (TODO: multiple namespace layers)
        @param accumulate_actions: if true each 5 action topic graph nodes are shown as single graph node
        @return: dotcode generated from graph
        @rtype: str
        """
        dotgraph = self.generate_dotgraph(
            knowledge_graphinst=knowledge_graphinst,
            dotcode_factory=dotcode_factory,
            orientation=orientation,
            rank=rank,
            ranksep=ranksep,
            rankdir=rankdir,
            simplify=simplify,
            quiet=quiet,
            unreachable=unreachable,
        )
        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode
