#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(
        sys.argv, standalone="knowledge_graph_viewer.knowledge_graph_plugin.KnowledgeGraphPlugin"))


if __name__ == "__main__":
    main()
