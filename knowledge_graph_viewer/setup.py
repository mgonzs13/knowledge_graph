from setuptools import setup, find_packages

package_name = "knowledge_graph_viewer"
setup(
    name=package_name,
    version="4.0.1",
    # package_dir={'': ''},
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource", ["resource/KnowledgeGraphPlugin.ui"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Francisco Martin",
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=(
        "knowledge_graph_viewer provides a GUI plugin for visualizing the BICA graph."
    ),
    license="BSD",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "rqt_knowledge_graph = knowledge_graph_viewer.main:main",
        ],
    },
)
