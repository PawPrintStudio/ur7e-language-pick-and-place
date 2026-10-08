# Diagrams

`ros2_node_graph.pdf` shows every ROS 2 node in the project and how they connect:

1. The big picture: the five layers.
2. The Gazebo graph, captured live with `ros2 node info` on 2026-10-08.
3. The real arm and tier-1 graph (`lab_pick.py`).
4. A troubleshooting cheat sheet: which node to check for each symptom.

To change a page, edit its `.dot` file (Graphviz), then run `bash docs/diagrams/build.sh` inside the devcontainer, which has `dot`. To re-check page 2 against a running system:

```bash
ros2 node info <node>
```

Update the page whenever a node, topic, service or action changes.
