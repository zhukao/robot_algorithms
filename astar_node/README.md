# `A star` planning algorithm demo

![](./imgs/astar_planning.gif)

1. An `A*` pathfinding algorithm compiled and run using ROS2.

2. Generate a map of size `col X row` (default `30 X 10`) and attempt to plan a path.

3. Upon successful planning, visualize the pathfinding results in an image with a resolution of `1280 X 640`. The rendered information includes obstacles on the map, map boundaries, start point, end point, the planned path, and the search tree.

# Building

```bash
colcon build --packages-select astar_node
```

# Running

```bash
ros2 run astar_node astar_node
```
