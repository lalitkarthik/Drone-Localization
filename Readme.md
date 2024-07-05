# Drone Localization and Path Planning in Maze Environment

This project demonstrates drone localization and path planning within a maze environment using computer vision techniques and probabilistic algorithms.


# Project Overview

This project aims to localize a drone within a maze environment using template matching techniques, generate a roadmap using PRM (Probabilistic Roadmap) around the initial position, and plan paths using the A* algorithm to navigate through the maze.

## Initialization and Localization

The script starts by localizing the drone's initial position within the maze using various template matching methods provided by OpenCV (`cv2.TM_CCOEFF`, `cv2.TM_CCOEFF_NORMED`, `cv2.TM_CCORR`, `cv2.TM_CCORR_NORMED`, `cv2.TM_SQDIFF`, `cv2.TM_SQDIFF_NORMED`). These methods help identify the drone's location accurately amidst maze structures.

## Roadmap Generation with PRM

Once the initial position is determined, a PRM is generated around this position. PRM is a graph-based planning technique that constructs a roadmap by connecting random points (nodes) and their nearest neighbors in the maze. This roadmap serves as a guide for efficient path planning.

## Path Planning with A*

The A* algorithm is then employed to find the shortest path from the drone's initial position to subsequent target positions within the maze. A* is a widely used algorithm for pathfinding and ensures optimal routes while considering obstacles and maze layout constraints.

## Visualization and Results

The script provides visualizations at each step of the localization and path planning process. It generates two main outputs:

- **map.png**: A snapshot of the maze map with the drone's final initial position marked.
- **snapshot.png**: A snapshot of the maze showing visualizations of paths planned from different starting conditions, illustrating connections between nodes in the roadmap and the drone's path.


