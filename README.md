Autonomous Delivery Agent
Project Overview
This project implements an autonomous delivery agent that navigates a 2D grid city to deliver packages. The agent is designed to be rational, choosing actions that maximize delivery efficiency. It utilizes various pathfinding algorithms to handle static and dynamic obstacles.

Required Deliverables
Source Code: All source code files are well-documented and organized.

Test Maps: Four test maps (small, medium, large, and dynamic) are included in the maps/ directory.

Report: A short report detailing the environment model, agent design, experimental results, and analysis.

Demonstration: A proof-of-concept for dynamic replanning is included in the main.py script's output log.

Setup and Dependencies
This project requires Python 3.x. The following Python libraries are used:

argparse: For handling command-line arguments.

heapq: For implementing the priority queue in A* and UCS.

collections.deque: For implementing the queue in BFS.

You can install