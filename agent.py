# agent.py
from collections import deque
import heapq
import random

class Agent:
    """
    Represents the autonomous delivery agent.

    This class maintains the agent's state, including its position and goals.
    It will be responsible for calling the pathfinding algorithms to find a path.
    """

    def __init__(self, environment):
        """
        Initializes the agent with a given environment.

        Args:
            environment (Environment): An instance of the Environment class.
        """
        self.env = environment
        self.current_pos = self.env.start_pos
        self.current_packages = self.env.package_pos.copy()
        self.final_goal = self.env.goal_pos
        self.path = []
        self.path_cost = 0

    def get_neighbors(self, pos):
        """
        Gets a list of valid neighboring positions for the agent to move to.

        Args:
            pos (tuple): The current (row, col) position.

        Returns:
            list: A list of tuples representing valid neighboring coordinates.
        """
        neighbors = []
        row, col = pos
        
        # 4-connected movements (up, down, left, right)
        possible_moves = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]

        for move_row, move_col in possible_moves:
            if self.env.is_traversable(move_row, move_col):
                neighbors.append((move_row, move_col))
        
        return neighbors

    def manhattan_distance(self, pos1, pos2):
        """
        Calculates the Manhattan distance between two points.
        This serves as an admissible heuristic for A* search on a grid.

        Args:
            pos1 (tuple): The first (row, col) position.
            pos2 (tuple): The second (row, col) position.

        Returns:
            int: The Manhattan distance.
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def find_path_a_star(self, destination):
        """
        Finds a path to a destination using the A* search algorithm.

        Args:
            destination (tuple): The target (row, col) coordinates.
        
        Returns:
            list: The path from start to destination, or an empty list if no path is found.
        """
        # The frontier is a priority queue of (f_cost, position)
        frontier = [(0, self.current_pos)]
        
        # A dictionary to keep track of visited nodes and their parents for path reconstruction
        came_from = {}
        came_from[self.current_pos] = None
        
        # A dictionary to store the cost from the start node to the current node (g_cost)
        cost_so_far = {}
        cost_so_far[self.current_pos] = 0

        print("Using A* Search...")
        
        self.path = []
        self.path_cost = 0

        while frontier:
            # Pop the node with the lowest f_cost
            current_f_cost, current = heapq.heappop(frontier)

            if current == destination:
                # Reconstruct path and cost
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                self.path = path[::-1] # Reverse the path to be from start to goal
                self.path_cost = cost_so_far[destination]
                return self.path

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.env.get_cost(next_node[0], next_node[1])

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    # f_cost = g_cost + h_cost
                    priority = new_cost + self.manhattan_distance(next_node, destination)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
        
        print("No path found.")
        return []

    def find_path_bfs_ucs(self, destination):
        """
        Finds a path to a destination using BFS or Uniform-Cost Search.
        BFS is used if all terrain costs are uniform (cost = 1), otherwise UCS is used.

        Args:
            destination (tuple): The target (row, col) coordinates.
        
        Returns:
            list: The path from start to destination, or an empty list if no path is found.
        """
        # A dictionary to keep track of visited nodes and their parents for path reconstruction
        came_from = {}
        came_from[self.current_pos] = None
        
        # A dictionary to store the cost from the start node to the current node
        cost_so_far = {}
        cost_so_far[self.current_pos] = 0

        # Check if all terrain costs are uniform to decide between BFS and UCS
        is_uniform_cost = all(cost == 1 for row in self.env.grid for cost in row if cost != float('inf'))

        if is_uniform_cost:
            # Use a deque for BFS (First-In, First-Out)
            frontier = deque([self.current_pos])
            print("Using Breadth-First Search (BFS)...")
        else:
            # Use a priority queue for UCS (lowest cost first)
            frontier = [(0, self.current_pos)] # (cost, position)
            print("Using Uniform-Cost Search (UCS)...")
        
        self.path = []
        self.path_cost = 0

        while frontier:
            if is_uniform_cost:
                current = frontier.popleft()
            else:
                current_cost, current = heapq.heappop(frontier)

            if current == destination:
                # Reconstruct path and cost
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                self.path = path[::-1] # Reverse the path to be from start to goal
                self.path_cost = cost_so_far[destination]
                return self.path

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.env.get_cost(next_node[0], next_node[1])

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    came_from[next_node] = current
                    if is_uniform_cost:
                        frontier.append(next_node)
                    else:
                        heapq.heappush(frontier, (new_cost, next_node))
        
        print("No path found.")
        return []

    def find_path_local_search(self, destination, max_restarts=5):
        """
        Finds a path using a local search (Hill-Climbing with Random Restarts).

        Args:
            destination (tuple): The target (row, col) coordinates.
            max_restarts (int): The maximum number of random restarts to attempt.
        
        Returns:
            list: The best path found, or an empty list.
        """
        best_path = []
        best_cost = float('inf')

        for restart in range(max_restarts):
            print(f"Starting Hill-Climbing, restart {restart + 1}/{max_restarts}...")
            
            current_path = [self.current_pos]
            current_cost = 0
            current_node = self.current_pos

            for _ in range(self.env.width * self.env.height): # A limit to prevent infinite loops
                if current_node == destination:
                    if current_cost < best_cost:
                        best_cost = current_cost
                        best_path = current_path
                    break

                neighbors = self.get_neighbors(current_node)
                if not neighbors:
                    break # Dead end

                # Find the neighbor with the lowest heuristic cost
                next_node = min(neighbors, key=lambda n: self.manhattan_distance(n, destination))
                next_cost = self.env.get_cost(next_node[0], next_node[1])
                
                # Hill-climbing step: only move if it improves the heuristic
                if self.manhattan_distance(next_node, destination) < self.manhattan_distance(current_node, destination):
                    current_node = next_node
                    current_path.append(current_node)
                    current_cost += next_cost
                else:
                    # If we can't improve, we're at a local optimum. Randomly jump to a neighbor
                    # to try and escape it.
                    current_node = random.choice(neighbors)
                    current_path.append(current_node)
                    current_cost += self.env.get_cost(current_node[0], current_node[1])


        self.path = best_path
        self.path_cost = best_cost
        if not self.path:
            print("Local search failed to find a path.")
        else:
            print("Local search found a path.")
        return self.path

    def find_path(self, destination, algorithm="A_star"):
        """
        This is the main pathfinding method that chooses the algorithm to use.

        Args:
            destination (tuple): The target (row, col) coordinates.
            algorithm (str): The name of the algorithm to use ('BFS', 'UCS', 'A_star', or 'Local_Search').
        """
        if algorithm == "BFS" or algorithm == "UCS":
            return self.find_path_bfs_ucs(destination)
        elif algorithm == "A_star":
            return self.find_path_a_star(destination)
        elif algorithm == "Local_Search":
            return self.find_path_local_search(destination)
        else:
            print("Invalid algorithm specified. Using A* by default.")
            return self.find_path_a_star(destination)
        
    def move(self):
        """
        Moves the agent one step along its pre-planned path.
        """
        if self.path:
            next_pos = self.path.pop(0)
            self.path_cost += self.env.get_cost(next_pos[0], next_pos[1])
            self.current_pos = next_pos
            print(f"Agent moved to {self.current_pos}")
        else:
            print("No path to follow.")
