# environment.py

class Environment:
    """
    Represents the 2D grid city for the autonomous delivery agent.

    This class handles loading the map from a file, managing the grid state,
    and providing information about cell costs and traversability.
    """

    def __init__(self, map_file_path):
        """
        Initializes the environment by loading a map from a file.

        Args:
            map_file_path (str): The path to the text file containing the map.
        """
        self.grid = []
        self.start_pos = None
        self.goal_pos = None
        self.package_pos = []
        self.dynamic_obstacles = []
        self.width = 0
        self.height = 0
        self.load_map(map_file_path)

    def load_map(self, map_file_path):
        """
        Reads a map from a text file and populates the grid.

        The map format is a text file where each character represents a cell type:
        'S': Start point (cost 1)
        'G': Goal point (cost 1)
        'P': Package location (cost 1)
        'R': Road (cost 1)
        'T': Terrain (cost 3)
        'B': Building/Static Obstacle (infinite cost/impassable)
        
        Args:
            map_file_path (str): The path to the map file.
        """
        try:
            with open(map_file_path, 'r') as f:
                lines = f.readlines()
                self.height = len(lines)
                if self.height > 0:
                    self.width = len(lines[0].strip())
                
                for r, line in enumerate(lines):
                    row = []
                    for c, char in enumerate(line.strip()):
                        if char == 'S':
                            self.start_pos = (r, c)
                            row.append(1)
                        elif char == 'G':
                            self.goal_pos = (r, c)
                            row.append(1)
                        elif char == 'P':
                            self.package_pos.append((r, c))
                            row.append(1)
                        elif char == 'R':
                            row.append(1)
                        elif char == 'T':
                            row.append(3)
                        elif char == 'B':
                            row.append(float('inf')) # Use infinity for impassable
                        else:
                            # Default to a standard road if an unknown character is found
                            row.append(1)
                    self.grid.append(row)
        except FileNotFoundError:
            print(f"Error: The map file '{map_file_path}' was not found.")
            exit()

    def get_cost(self, row, col):
        """
        Returns the movement cost for a specific cell.

        Args:
            row (int): The row index of the cell.
            col (int): The column index of the cell.

        Returns:
            float: The movement cost of the cell. Returns infinity if the
                   cell is out of bounds or an impassable obstacle.
        """
        if not (0 <= row < self.height and 0 <= col < self.width):
            return float('inf')  # Out of bounds
        
        # Check if a dynamic obstacle is in the cell at this moment
        for obstacle in self.dynamic_obstacles:
            if obstacle.position == (row, col):
                return float('inf') # Impassable due to dynamic obstacle

        return self.grid[row][col]

    def add_dynamic_obstacle(self, obstacle):
        """
        Adds a dynamic obstacle to the environment.

        Args:
            obstacle (DynamicObstacle): An instance of the DynamicObstacle class.
        """
        self.dynamic_obstacles.append(obstacle)
    
    def update_dynamic_obstacles(self):
        """
        Updates the position of all dynamic obstacles based on their schedules.
        """
        for obstacle in self.dynamic_obstacles:
            obstacle.move()

    def is_traversable(self, row, col):
        """
        Checks if a given cell is traversable at the current moment.

        Args:
            row (int): The row index of the cell.
            col (int): The column index of the cell.
        
        Returns:
            bool: True if the cell is traversable, False otherwise.
        """
        if not (0 <= row < self.height and 0 <= col < self.width):
            return False  # Out of bounds
        
        if self.get_cost(row, col) == float('inf'):
            return False # Impassable static or dynamic obstacle

        return True
