# dynamic_obstacle.py

class DynamicObstacle:
    """
    Models a moving obstacle in the 2D grid environment.

    The obstacle moves deterministically according to a predefined schedule.
    """
    def __init__(self, initial_pos, movement_schedule):
        """
        Initializes a dynamic obstacle.

        Args:
            initial_pos (tuple): The starting (row, col) position of the obstacle.
            movement_schedule (list): A list of tuples, where each tuple
                                      represents the (delta_row, delta_col)
                                      movement for each time step.
        """
        self.position = initial_pos
        self.schedule = movement_schedule
        self.schedule_index = 0

    def move(self):
        """
        Moves the obstacle one step according to its schedule.
        """
        if self.schedule_index < len(self.schedule):
            delta_row, delta_col = self.schedule[self.schedule_index]
            self.position = (self.position[0] + delta_row, self.position[1] + delta_col)
            self.schedule_index += 1
        else:
            # Loop the schedule if it reaches the end
            self.schedule_index = 0
            delta_row, delta_col = self.schedule[self.schedule_index]
            self.position = (self.position[0] + delta_row, self.position[1] + delta_col)
            self.schedule_index += 1

    def get_future_position(self, steps_ahead):
        """
        Calculates the position of the obstacle a certain number of steps into the future.
        This is crucial for the agent's planning.

        Args:
            steps_ahead (int): The number of time steps to look ahead.

        Returns:
            tuple: The predicted (row, col) position of the obstacle.
        """
        current_step = self.schedule_index
        future_pos = list(self.position)
        for i in range(steps_ahead):
            delta_row, delta_col = self.schedule[(current_step + i) % len(self.schedule)]
            future_pos[0] += delta_row
            future_pos[1] += delta_col

        return tuple(future_pos)
