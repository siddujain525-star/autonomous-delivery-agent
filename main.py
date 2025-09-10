# main.py
import argparse
from environment import Environment
from agent import Agent
from dynamic_obstacle import DynamicObstacle

def main():
    """
    Main function to run the autonomous delivery agent simulation.
    """
    parser = argparse.ArgumentParser(description="Autonomous Delivery Agent Simulation.")
    parser.add_argument("map_file", type=str, help="Path to the map file (e.g., maps/map_medium.txt)")
    parser.add_argument("--algorithm", type=str, default="A_star",
                        choices=["BFS", "UCS", "A_star", "Local_Search"],
                        help="The pathfinding algorithm to use.")
    
    args = parser.parse_args()

    # Create the environment and agent
    env = Environment(args.map_file)
    agent = Agent(env)

    # Add a dynamic obstacle for demonstration purposes
    # The obstacle will move right, then down.
    # We will simulate a scenario where it appears unexpectedly during the run.
    dynamic_obstacle_schedule = [(0, 1), (0, 1), (1, 0), (1, 0)]
    dynamic_obstacle = DynamicObstacle(initial_pos=(2, 2), movement_schedule=dynamic_obstacle_schedule)
    
    print(f"Starting simulation on map: {args.map_file}")
    print(f"Algorithm selected: {args.algorithm}")
    print(f"Start position: {agent.current_pos}")
    print(f"Goal position: {agent.final_goal}")

    # Initial plan
    print("\n--- Initial Planning ---")
    path_found = agent.find_path(agent.final_goal, algorithm=args.algorithm)
    
    if not path_found:
        print("Initial path could not be found. Exiting.")
        return

    print("Initial Path Found:", path_found)
    print("Initial Path Cost:", agent.path_cost)
    
    # Simulate movement with dynamic replanning
    print("\n--- Simulation ---")
    current_time_step = 0
    while agent.current_pos != agent.final_goal and agent.path:
        # Simulate an unexpected obstacle appearing at a specific time step
        if current_time_step == 3:
            print(f"\nTime step {current_time_step}: A dynamic obstacle has appeared at {dynamic_obstacle.position}!")
            env.add_dynamic_obstacle(dynamic_obstacle)
            
            # Agent replans its path from its current position
            print("Agent detects the new obstacle and is replanning...")
            agent.find_path(agent.final_goal, algorithm=args.algorithm)
            
            if not agent.path:
                print("Replanning failed. Agent is stuck.")
                break
            
            print("New Path Found:", agent.path)
            
        print(f"\nTime step {current_time_step}: Agent at {agent.current_pos}")
        agent.move()
        current_time_step += 1
    
    print("\n--- Simulation Complete ---")
    if agent.current_pos == agent.final_goal:
        print(f"Agent reached the goal at {agent.final_goal} in {current_time_step} time steps.")
    else:
        print("Agent did not reach the goal.")

if __name__ == "__main__":
    main()
