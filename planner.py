# Type of planner
import numpy as np  # Import numpy for calculations

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Define trajectory types
PARABOLA=0; SIGMOID=1

NUM_SETPOINTS=10

class planner:
    def __init__(self, type_, trajectory_type=PARABOLA):
        self.type = type_
        self.trajectory_type = trajectory_type  # Store the trajectory type

    def plan(self, goalPoint=[-1.0, -1.0]):
        if self.type == POINT_PLANNER:
            return self.point_planner(goalPoint)
        elif self.type == TRAJECTORY_PLANNER:
            return self.trajectory_planner()

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # Implement the trajectories here
    def trajectory_planner(self):
        # The return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        trajectory_points = []

        if self.trajectory_type == PARABOLA:
            # Parabola: y = x^2 for x in [0.0, 1.5]
            x_values = np.linspace(0.0, 1.5, num=NUM_SETPOINTS)
            y_values = x_values ** 2
        elif self.trajectory_type == SIGMOID:
            # Sigmoid: y = 2 / (1 + e^{-2x}) - 1 for x in [0.0, 2.5]
            x_values = np.linspace(0.0, 2.5, num=NUM_SETPOINTS)
            y_values = 2 / (1 + np.exp(-2 * x_values)) - 1
        else:
            # Default to empty trajectory
            return trajectory_points

        trajectory_points = [[x, y] for x, y in zip(x_values, y_values)]
        return trajectory_points

