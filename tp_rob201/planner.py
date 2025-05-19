
"""
Planner class
Implementation of A* with map inflation to keep paths away from walls
"""

import numpy as np
import heapq
from occupancy_grid import OccupancyGrid


class Planner:
    """Simple occupancy grid A* planner with obstacle inflation."""

    # Inflation radius (cells) and obstacle threshold for occupancy
    MAP_INFLATION_RADIUS = 5
    OBSTACLE_THRESHOLD   = 15

    def __init__(self, occupancy_grid: OccupancyGrid):
        # Store reference to the occupancy grid
        self.grid = occupancy_grid

        # Precompute an inflated occupancy map for safety margin
        self.inflated_map = None

    def get_neighbors(self, cell, inflated):
        """Return free 8-connected neighbors of cell on inflated map."""
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cell[0] + dx, cell[1] + dy
                if 0 <= nx < inflated.shape[0] and 0 <= ny < inflated.shape[1]:
                    # only free cells in inflated map
                    if inflated[nx, ny] <= self.OBSTACLE_THRESHOLD:
                        neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, cell1, cell2):
        """Euclidean distance between two cells."""
        return np.hypot(cell1[0] - cell2[0], cell1[1] - cell2[1])

    def plan(self, start, goal):
        """
        Compute a path from start to goal using A* on an inflated map.

        start, goal: world coords [x, y, theta].
        Returns list of [x, y, 0] poses or None if no path.
        """
        # Convert to map indices
        sx, sy = self.grid.conv_world_to_map(start[0], start[1])
        gx, gy = self.grid.conv_world_to_map(goal[0], goal[1])
        start_cell = (sx, sy)
        goal_cell  = (gx, gy)

        # Generate inflated map if not done
        inflated = self.grid.get_inflated_map(
                radius=self.MAP_INFLATION_RADIUS,
                threshold=self.OBSTACLE_THRESHOLD
            )

        # Validate start/goal
        if inflated[start_cell] > self.OBSTACLE_THRESHOLD:
            print("Start inside an inflated obstacle!")
            return None
        if inflated[goal_cell] > self.OBSTACLE_THRESHOLD:
            print("Goal inside an inflated obstacle!")
            return None

        # A* structures
        open_heap = []
        heapq.heappush(open_heap, (0, start_cell))
        in_open = {start_cell}
        came_from = {}
        g_score = {start_cell: 0.0}
        f_score = {start_cell: self.heuristic(start_cell, goal_cell)}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            in_open.remove(current)

            if current == goal_cell:
                # reconstruct path
                path_cells = [current]
                while current in came_from:
                    current = came_from[current]
                    path_cells.append(current)
                path_cells.reverse()

                # world coordinates
                path_world = []
                for cx, cy in path_cells:
                    xw, yw = self.grid.conv_map_to_world(cx, cy)
                    path_world.append(np.array([xw, yw, 0.0]))
                return path_world

            # expand neighbors on inflated map
            for neighbor in self.get_neighbors(current, inflated):
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                move_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, np.inf):
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f_score[neighbor]   = tentative_g + self.heuristic(neighbor, goal_cell)
                    if neighbor not in in_open:
                        heapq.heappush(open_heap, (f_score[neighbor], neighbor))
                        in_open.add(neighbor)

        # no path found
        print("No path found on inflated map")
        return None

    def is_in_bounds(self, position):
        """Check if a map position lies within the grid bounds."""
        x, y = int(position[0]), int(position[1])
        max_x, max_y = self.grid.occupancy_map.shape
        return (0 <= x < max_x) and (0 <= y < max_y)

    def explore_frontiers(self):
        """Placeholder for frontier-based exploration."""
        return np.array([0, 0, 0])
