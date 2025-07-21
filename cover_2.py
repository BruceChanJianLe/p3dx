#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

def point_in_polygon(x, y, polygon):
    num = len(polygon)
    j = num - 1
    inside = False
    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
            inside = not inside
        j = i
    return inside

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

class WorldCover(Node):
    def __init__(self, sdf_path, grid_resolution, camera_radius):
        super().__init__('worldcover')
        self.padding = 3.0
        # self.polygon = np.array([
        #     [-57.8, 53.6],
        #     [25.0, 25.0],
        #     [25.0, -35.0],
        #     [-95.0, -35.0]
        # ])
        self.polygon = np.array([
            [-100.0, 100.0],
            [100.0, 100.0],
            [100.0, -100.0],
            [-100.0, -100.0]
        ])
        polygon_width = np.max(self.polygon[:, 0]) - np.min(self.polygon[:, 0]) + 2 * self.padding
        polygon_height = np.max(self.polygon[:, 1]) - np.min(self.polygon[:, 1]) + 2 * self.padding
        min_reasonable_resolution = max(polygon_width, polygon_height) / 200
        # if grid_resolution < min_reasonable_resolution:
        #     grid_resolution = min_reasonable_resolution

        # print(f"resolution: {grid_resolution}")
        self.grid_resolution = grid_resolution
        self.camera_radius = camera_radius
        self.min_x = np.min(self.polygon[:, 0]) - self.padding
        self.max_x = np.max(self.polygon[:, 0]) + self.padding
        self.min_y = np.min(self.polygon[:, 1]) - self.padding
        self.max_y = np.max(self.polygon[:, 1]) + self.padding
        self.grid_width = int((self.max_x - self.min_x) / self.grid_resolution)
        self.grid_height = int((self.max_y - self.min_y) / self.grid_resolution)
        self.coverage_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        total_cells = self.grid_width * self.grid_height
        processed = 0
        for row in range(self.grid_height):
            for col in range(self.grid_width):
                x = self.min_x + col * self.grid_resolution + self.grid_resolution / 2
                y = self.min_y + row * self.grid_resolution + self.grid_resolution / 2
                if point_in_polygon(x, y, self.polygon):
                    self.coverage_grid[row, col] = 0
                processed += 1
        self.total_cells = np.count_nonzero(self.coverage_grid >= 0)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.laser_ranges = None
        self.laser_angle_min = None
        self.laser_angle_increment = None
        self.laser_range_max = None
        self.obstacles_detected_since_last_report = 0
        self.cells_changed_since_last_report = 0
        self.odom_msg_count = 0
        self.laser_msg_count = 0
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, '/RosAria/odom', self.track_position, qos_profile)
        self.laser_subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile)
        self.timer = self.create_timer(0.255, self.camera_visibility)
        self.cycle_count = 0

    def world_to_grid(self, x, y):
        gx = int((x - self.min_x) / self.grid_resolution)
        gy = int((y - self.min_y) / self.grid_resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.min_x + gx * self.grid_resolution + self.grid_resolution / 2
        y = self.min_y + gy * self.grid_resolution + self.grid_resolution / 2
        return x, y

    def track_position(self, msg):
        self.odom_msg_count += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_x = x
        self.robot_y = y

    def laser_callback(self, msg):
        self.laser_msg_count += 1
        self.laser_ranges = msg.ranges
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        self.laser_range_max = msg.range_max
        self.update_obstacles()

    def update_obstacles(self):
        if (self.robot_x is None or self.robot_y is None or self.robot_yaw is None or self.laser_ranges is None):
            return
        new_obstacles_this_scan = 0
        for i, range_val in enumerate(self.laser_ranges):
            if range_val <= 0 or range_val >= self.laser_range_max:
                continue
            beam_angle = self.laser_angle_min + i * self.laser_angle_increment
            global_angle = self.robot_yaw + beam_angle
            obstacle_x = self.robot_x + range_val * math.cos(global_angle)
            obstacle_y = self.robot_y + range_val * math.sin(global_angle)
            obs_gx, obs_gy = self.world_to_grid(obstacle_x, obstacle_y)
            if (0 <= obs_gx < self.grid_width and 0 <= obs_gy < self.grid_height):
                if self.coverage_grid[obs_gy, obs_gx] == 0:
                    self.coverage_grid[obs_gy, obs_gx] = 2
                    new_obstacles_this_scan += 1
        if new_obstacles_this_scan > 0:
            self.obstacles_detected_since_last_report += new_obstacles_this_scan

    def is_line_of_sight_clear(self, robot_gx, robot_gy, target_gx, target_gy):
        line_cells = bresenham_line(robot_gx, robot_gy, target_gx, target_gy)
        for gx, gy in line_cells[1:]:
            if (0 <= gx < self.grid_width and 0 <= gy < self.grid_height):
                if self.coverage_grid[gy, gx] == 2:
                    return False
        return True

    def camera_visibility(self):
        if self.robot_x is None or self.robot_y is None:
            self.cycle_count += 1
            return
        gx, gy = self.world_to_grid(self.robot_x, self.robot_y)
        if not (self.min_x <= self.robot_x <= self.max_x and self.min_y <= self.robot_y <= self.max_y):
            self.cycle_count += 1
            return
        if not point_in_polygon(self.robot_x, self.robot_y, self.polygon):
            self.cycle_count += 1
            return
        grid_radius = int(self.camera_radius / self.grid_resolution)
        cells_covered_this_cycle = 0
        for y in range(gy - grid_radius, gy + grid_radius + 1):
            for x in range(gx - grid_radius, gx + grid_radius + 1):
                dx = x - gx
                dy = y - gy
                if dx ** 2 + dy ** 2 > grid_radius ** 2:
                    continue
                if not self.check_circle_in_bounds(x, y):
                    continue
                if self.coverage_grid[y, x] != 0:
                    continue
                wx, wy = self.grid_to_world(x, y)
                angle_to_cell = math.atan2(wy - self.robot_y, wx - self.robot_x)
                relative_angle = (angle_to_cell - self.robot_yaw + math.pi) % (2 * math.pi) - math.pi
                angle_deg = math.degrees(relative_angle)
                if (-34 <= angle_deg <= 34) or (146 <= angle_deg <= 214):
                    if self.is_line_of_sight_clear(gx, gy, x, y):
                        self.coverage_grid[y, x] = 1
                        cells_covered_this_cycle += 1
        self.cycle_count += 1
        if self.cycle_count % 10 == 0:
            self.debug_status()
            if cells_covered_this_cycle > 0:
                print(f"New cells covered this cycle: {cells_covered_this_cycle}")
            if self.obstacles_detected_since_last_report > 0:
                print(f"New obstacles detected: {self.obstacles_detected_since_last_report}")
                self.obstacles_detected_since_last_report = 0
            self.total_covered()

    def check_circle_in_bounds(self, x, y):
        return (0 <= x < self.grid_width and 0 <= y < self.grid_height and self.coverage_grid[y, x] != -1)

    def debug_status(self):
        print(f"DEBUG: Messages received - Odom: {self.odom_msg_count}, Laser: {self.laser_msg_count}")
        print(f"DEBUG: Robot position: x={self.robot_x}, y={self.robot_y}, yaw={self.robot_yaw}")
        print(f"DEBUG: Laser data available: {self.laser_ranges is not None}")
        if self.robot_x is not None and self.robot_y is not None:
            in_bounds = (self.min_x <= self.robot_x <= self.max_x and self.min_y <= self.robot_y <= self.max_y)
            in_polygon = point_in_polygon(self.robot_x, self.robot_y, self.polygon)
            gx, gy = self.world_to_grid(self.robot_x, self.robot_y)
            in_grid = 0 <= gx < self.grid_width and 0 <= gy < self.grid_height
            print(f"DEBUG: Robot in coordinate bounds: {in_bounds}")
            print(f"DEBUG: Robot in polygon: {in_polygon}")
            print(f"DEBUG: Grid position: ({gx}, {gy})")
            print(f"DEBUG: In grid bounds: {in_grid}")
            if in_bounds and in_polygon and in_grid:
                print(f"DEBUG: Robot is in valid position for coverage!")
            else:
                print(f"DEBUG: Robot position issues preventing coverage")
        print("DEBUG: ---")

    def total_covered(self):
        count_covered = np.count_nonzero(self.coverage_grid == 1)
        count_free = np.count_nonzero((self.coverage_grid == 0) | (self.coverage_grid == 1))
        count_obstacles = np.count_nonzero(self.coverage_grid == 2)
        covered_percent = (count_covered / count_free) * 100 if count_free > 0 else 0
        print(f"Total coverage of the world: {covered_percent:.3f}%")
        print(f"Cells covered: {count_covered}, Obstacles detected: {count_obstacles}")
        print(f"Total free cells (covered + uncovered): {count_free}")
        actual_area = count_covered * (self.grid_resolution ** 2)
        print(f"Actual coverage area (m²): {actual_area:.3f}")
        print("-" * 50)

def main():
    rclpy.init()
    node = WorldCover('/home/user/ros2_ws/sim_ws/src/p3dx/p3dx_gazebo/worlds/warehouse_world.sdf', 0.5, 2.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
