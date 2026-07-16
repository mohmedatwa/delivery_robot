#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from queue import PriorityQueue


class GraphNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.g = float("inf")
        self.h = 0.0
        self.f = float("inf")

        self.prev = None

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class AStarPlanner(Node):

    def __init__(self):
        super().__init__("astar_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.map_callback,
            map_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/planning_goal_pose",
            self.goal_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/astar/path",
            10,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/astar/visited_map",
            10,
        )

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        self.visited_publish_stride_ = 25

    def map_callback(self, msg):

        self.map_ = msg

        self.visited_map_.header.frame_id = msg.header.frame_id
        self.visited_map_.info = msg.info
        self.visited_map_.data = [-1] * (
            msg.info.width * msg.info.height
        )

    def goal_callback(self, msg):

        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [0] * (
            self.visited_map_.info.width *
            self.visited_map_.info.height
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id,
                "base_footprint",
                rclpy.time.Time(),
            )

        except TransformException as ex:
            self.get_logger().error(str(ex))
            return

        start = Pose()
        start.position.x = tf.transform.translation.x
        start.position.y = tf.transform.translation.y

        path = self.plan(start, msg.pose)

        if len(path.poses):
            self.path_pub.publish(path)
            self.get_logger().info("Path found.")
        else:
            self.get_logger().warn("No path found.")

    def heuristic(self, node, goal):
        return math.hypot(goal.x - node.x, goal.y - node.y)

    def plan(self, start, goal):

        start_time = time.perf_counter()

        path = Path()
        path.header.frame_id = self.map_.header.frame_id

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.pose_on_map(start_node):
            return path

        if not self.pose_on_map(goal_node):
            return path

        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]

        open_set = PriorityQueue()

        closed = set()

        g_score = {}

        start_node.g = 0
        start_node.h = self.heuristic(start_node, goal_node)
        start_node.f = start_node.h

        g_score[(start_node.x, start_node.y)] = 0

        open_set.put(start_node)

        found = False

        expansions = 0

        max_open_set_size = 0

        while not open_set.empty() and rclpy.ok():

            max_open_set_size = max(
                max_open_set_size,
                open_set.qsize()
            )

            current = open_set.get()

            if current in closed:
                continue

            closed.add(current)

            expansions += 1

            self.visited_map_.data[
                self.pose_to_cell(current)
            ] = 10

            if expansions % self.visited_publish_stride_ == 0:
                self.map_pub.publish(self.visited_map_)

            if current == goal_node:
                found = True
                break

            for dx, dy in directions:

                neighbor = current + (dx, dy)

                if not self.pose_on_map(neighbor):
                    continue

                cell = self.pose_to_cell(neighbor)

                cost = self.map_.data[cell]

                if cost < 0 or cost >= 99:
                    continue

                tentative_g = current.g + 1 + cost

                key = (neighbor.x, neighbor.y)

                if key not in g_score or tentative_g < g_score[key]:

                    g_score[key] = tentative_g

                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(
                        neighbor,
                        goal_node,
                    )
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.prev = current

                    open_set.put(neighbor)

        self.map_pub.publish(self.visited_map_)

        if not found:
            return path

        node = current

        while node is not None:

            ps = PoseStamped()

            ps.header.frame_id = self.map_.header.frame_id

            ps.pose = self.grid_to_world(node)

            path.poses.append(ps)

            node = node.prev

        path.poses.reverse()

        planning_time = (time.perf_counter() - start_time) * 1000.0

        path_cost = current.g

        path_length = self.compute_path_length(path)

        self.get_logger().info("====================================")
        self.get_logger().info("Planner Statistics")
        self.get_logger().info("====================================")
        self.get_logger().info(f"Planning Time      : {planning_time:.2f} ms")
        self.get_logger().info(f"Expanded Nodes     : {expansions}")
        self.get_logger().info(f"Path Length        : {path_length:.3f} m")
        self.get_logger().info(f"Path Cost          : {path_cost:.2f}")
        self.get_logger().info(f"Number of Waypoints: {len(path.poses)}")
        self.get_logger().info(f"Peak Open List Size: {max_open_set_size}")
        if planning_time > 0:
            expansion_rate = expansions / (planning_time / 1000.0)
            self.get_logger().info(
                f"Expansion Rate     : {expansion_rate:.2f} nodes/s"
            )
        self.get_logger().info("====================================")

        return path

    def compute_path_length(self, path: Path):

        length = 0.0

        for i in range(1, len(path.poses)):

            x0 = path.poses[i - 1].pose.position.x
            y0 = path.poses[i - 1].pose.position.y

            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y

            length += math.hypot(x1 - x0, y1 - y0)

        return length

    def pose_on_map(self, node):
        return (
            0 <= node.x < self.map_.info.width and
            0 <= node.y < self.map_.info.height
        )

    def world_to_grid(self, pose):

        x = int(
            (pose.position.x -
             self.map_.info.origin.position.x)
            / self.map_.info.resolution
        )

        y = int(
            (pose.position.y -
             self.map_.info.origin.position.y)
            / self.map_.info.resolution
        )

        return GraphNode(x, y)

    def grid_to_world(self, node):

        pose = Pose()

        pose.position.x = (
            node.x *
            self.map_.info.resolution +
            self.map_.info.origin.position.x
        )

        pose.position.y = (
            node.y *
            self.map_.info.resolution +
            self.map_.info.origin.position.y
        )

        return pose

    def pose_to_cell(self, node):

        return (
            node.y *
            self.map_.info.width +
            node.x
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()