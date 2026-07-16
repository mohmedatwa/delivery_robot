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
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/planning_goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        # Only publish the visited-map preview every N node expansions,
        # instead of flooding the topic on every single expansion.
        self.visited_publish_stride_ = 25

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [0] * (self.visited_map_.info.height * self.visited_map_.info.width)

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not transform from map to base_footprint: {ex}")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose) -> Path:
        start_time = time.perf_counter()

        explore_directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        path = Path()
        path.header.frame_id = self.map_.header.frame_id

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.pose_on_map(start_node):
            self.get_logger().error("Start pose is outside the costmap bounds.")
            return path
        if not self.pose_on_map(goal_node):
            self.get_logger().error("Goal pose is outside the costmap bounds.")
            return path

        pending_nodes = PriorityQueue()
        visited_nodes = set()
        pending_nodes.put(start_node)

        found_goal = False
        active_node = None
        expansions = 0

        max_pending_nodes = 0

        while not pending_nodes.empty() and rclpy.ok():
            max_pending_nodes = max(
                max_pending_nodes,
                pending_nodes.qsize()
            )

            active_node = pending_nodes.get()

            # Lazy deletion: this node may have been queued more than once
            # (once per path that reached it before either was expanded).
            # Only the first pop -- guaranteed lowest-cost, since the queue
            # pops in non-decreasing cost order -- is authoritative.
            if active_node in visited_nodes:
                continue
            visited_nodes.add(active_node)

            expansions += 1
            self.visited_map_.data[self.pose_to_cell(active_node)] = 10  # Blue
            if expansions % self.visited_publish_stride_ == 0:
                self.map_pub.publish(self.visited_map_)

            if active_node == goal_node:
                found_goal = True
                break

            for dir_x, dir_y in explore_directions:
                new_node: GraphNode = active_node + (dir_x, dir_y)

                if (new_node not in visited_nodes and self.pose_on_map(new_node) and
                        0 <= self.map_.data[self.pose_to_cell(new_node)] < 99):

                    new_node.cost = active_node.cost + 1 + self.map_.data[self.pose_to_cell(new_node)]
                    new_node.prev = active_node
                    pending_nodes.put(new_node)

        # Publish the final visited-map snapshot regardless of stride.
        self.map_pub.publish(self.visited_map_)

        if not found_goal:
            return path

        node = active_node
        while node and rclpy.ok():
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_.header.frame_id
            pose_stamped.pose = self.grid_to_world(node)
            path.poses.append(pose_stamped)
            node = node.prev

        path.poses.reverse()

        planning_time = (time.perf_counter() - start_time) * 1000.0

        path_cost = active_node.cost

        path_length = self.compute_path_length(path)

        self.get_logger().info("====================================")
        self.get_logger().info("Planner Statistics")
        self.get_logger().info("====================================")
        self.get_logger().info(f"Planning Time      : {planning_time:.2f} ms")
        self.get_logger().info(f"Expanded Nodes     : {expansions}")
        self.get_logger().info(f"Path Length        : {path_length:.3f} m")
        self.get_logger().info(f"Path Cost          : {path_cost:.2f}")
        self.get_logger().info(f"Number of Waypoints: {len(path.poses)}")
        self.get_logger().info(f"Peak Open List Size: {max_pending_nodes}")
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

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()