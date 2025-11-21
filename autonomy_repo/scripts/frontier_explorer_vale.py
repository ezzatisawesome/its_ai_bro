#!/usr/bin/env python3
import math
import numpy as np
import typing as T

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D


def yaw_to_quat(yaw: float):
    """Create a quaternion for a pure yaw rotation (z-axis)."""
    half = 0.5 * yaw
    q = [0.0, 0.0, math.sin(half), math.cos(half)]  # x,y,z,w
    return q

def box_filter_count(mask: np.ndarray, k: int) -> np.ndarray:
    """
    Fast box-sum using integral image (no SciPy).
    mask: HxW (float or int)
    k: odd window size
    Returns HxW array of windowed sums with zero padding at borders.
    """
    assert k % 2 == 1, "window size k must be odd"
    r = k // 2
    H, W = mask.shape

    # Pad with r zeros on each side
    pad = np.pad(mask, ((r, r), (r, r)), mode='constant', constant_values=0)

    # Build integral image with an extra row/col of zeros at top-left
    I = np.zeros((H + 2 * r + 1, W + 2 * r + 1), dtype=pad.dtype)
    I[1:, 1:] = pad.cumsum(axis=0).cumsum(axis=1)

    # For each original cell (i, j), sum over a k x k window centered at it.
    # Using integral image indices:
    # S = I[i+2r+1, j+2r+1] - I[i, j+2r+1] - I[i+2r+1, j] + I[i, j]
    br = I[2 * r + 1:, 2 * r + 1:]        # bottom-right
    bl = I[2 * r + 1:, :-2 * r - 1]       # bottom-left
    tr = I[:-2 * r - 1, 2 * r + 1:]       # top-right
    tl = I[:-2 * r - 1, :-2 * r - 1]      # top-left

    # All of these are H x W now
    return br - bl - tr + tl


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ---------------- Parameters ----------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('state_topic', '/state')
        self.declare_parameter('nav_goal_topic', '/cmd_nav')    # match rviz_goal_relay output
        self.declare_parameter('nav_success_topic', '/nav_success')
        self.declare_parameter('map_frame', 'map')

        # Heuristics (same as notebook)
        self.declare_parameter('window_size', 13)
        self.declare_parameter('unknown_pct_min', 0.20)      # > 20% unknown
        self.declare_parameter('unoccupied_pct_min', 0.30)   # > 30% known free
        self.declare_parameter('occupied_zero_required', True)
        self.declare_parameter('done_unknown_ratio', 0.05)   # stop when unknown < 5%

        # Planner timing
        self.declare_parameter('plan_period', 1.0)           # seconds

        # ---------------- Internal state ----------------
        self.map_msg: T.Optional[OccupancyGrid] = None
        self.occ: T.Optional[StochOccupancyGrid2D] = None
        self.current_state: T.Optional[np.ndarray] = None    # np.array([x, y])
        self.goal_queue: T.List[T.Tuple[float, T.Tuple[float, float]]] = []
        self.pending_goal: T.Optional[T.Tuple[float, float]] = None
        self.waiting_for_result: bool = False
        self.planner_timer = None

        self.plan_period = float(
            self.get_parameter('plan_period').get_parameter_value().double_value
        )

        # ---------------- Subscriptions / Publishers ----------------
        # /state (TurtleBotState)
        self.state_sub = self.create_subscription(
            TurtleBotState,
            self.get_parameter('state_topic').get_parameter_value().string_value,
            self.on_state,
            10
        )

        # /map (OccupancyGrid)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').get_parameter_value().string_value,
            self.on_map,
            10
        )

        # /nav_success (Bool)
        self.nav_success_sub = self.create_subscription(
            Bool,
            self.get_parameter('nav_success_topic').get_parameter_value().string_value,
            self.on_nav_success,
            10
        )

        # /cmd_nav (PoseStamped goals)
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('nav_goal_topic').get_parameter_value().string_value,
            10
        )

        # Start planner timer immediately (you can also delay in the launch file)
        self.planner_timer = self.create_timer(self.plan_period, self.plan_step)
        self.get_logger().info('FrontierExplorer initialized, waiting for map and state...')

    # -------- State Callback --------
    def on_state(self, msg: TurtleBotState):
        # TurtleBotState fields: x, y, theta
        self.current_state = np.array([msg.x, msg.y], dtype=float)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        size_xy = np.array([msg.info.width, msg.info.height], dtype=int)
        origin_xy = np.array([msg.info.origin.position.x, msg.info.origin.position.y], dtype=float)
        resolution = float(msg.info.resolution)
        probs = np.asarray(msg.data, dtype=np.int16)  # [-1, 0..100]

        window_size = int(self.get_parameter('window_size').get_parameter_value().integer_value)
        self.occ = StochOccupancyGrid2D(
            resolution=resolution,
            size_xy=size_xy,
            origin_xy=origin_xy,
            window_size=window_size,
            probs=probs,
            thresh=0.5
        )

    def on_nav_success(self, msg: Bool):
        if self.waiting_for_result:
            if msg.data:
                self.get_logger().info('Goal reached. Planning next...')
            else:
                self.get_logger().warn('Navigation failed. Trying next frontier...')
            self.pending_goal = None
            self.waiting_for_result = False

    # -------- Planning Loop --------
    def plan_step(self):
        # Preconditions
        if self.occ is None or self.current_state is None:
            return
        if self.waiting_for_result:
            return  # still waiting on the navigator

        # Done criterion: ratio of unknown cells below threshold
        done_unknown_ratio = float(
            self.get_parameter('done_unknown_ratio').get_parameter_value().double_value
        )
        probs = self.occ.probs
        unknown = (probs == -1)
        known = (probs != -1)

        total_cells = probs.size
        if total_cells > 0:
            unknown_ratio = unknown.sum() / float(total_cells)
            if unknown_ratio < done_unknown_ratio:
                self.get_logger().info(
                    f'Exploration done (unknown ratio ~ {unknown_ratio:.3f}). Stopping planner.'
                )
                if self.planner_timer is not None:
                    self.planner_timer.cancel()
                    self.planner_timer = None
                return

        # If we have no queued goals, recompute frontiers
        if not self.goal_queue:
            frontiers = self.compute_frontiers(self.occ)
            if frontiers.size == 0:
                self.get_logger().warn('No frontiers found. Exploration complete or map unavailable.')
                if self.planner_timer is not None:
                    self.planner_timer.cancel()
                    self.planner_timer = None
                return

            # Sort by distance to current pose
            d = np.linalg.norm(frontiers - self.current_state.reshape(1, 2), axis=1)
            order = np.argsort(d)
            self.goal_queue = [
                (float(d[i]), (float(frontiers[i, 0]), float(frontiers[i, 1])))
                for i in order
            ]

            # Drop very-close frontiers to avoid chattering
            self.goal_queue = [item for item in self.goal_queue if item[0] > 0.3]

        if not self.goal_queue:
            return

        # Pop the nearest and send
        _, (gx, gy) = self.goal_queue.pop(0)
        self.publish_goal((gx, gy))
        self.pending_goal = (gx, gy)
        self.waiting_for_result = True

    def publish_goal(self, goal_xy):
        gx, gy = goal_xy
        # Point facing the goal from the current state
        yaw = 0.0
        if self.current_state is not None:
            dx = gx - self.current_state[0]
            dy = gy - self.current_state[1]
            yaw = math.atan2(dy, dx)

        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.nav_goal_pub.publish(msg)
        self.get_logger().info(f'Published nav goal: ({gx:.2f}, {gy:.2f})')

    # -------- Frontier computation (Notebook logic, vectorized) --------
    def compute_frontiers(self, occ: StochOccupancyGrid2D) -> np.ndarray:
        probs = occ.probs  # shape (H, W), int: -1 unknown, 0..100 known
        H, W = probs.shape

        window_size = int(self.get_parameter('window_size').get_parameter_value().integer_value)
        unknown_pct_min = float(
            self.get_parameter('unknown_pct_min').get_parameter_value().double_value
        )
        unocc_pct_min = float(
            self.get_parameter('unoccupied_pct_min').get_parameter_value().double_value
        )
        require_zero_occ = bool(
            self.get_parameter('occupied_zero_required').get_parameter_value().bool_value
        )

        # Masks
        unknown = (probs == -1).astype(np.float32)
        known = (probs != -1)
        occupied = ((probs >= 50) & known).astype(np.float32)      # >= 0.5 occupied
        unoccupied = ((probs >= 0) & (probs < 50) & known).astype(np.float32)

        # Window counts via integral image box filter
        total_cells = box_filter_count(np.ones((H, W), dtype=np.float32), window_size)
        unknown_count = box_filter_count(unknown, window_size)
        unocc_count = box_filter_count(unoccupied, window_size)
        occ_count = box_filter_count(occupied, window_size)

        # Percentages (avoid divide by zero)
        eps = 1e-9
        unknown_pct = unknown_count / (total_cells + eps)
        unocc_pct = unocc_count / (total_cells + eps)

        # Heuristic mask
        mask = (unknown_pct >= unknown_pct_min) & (unocc_pct >= unocc_pct_min)
        if require_zero_occ:
            mask &= (occ_count == 0)

        # Require the center cell be known-free for safety
        mask &= (unoccupied.astype(bool))

        # Extract grid indices
        y_idx, x_idx = np.nonzero(mask)
        if y_idx.size == 0:
            return np.empty((0, 2), dtype=float)

        grid_xy = np.stack([x_idx, y_idx], axis=1)  # (N, 2) grid coords (x, y)
        # Convert to continuous (x, y) in meters
        frontier_states = occ.grid2state(grid_xy)
        return frontier_states


def main(args=None):
    rclpy.init(args=None)
    print("main")
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()