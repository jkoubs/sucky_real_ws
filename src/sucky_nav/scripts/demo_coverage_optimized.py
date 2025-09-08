#!/usr/bin/env python3
from enum import Enum
import math
import time
from functools import partial

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon, PoseStamped, Quaternion, Point, PointStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        # --- params ---
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.close_threshold = float(self.declare_parameter('close_threshold', 0.5).value)  # meters
        self.min_vertex_separation = float(self.declare_parameter('min_vertex_separation', 0.05).value)
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)
        self.clear_on_finish = bool(self.declare_parameter('clear_on_finish', True).value)

        # Working state
        self._drawing_points = []  # list of (x, y)
        self._coverage_active = False

        # Reentrant callback group so callbacks can run concurrently
        self.cb = ReentrantCallbackGroup()

        # Coverage action
        self.coverage_client = ActionClient(
            self, NavigateCompleteCoverage, 'navigate_complete_coverage',
            callback_group=self.cb
        )

        # Core Nav2 servers (planner + controller)
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=self.cb
        )
        self.follow_path_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=self.cb
        )

        # RViz helper IO
        self.marker_pub = self.create_publisher(Marker, 'coverage_polygon', 10)
        self.clicked_sub = self.create_subscription(
            PointStamped, '/clicked_point', self._on_clicked_point, 10,
            callback_group=self.cb
        )

        # Clear service
        self.clear_srv = self.create_service(
            Empty, 'clear_coverage_polygon', self._on_clear_srv,
            callback_group=self.cb
        )

        # Periodic marker refresh (in case RViz connects later)
        self.create_timer(0.25, self._publish_polygon_markers, callback_group=self.cb)

    # -------------------- helpers --------------------

    def destroy_node(self):
        self.coverage_client.destroy()
        self.compute_path_client.destroy()
        self.follow_path_client.destroy()
        super().destroy_node()

    def _yaw_to_quaternion(self, yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

    def _ensure_closed(self, field):
        if len(field) < 3:
            return field[:]
        first, last = field[0], field[-1]
        if abs(first[0] - last[0]) > 1e-6 or abs(first[1] - last[1]) > 1e-6:
            return field[:] + [first]
        return field[:]

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32(x=float(coord[0]), y=float(coord[1]), z=0.0)
            poly.points.append(pt)
        return poly

    def _make_goal_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation = self._yaw_to_quaternion(float(yaw))
        return ps

    @staticmethod
    def _dist2(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    # -------------------- RViz interaction --------------------

    def _on_clear_srv(self, request, response):
        self._reset_polygon("manual clear")
        return response

    def _on_clicked_point(self, msg: PointStamped):
        if self._coverage_active:
            self.get_logger().warn("Coverage running; ignoring clicks. Call /clear_coverage_polygon to draw a new area.")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Clicked point frame '{msg.header.frame_id}' != '{self.map_frame}'. "
                f"Set RViz Fixed Frame to '{self.map_frame}'. Ignoring point."
            )
            return

        new_pt = (float(msg.point.x), float(msg.point.y))

        # Reject accidental doubles too close to the last vertex
        if self._drawing_points:
            if self._dist2(new_pt, self._drawing_points[-1]) < (self.min_vertex_separation ** 2):
                self.get_logger().debug("Ignored click: too close to previous vertex.")
                return

        self._drawing_points.append(new_pt)
        n = len(self._drawing_points)
        self.get_logger().info(f"Added vertex #{n}: ({new_pt[0]:.3f}, {new_pt[1]:.3f})")
        self._publish_polygon_markers()

        # Check for closure
        if n >= 3 and self._dist2(new_pt, self._drawing_points[0]) <= (self.close_threshold ** 2):
            # Replace the last point with the exact first point for a clean loop
            self._drawing_points[-1] = self._drawing_points[0]
            self.get_logger().info(f"Polygon closed (within {self.close_threshold:.2f} m of first point).")
            self._publish_polygon_markers(closed=True)

            if self.auto_start:
                self._start_from_drawn_polygon()

    def _publish_polygon_markers(self, closed: bool = False):
        # Points marker
        m_pts = Marker()
        m_pts.header.frame_id = self.map_frame
        m_pts.header.stamp = self.get_clock().now().to_msg()
        m_pts.ns = "coverage_polygon"
        m_pts.id = 1
        m_pts.type = Marker.SPHERE_LIST
        m_pts.action = Marker.ADD
        m_pts.scale.x = 0.10
        m_pts.scale.y = 0.10
        m_pts.scale.z = 0.10
        m_pts.color.a = 1.0
        m_pts.color.r = 1.0
        m_pts.color.g = 1.0
        m_pts.color.b = 1.0
        m_pts.points = [Point(x=p[0], y=p[1], z=0.0) for p in self._drawing_points]

        # Line marker
        m_line = Marker()
        m_line.header.frame_id = self.map_frame
        m_line.header.stamp = self.get_clock().now().to_msg()
        m_line.ns = "coverage_polygon"
        m_line.id = 2
        m_line.type = Marker.LINE_STRIP
        m_line.action = Marker.ADD
        m_line.scale.x = 0.05
        m_line.color.a = 1.0
        m_line.color.r = 0.0
        m_line.color.g = 1.0 if closed else 0.5
        m_line.color.b = 0.0
        pts = self._drawing_points[:]
        if closed and len(pts) >= 3 and (pts[0] != pts[-1]):
            pts.append(pts[0])
        m_line.points = [Point(x=p[0], y=p[1], z=0.0) for p in pts]

        self.marker_pub.publish(m_pts)
        self.marker_pub.publish(m_line)

    def _clear_markers(self):
        # Remove all markers published by this node (cleans up RViz)
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        self.marker_pub.publish(m)

    def _reset_polygon(self, reason: str = ""):
        self._drawing_points = []
        self._coverage_active = False
        self._clear_markers()
        if reason:
            self.get_logger().info(f"Polygon reset: {reason}")

    # -------------------- fast pre-position (planner + controller) --------------------
    # ASYNC version: no spin_until_future_complete, runs fully non-blocking.

    def fastPreposition_async(self, x: float, y: float, yaw: float = 0.0, on_done=None):
        goal_pose = self._make_goal_pose(x, y, yaw)
        plan_goal = ComputePathToPose.Goal()
        plan_goal.use_start = False
        plan_goal.goal = goal_pose

        self.get_logger().info(f"Planning path to ({x:.3f}, {y:.3f})...")
        send_plan_future = self.compute_path_client.send_goal_async(plan_goal)

        def _on_plan_goal(fut):
            handle = fut.result()
            ok = bool(handle and handle.accepted)
            self.get_logger().info(f"ComputePathToPose accepted? {ok}")
            if not ok:
                if on_done: on_done(False)
                return
            handle.get_result_async().add_done_callback(_on_plan_result)

        def _on_plan_result(fut):
            res = fut.result()
            if not res or res.result is None or len(res.result.path.poses) == 0:
                self.get_logger().error("Planner returned empty path.")
                if on_done: on_done(False)
                return

            self.get_logger().info(f"Path received with {len(res.result.path.poses)} poses. Following path...")

            follow_goal = FollowPath.Goal()
            follow_goal.path = res.result.path
            send_follow_future = self.follow_path_client.send_goal_async(follow_goal)

            def _on_follow_goal(fut2):
                h2 = fut2.result()
                ok2 = bool(h2 and h2.accepted)
                self.get_logger().info(f"FollowPath accepted? {ok2}")
                if not ok2:
                    if on_done: on_done(False)
                    return
                h2.get_result_async().add_done_callback(_on_follow_result)

            def _on_follow_result(fut3):
                try:
                    status = fut3.result().status
                except Exception as e:
                    self.get_logger().error(f"FollowPath result error: {e}")
                    if on_done: on_done(False)
                    return
                self.get_logger().info(f"FollowPath terminal status: {status}")
                if on_done: on_done(status == GoalStatus.STATUS_SUCCEEDED)

            send_follow_future.add_done_callback(_on_follow_goal)

        send_plan_future.add_done_callback(_on_plan_goal)

    # -------------------- coverage action (ASYNC) --------------------

    def navigateCoverage_async(self, field):
        # Wait briefly for server, letting executor breathe
        while not self.coverage_client.wait_for_server(timeout_sec=0.2):
            rclpy.spin_once(self, timeout_sec=0.0)

        field_closed = self._ensure_closed(field)

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = self.map_frame
        goal_msg.polygons.append(self.toPolygon(field_closed))

        self.get_logger().info('Starting complete coverage...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg, self._feedbackCallback)

        def _on_cov_goal(fut):
            self.goal_handle = fut.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error('Navigate Coverage request was rejected!')
                # Allow new drawing attempts
                self._coverage_active = False
                return
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self._on_cov_result)

        send_goal_future.add_done_callback(_on_cov_goal)

    # -------------------- task monitoring / feedback --------------------

    def _on_cov_result(self, fut):
        try:
            self.status = fut.result().status
        except Exception as e:
            self.get_logger().error(f"Coverage result error: {e}")
            self._coverage_active = False
            return

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Coverage goal succeeded!')
            if getattr(self, 'clear_on_finish', True):
                # wipes points + markers and unlocks clicks
                self._reset_polygon("coverage succeeded")
            else:
                # keep polygon visible but allow new clicks
                self._coverage_active = False
            return  # avoid falling through to the non-success unlock below
        
        # Non-success terminal states
        if self.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Coverage goal was canceled.')
        elif self.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Coverage goal failed.')
        else:
            self.get_logger().warn(f'Coverage finished with status {self.status}')
        self._coverage_active = False

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        # Optional: throttle or use a timer to log ETA if you like
        # eta = Duration.from_msg(self.feedback.estimated_time_remaining).nanoseconds / 1e9
        # self.get_logger().info(f"ETA: {eta:.0f} s", throttle_duration_sec=2.0)

    # -------------------- startup helpers --------------------

    def startup(self, node_name='bt_navigator'):
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return

    # -------------------- glue: start coverage from the drawn polygon --------------------

    def _start_from_drawn_polygon(self):
        if len(self._drawing_points) < 3:
            self.get_logger().warn("Polygon has fewer than 3 vertices; not starting coverage.")
            return

        field = self._ensure_closed(self._drawing_points)
        if len(field) >= 2 and field[0] == field[-1]:
            field = field[:-1]

        start_vertex = min(field, key=lambda p: (p[0], p[1]))
        x0, y0 = start_vertex
        self.get_logger().info(f"Pre-positioning to smallest-x vertex: ({x0:.3f}, {y0:.3f})")


        self._coverage_active = True

        def after_prepos(ok: bool):
            if not ok:
                self.get_logger().warn('Pre-position skipped/failed; continuing to coverage.')
            self.navigateCoverage_async(field)

        self.fastPreposition_async(x0, y0, yaw=0.0, on_done=after_prepos)


def main():
    rclpy.init()
    navigator = CoverageNavigatorTester()
    navigator.startup()

    # Multi-threaded executor so callbacks (subs + action results) can run concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
