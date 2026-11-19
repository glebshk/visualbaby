"""ROS 2 node: runs the aufdeckung MissionController against PX4 via px4_to_ros2.

Inputs
  /drone/out/pos/ned      geometry_msgs/PoseStamped  (x = N, y = E, z = D, relative to PX4 home)
  /drone/out/heading      std_msgs/Float32            (deg, 0 = north)
  /drone/out/health       ros_msg/HEALTH
  /drone/out/state/land   std_msgs/String             (ON_GROUND / IN_AIR / ...)
  <lidar_topic>           sensor_msgs/PointCloud2     (sensor frame)
  /aufdeckung/rain        std_msgs/Float32MultiArray  optional active rain zones, cage ENU:
                                                      [x, y, r, x, y, r, ...]

Outputs
  /drone/in/arm, /drone/in/offboard/status, /drone/in/offboard/pos/ned, /drone/in/land
  /aufdeckung/phase       std_msgs/String
  /aufdeckung/route       nav_msgs/Path               (cage ENU, frame "map")
  /aufdeckung/height_map  nav_msgs/OccupancyGrid      (0..100 = 0..cage height)
  /aufdeckung/coverage    nav_msgs/OccupancyGrid      (100 = imaged)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from ros_msg.msg import HEALTH, Vector4
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from aufdeckung.config import Config
from aufdeckung.planning.mission import MissionController, Phase
from aufdeckung.scenario.io import load_scenario
from aufdeckung.scenario.model import RainZone
from aufdeckung_ros.conversions import (
    SELF_FILTER_RADIUS_M,
    CaptureClock,
    StateHistory,
    cage_enu_to_setpoint,
    cloud_xyz,
    heading_to_enu_yaw,
    lidar_to_world,
    px4_ned_to_cage_enu,
    tilt_deg,
)

QUEUE = 10
YAW_DEG = 90.0  # keep the camera's long axis along cage x (east), as the planner assumes


class AufdeckungNode(Node):
    def __init__(self) -> None:
        super().__init__("aufdeckung_mission")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", -18.0)
        self.declare_parameter("config_file", "")
        self.declare_parameter("prior_scenario", "")
        self.declare_parameter("lidar_topic", "/lidar/points")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("lidar_self_filter", SELF_FILTER_RADIUS_M)
        # Mapping gates. Scans taken while the drone tilts hard carry the largest attitude errors
        # (Gazebo flight: >15 deg tilt gave 55 % phantom points, <3 deg gave 5 %), and any angle
        # error grows with distance. Skipping them costs little at 10 scans/s and 3 m/s.
        self.declare_parameter("mapping_max_tilt_deg", 12.0)
        self.declare_parameter("mapping_max_range", 20.0)

        cfg = Config()
        cfg_file = str(self.get_parameter("config_file").value)
        if cfg_file:
            cfg = cfg.with_overrides(yaml.safe_load(Path(cfg_file).read_text()) or {})
        self.cfg = cfg
        self.home = (float(self.get_parameter("home_x").value), float(self.get_parameter("home_y").value))
        prior_file = str(self.get_parameter("prior_scenario").value)
        prior = load_scenario(prior_file).obstacles if prior_file else None
        self.ctrl = MissionController(cfg, self.home, prior=prior)
        self.beam_step = (cfg.lidar.v_max_deg - cfg.lidar.v_min_deg) / max(cfg.lidar.v_samples - 1, 1)

        self.pos: tuple[float, float, float] | None = None
        self.yaw = 0.0
        self.health_ok = False
        self.land_state = ""
        self.scan: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None  # points, top beam, sensor pos
        self.rain: tuple[RainZone, ...] = ()
        self.started = False
        self.t0: float | None = None
        self.health: HEALTH | None = None
        self.attitude: tuple[float, float, float, float] | None = None
        self.attitude_time = -1e9
        self._warned_no_attitude = False
        self._scans_used = 0
        self._scans_skipped = 0
        self._last_gate_log = -1e9
        self.pos_hist = StateHistory(horizon_s=3.0)
        self.att_hist = StateHistory(horizon_s=3.0, interpolate=False)
        self.yaw_hist = StateHistory(horizon_s=3.0, interpolate=False)
        self.capture_clock = CaptureClock()
        self._scan_age = 0.0
        self.armed = False
        self.flight_mode = ""
        self._last_wait_log = -1e9
        self._last_retry = -1e9
        self._events_logged = 0
        self.finished = False

        self.sp_pub = self.create_publisher(Vector4, "/drone/in/offboard/pos/ned", QUEUE)
        self.arm_pub = self.create_publisher(Bool, "/drone/in/arm", QUEUE)
        self.offboard_pub = self.create_publisher(Bool, "/drone/in/offboard/status", QUEUE)
        self.land_pub = self.create_publisher(Bool, "/drone/in/land", QUEUE)
        self.phase_pub = self.create_publisher(String, "/aufdeckung/phase", QUEUE)
        self.route_pub = self.create_publisher(PathMsg, "/aufdeckung/route", QUEUE)
        self.height_pub = self.create_publisher(OccupancyGrid, "/aufdeckung/height_map", 1)
        self.cov_pub = self.create_publisher(OccupancyGrid, "/aufdeckung/coverage", 1)

        self.create_subscription(PoseStamped, "/drone/out/pos/ned", self.on_pose, QUEUE)
        self.create_subscription(Float32, "/drone/out/heading", self.on_heading, QUEUE)
        self.create_subscription(QuaternionStamped, "/drone/out/attitude", self.on_attitude, QUEUE)
        self.create_subscription(HEALTH, "/drone/out/health", self.on_health, QUEUE)
        self.create_subscription(String, "/drone/out/state/land", self.on_land_state, QUEUE)
        self.create_subscription(Bool, "/drone/out/armed", self.on_armed, QUEUE)
        self.create_subscription(String, "/drone/out/flightmode", self.on_flight_mode, QUEUE)
        self.create_subscription(PointCloud2, str(self.get_parameter("lidar_topic").value), self.on_lidar, 2)
        self.create_subscription(Float32MultiArray, "/aufdeckung/rain", self.on_rain, QUEUE)

        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.create_timer(1.0, self.publish_maps)
        self.get_logger().info(f"aufdeckung mission node ready, home {self.home}")

    # ------------------------------------------------------------------ callbacks

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.pos = px4_ned_to_cage_enu(p.x, p.y, p.z, self.home)
        self.pos_hist.add(self._now(), self.pos)

    def on_attitude(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        self.attitude = (q.w, q.x, q.y, q.z)
        self.attitude_time = self._now()
        self.att_hist.add(self.attitude_time, self.attitude)

    def on_heading(self, msg: Float32) -> None:
        self.yaw = heading_to_enu_yaw(msg.data)
        self.yaw_hist.add(self._now(), (self.yaw,))

    def on_health(self, msg: HEALTH) -> None:
        self.health = msg
        self.health_ok = bool(msg.gyro and msg.acc and msg.mag and msg.ned and msg.globalpos and msg.armable)

    def on_armed(self, msg: Bool) -> None:
        if msg.data != self.armed:
            self.get_logger().info(f"PX4 {'armed' if msg.data else 'disarmed'}")
        self.armed = msg.data

    def on_flight_mode(self, msg: String) -> None:
        if msg.data != self.flight_mode:
            self.get_logger().info(f"PX4 flight mode: {msg.data}")
        self.flight_mode = msg.data

    def _waiting_for(self) -> str:
        """Human-readable reason why the mission has not started yet."""
        if self.pos is None:
            return "no position from PX4 yet (/drone/out/pos/ned): is px4_to_ros2 connected?"
        if self.health is None:
            return "no health report from PX4 yet (/drone/out/health)"
        missing = [name for name in ("gyro", "acc", "mag", "ned", "globalpos", "armable")
                   if not getattr(self.health, name)]
        if missing:
            return "PX4 health checks not passed yet: " + ", ".join(missing)
        if not bool(self.get_parameter("auto_start").value):
            return "auto_start is false"
        return ""

    def on_land_state(self, msg: String) -> None:
        self.land_state = msg.data

    def on_lidar(self, msg: PointCloud2) -> None:
        if self.pos is None:
            return
        try:
            raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts = cloud_xyz(raw)
        except Exception as exc:  # a bad scan must not take the whole mission down
            self.get_logger().error(f"lidar message skipped: {exc}", throttle_duration_sec=5.0)
            return
        now = self._now()
        # Use the drone state from when the scan was TAKEN, not when it arrived (up to ~0.8 s later).
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_cap = now
        if stamp > 0:
            self.capture_clock.observe(now, stamp)
            t_cap = min(self.capture_clock.capture_time(stamp), now)
        pos_cap = self.pos_hist.at(t_cap)
        if pos_cap is None:
            self.get_logger().warning("lidar scan older than the pose history: skipped",
                                      throttle_duration_sec=5.0)
            return
        yaw_cap = self.yaw_hist.at(t_cap)
        att_cap = self.att_hist.at(t_cap)
        attitude = att_cap if att_cap is not None and now - self.attitude_time < 0.5 else None
        if attitude is None and not self._warned_no_attitude:
            self._warned_no_attitude = True
            self.get_logger().warning("no attitude on /drone/out/attitude: lidar is corrected for yaw only, "
                                      "tilt will put ground hits too high (rebuild px4_to_ros2?)")
        max_tilt = float(self.get_parameter("mapping_max_tilt_deg").value)
        if now - self._last_gate_log >= 10.0 and self._scans_used + self._scans_skipped:
            self._last_gate_log = now
            self.get_logger().info(f"lidar: {self._scans_used} scans mapped, {self._scans_skipped} skipped "
                                   f"(tilt > {max_tilt:.0f} deg), last scan taken {self._scan_age:.2f} s "
                                   f"before it arrived")
        if attitude is not None and tilt_deg(attitude) > max_tilt:
            self._scans_skipped += 1
            return
        self._scans_used += 1
        max_range = min(self.cfg.lidar.max_range, float(self.get_parameter("mapping_max_range").value))
        yaw = yaw_cap[0] if yaw_cap is not None else self.yaw
        world, top = lidar_to_world(pts, pos_cap, yaw, self.cfg.lidar.v_max_deg, self.beam_step,
                                    min_range=float(self.get_parameter("lidar_self_filter").value),
                                    attitude_ned=attitude, max_range=max_range)  # type: ignore[arg-type]
        self.scan = (world, top, np.array(pos_cap))
        self._scan_age = now - t_cap

    def on_rain(self, msg: Float32MultiArray) -> None:
        vals = list(msg.data)
        self.rain = tuple(RainZone(vals[i], vals[i + 1], vals[i + 2]) for i in range(0, len(vals) - 2, 3))

    # ------------------------------------------------------------------ main loop

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.finished:
            return
        if not self.started and (reason := self._waiting_for()):
            if now - self._last_wait_log >= 5.0:
                self._last_wait_log = now
                self.get_logger().info(f"waiting: {reason}")
            return
        if not self.started:
            self.get_logger().info("health ok: arming and switching to offboard")
            self.publish_setpoint(self.home[0], self.home[1], max(self.pos[2], 0.0))
            self.arm_pub.publish(Bool(data=True))
            self.offboard_pub.publish(Bool(data=True))
            self.started = True
            self.t0 = now
            self._last_retry = now
        elif (self.ctrl.phase == Phase.TAKEOFF and not (self.armed and self.flight_mode == "OFFBOARD")
              and now - self._last_retry >= 3.0):
            # PX4 rejects arming/offboard now and then (e.g. right after boot): ask again.
            self._last_retry = now
            self.get_logger().warn(f"not airborne yet (armed={self.armed}, mode={self.flight_mode or '?'}): "
                                   "requesting arm + offboard again")
            self.arm_pub.publish(Bool(data=True))
            self.offboard_pub.publish(Bool(data=True))
        t = now - (self.t0 or now)
        scan, self.scan = self.scan, None
        sp = self.ctrl.update(t, np.array(self.pos), scan, self.rain)
        for ev in self.ctrl.events[self._events_logged:]:
            self._log_event(ev.t, ev.kind, ev.detail)
        self._events_logged = len(self.ctrl.events)
        self.phase_pub.publish(String(data=sp.phase.value))
        if sp.phase == Phase.DONE:
            self.get_logger().info("mission done: landing")
            self.land_pub.publish(Bool(data=True))
            self.finished = True
            return
        self.publish_setpoint(sp.x, sp.y, sp.z)

    def _log_event(self, t: float, kind: str, detail: str) -> None:
        # rclpy pins one severity to each logging call site ("Logger severity cannot be changed
        # between calls"), so warnings and infos need separate calls, not one shared one.
        text = f"[{t:6.1f}s] {kind}: {detail}"
        if kind in ("planning_error", "return_failed"):
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def publish_setpoint(self, x: float, y: float, z: float) -> None:
        n, e, alt, yaw = cage_enu_to_setpoint(x, y, z, self.home, YAW_DEG)
        self.sp_pub.publish(Vector4(x=n, y=e, z=alt, w=yaw))

    def publish_maps(self) -> None:
        spec = self.ctrl.spec
        stamp = self.get_clock().now().to_msg()
        for pub, values in (
            (self.height_pub, np.clip(self.ctrl.hm.height / self.cfg.arena.height * 100, 0, 100)),
            (self.cov_pub, np.where(self.ctrl.coverage.covered, 100, 0)),
        ):
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.header.stamp = stamp
            grid.info.resolution = float(spec.res)
            grid.info.width = spec.nx
            grid.info.height = spec.ny
            grid.info.origin.position.x = spec.xmin
            grid.info.origin.position.y = spec.ymin
            grid.data = [int(v) for v in values.astype(np.int8).ravel()]
            pub.publish(grid)
        path = PathMsg()
        path.header.frame_id = "map"
        path.header.stamp = stamp
        for w in self.ctrl.route:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = w.x, w.y, w.z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.route_pub.publish(path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AufdeckungNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
