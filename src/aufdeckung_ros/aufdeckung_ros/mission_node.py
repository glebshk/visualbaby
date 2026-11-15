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
from aufdeckung.config import Config
from aufdeckung.planning.mission import MissionController, Phase
from aufdeckung.scenario.io import load_scenario
from aufdeckung.scenario.model import RainZone
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from aufdeckung_ros.conversions import (
    cage_enu_to_setpoint,
    heading_to_enu_yaw,
    lidar_to_world,
    px4_ned_to_cage_enu,
)
from ros_msg.msg import HEALTH, Vector4

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
        self.scan: tuple[np.ndarray, np.ndarray] | None = None
        self.rain: tuple[RainZone, ...] = ()
        self.started = False
        self.t0: float | None = None
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
        self.create_subscription(HEALTH, "/drone/out/health", self.on_health, QUEUE)
        self.create_subscription(String, "/drone/out/state/land", self.on_land_state, QUEUE)
        self.create_subscription(PointCloud2, str(self.get_parameter("lidar_topic").value), self.on_lidar, 2)
        self.create_subscription(Float32MultiArray, "/aufdeckung/rain", self.on_rain, QUEUE)

        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.create_timer(1.0, self.publish_maps)
        self.get_logger().info(f"aufdeckung mission node ready, home {self.home}")

    # ------------------------------------------------------------------ callbacks

    def on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.pos = px4_ned_to_cage_enu(p.x, p.y, p.z, self.home)

    def on_heading(self, msg: Float32) -> None:
        self.yaw = heading_to_enu_yaw(msg.data)

    def on_health(self, msg: HEALTH) -> None:
        self.health_ok = bool(msg.gyro and msg.acc and msg.mag and msg.ned and msg.globalpos and msg.armable)

    def on_land_state(self, msg: String) -> None:
        self.land_state = msg.data

    def on_lidar(self, msg: PointCloud2) -> None:
        if self.pos is None:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.scan = lidar_to_world(pts, self.pos, self.yaw, self.cfg.lidar.v_max_deg, self.beam_step)

    def on_rain(self, msg: Float32MultiArray) -> None:
        vals = list(msg.data)
        self.rain = tuple(RainZone(vals[i], vals[i + 1], vals[i + 2]) for i in range(0, len(vals) - 2, 3))

    # ------------------------------------------------------------------ main loop

    def tick(self) -> None:
        if self.pos is None or self.finished:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.started:
            if not (self.health_ok and bool(self.get_parameter("auto_start").value)):
                return
            self.get_logger().info("health ok: arming and switching to offboard")
            self.publish_setpoint(self.home[0], self.home[1], max(self.pos[2], 0.0))
            self.arm_pub.publish(Bool(data=True))
            self.offboard_pub.publish(Bool(data=True))
            self.started = True
            self.t0 = now
        t = now - (self.t0 or now)
        scan, self.scan = self.scan, None
        sp = self.ctrl.update(t, np.array(self.pos), scan, self.rain)
        self.phase_pub.publish(String(data=sp.phase.value))
        if sp.phase == Phase.DONE:
            self.get_logger().info("mission done: landing")
            self.land_pub.publish(Bool(data=True))
            self.finished = True
            return
        self.publish_setpoint(sp.x, sp.y, sp.z)

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
