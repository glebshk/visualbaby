#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError, VelocityNedYaw, AccelerationNed, VelocityBodyYawspeed
from mavsdk.action import ActionError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,Float32, String, Float32MultiArray
import time 
from ros_msg.msg import NEDPosVelAcc, NEDPosVel, Vector4, HEALTH, TelBatt
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import px4_to_ros2.config as cfg

import threading

class FlyNode(Node):
    def __init__(self):
        super().__init__("px4_ros2")
        self.get_logger().info("PX4 Interface Node started")

        # Declare Parameters
        self.declare_parameter("envirement","simulator")

        # Create ROS Telemetry Publischer
        self.setup_ROS_publisher()

        # Create ROS Subscription
        self.setup_ROS_subscription()

        # Setup checks
        self.setup_variables()

        #Setuo Drone Connection
        self.drone = System()

        # Setup MAVSDK Loop in extra Thread
        self.mavsdk_loop = asyncio.new_event_loop()
        # Start MAVSDK in separatem Thread
        thread = threading.Thread(target=self.start_mavsdk_loop, daemon=True)
        thread.start()
    
    def setup_variables(self):
        self.offboard_status_set = False
        self.action_arm_set = False
        self.arm = False
    
    def setup_ROS_publisher(self):
        # Setup ROS Publisher for Telemetry
        self.status_arm_pub = self.create_publisher(Bool, "/drone/out/armed", cfg.ROS_QUEUE_SIZE)
        self.alt_pub = self.create_publisher(Float32, "/drone/out/altitude", cfg.ROS_QUEUE_SIZE)
        self.bat_pub = self.create_publisher(TelBatt, "/drone/out/battery", cfg.ROS_QUEUE_SIZE)
        self.heading_pub = self.create_publisher(Float32, "/drone/out/heading", cfg.ROS_QUEUE_SIZE)
        self.fm_pub = self.create_publisher(String, "/drone/out/flightmode", cfg.ROS_QUEUE_SIZE)
        self.health_pub = self.create_publisher(HEALTH, "/drone/out/health", cfg.ROS_QUEUE_SIZE)
        self.imu_pub = self.create_publisher(Imu, "/drone/out/imu", cfg.ROS_QUEUE_SIZE)
        self.ned_pos_pub = self.create_publisher(PoseStamped, "/drone/out/pos/ned", cfg.ROS_QUEUE_SIZE)
        self.landing_state_pub = self.create_publisher(String, "/drone/out/state/land", cfg.ROS_QUEUE_SIZE)
        self.in_air_pub = self.create_publisher(Bool, "/drone/out/state/in_air", cfg.ROS_QUEUE_SIZE)

    def setup_ROS_subscription(self):
        # Setup ROS Subscription for Drone Controll
        self.create_subscription(Bool,"/drone/in/offboard/status", self.set_offboard_status, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Vector4,"/drone/in/offboard/pos/ned",self.set_offboard_ned_position, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(NEDPosVelAcc, "/drone/in/position/ned_acc_vel_pos", self.set_offboard_ned_acc_vel_pos, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Bool,"/drone/in/arm", self.set_action_arm, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Bool,"/drone/in/land",self.set_action_land, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Bool,"/drone/in/kill", self.set_action_kill, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Bool,"/drone/in/takeoff", self.set_action_takeoff, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Bool,"/drone/in/terminate", self.set_action_terminate, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Vector4, "/drone/in/offboard/vel/ned", self.set_offboard_ned_velocity, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Vector4, "/drone/in/offboard/body/vel", self.set_offboard_body_velocity, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(NEDPosVel, "/drone/in/offboard/pos/ned_vel", self.set_offboard_ned_velocity, cfg.ROS_QUEUE_SIZE)
        
    def start_mavsdk_loop(self):
        # Start MAVSDK Loop
        asyncio.set_event_loop(self.mavsdk_loop)
        self.mavsdk_loop.run_until_complete(self.mavsdk_task())
    
    async def setup_rates(self):
        # Setup Rates for Telemetry of the MAVLink Communication 
        await self.drone.telemetry.set_rate_battery(cfg.TELEMETRY_BATT_RATE)
        await self.drone.telemetry.set_rate_altitude(cfg.TELEMETRY_ALT_RATE)
        await self.drone.telemetry.set_rate_imu(cfg.TELEMETRY_IMU_RATE)
        await self.drone.telemetry.set_rate_landed_state(cfg.TELEMETRY_LANDED_STATE_RATE)
        await self.drone.telemetry.set_rate_position(cfg.TELEMETRY_POS_RATE)
        await self.drone.action.set_takeoff_altitude(cfg.PX4_TAKEOFF_ALTITUDE)
    
    async def mavsdk_task(self):
        # Start of mavlink Connection
        self.env = self.get_parameter("envirement").value
        self.get_logger().info("Connecting to PX4 ....")
        if self.env == "simulator":
            # Connect to Simulator
            self.get_logger().info(f"Connect to Simulator on {cfg.PX4_CONNECTION_STRING_SIM}")
            await self.drone.connect(system_address=cfg.PX4_CONNECTION_STRING_SIM)

        elif self.env == "drone":
            # Connect to drone via Serial
            self.get_logger().info(f"Connect to Drone on {cfg.PX4_CONNECTION_STRING_DRONE}")
            await self.drone.connect(system_address=cfg.PX4_CONNECTION_STRING_DRONE)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                # Check if Connection was sucessfull
                self.get_logger().info("PX4 connected!")
                break
        
        # Setup Telemetry Rates from PX4
        await self.setup_rates()

        # Start Telemetry Publisher Tasks
        await asyncio.gather(
        self.publish_status_armed(),
        self.publish_status_bat(),
        self.publish_status_altitude(),
        self.publish_status_flightmode(),
        self.publish_status_imu(),
        self.publish_status_ned_pos(),
        self.publish_status_landed_state(),
        self.publish_status_heading(),
        self.publish_status_health(),
        self.publish_status_in_air()
    )

    async def publish_status_armed(self):
        # Publish Telemetry armed Status
        async for self.status_arm in self.drone.telemetry.armed():
            #self.get_logger().info(f"armed {self.arm}")
            msg = Bool()
            msg.data = self.status_arm
            self.status_arm_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_bat(self):
        async for self.bat in self.drone.telemetry.battery():
            #self.get_logger().info(f"bat {self.bat}")
            msg = TelBatt()
            msg.voltage = self.bat.voltage_v
            msg.current = self.bat.current_battery_a
            msg.consumed_mah = self.bat.capacity_consumed_ah
            self.bat_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_in_air(self):
        async for self.in_air in self.drone.telemetry.in_air():
            #self.get_logger().info(f"bat {self.bat}")
            msg = Bool()
            msg.data = self.in_air
            self.in_air_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_health(self):
        async for self.health in self.drone.telemetry.health():
            # Publish Helth of systems
            msg = HEALTH()
            if self.health.is_accelerometer_calibration_ok:
                msg.acc = True
            if self.health.is_gyrometer_calibration_ok:
                msg.gyro = True
            if self.health.is_magnetometer_calibration_ok:
                msg.mag = True
            if self.health.is_local_position_ok:
                msg.ned = True
            if self.health.is_global_position_ok:
                msg.globalpos = True
            if self.health.is_home_position_ok:
                msg.homepos = True
            if self.health.is_armable:
                msg.armable = True
            self.health_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_altitude(self):
        async for self.alt in self.drone.telemetry.altitude():
            #self.get_logger().info(f"alt {self.alt}")
            msg = Float32()
            msg.data = self.alt.altitude_relative_m
            self.alt_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_heading(self):
        async for self.heading in self.drone.telemetry.heading():
            #self.get_logger().info(f"alt {self.alt}")
            msg = Float32()
            msg.data = self.heading.heading_deg
            self.heading_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_flightmode(self):
        async for self.fm in self.drone.telemetry.flight_mode():
            #self.get_logger().info(f"alt {self.alt}")
            msg = String()
            msg.data = self.fm.name
            self.fm_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_imu(self):
        async for self.imu in self.drone.telemetry.imu():
            #self.get_logger().info(f"alt {self.imu}")
            msg = Imu()
            msg.linear_acceleration.x = self.imu.acceleration_frd.forward_m_s2
            msg.linear_acceleration.y = self.imu.acceleration_frd.right_m_s2
            msg.linear_acceleration.z = self.imu.acceleration_frd.down_m_s2
            
            msg.angular_velocity.x = self.imu.angular_velocity_frd.forward_rad_s
            msg.angular_velocity.y = self.imu.angular_velocity_frd.right_rad_s
            msg.angular_velocity.z = self.imu.angular_velocity_frd.down_rad_s

            self.imu_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_IMU_PUBLISH_RATE)
    
    async def publish_status_ned_pos(self):
        async for self.ned_pos in self.drone.telemetry.position_velocity_ned():
            #self.get_logger().info(f"alt {self.alt}")
            msg = PoseStamped()
            msg.pose.position.x = self.ned_pos.position.north_m
            msg.pose.position.y = self.ned_pos.position.east_m
            msg.pose.position.z = self.ned_pos.position.down_m
            self.ned_pos_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    
    async def publish_status_landed_state(self):
        async for self.landed_state in self.drone.telemetry.landed_state():
            #self.get_logger().info(f"alt {self.landed_state.name}")
            msg = String()
            msg.data = str(self.landed_state.name)
            self.landing_state_pub.publish(msg)
            await asyncio.sleep(1/cfg.TELEMETRY_PUBLISH_RATE)
    

    def set_offboard_status(self,msg):
        # Set Offboard Modus
        self.get_logger().debug(f"Set Offboard Modus: {msg.data}")
        self.offboard_status = msg.data
        # Async Funktion thread-sicher ausführen
        if self.offboard_status and not self.offboard_status_set:
            self.offboard_status_set = True
            try: #TODO Action result
                self.get_logger().info("Start Offboard Modus")
                asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, -1*cfg.PX4_TAKEOFF_ALTITUDE, 0.0)  # N, E, D, Yaw
                    
                ),
                self.mavsdk_loop)
                asyncio.run_coroutine_threadsafe(
                    self.drone.offboard.start(),
                    self.mavsdk_loop
                )
                
            except OffboardError as e:
                self.get_logger().error(f"Error offboard: {e}")

        elif not self.offboard_status and self.offboard_status_set:
            self.offboard_status_set = False
            try: #TODO Action result
                self.get_logger().info("Stop Offboard Modus")
                asyncio.run_coroutine_threadsafe(
                    self.drone.offboard.stop(),
                    self.mavsdk_loop
                )
            except OffboardError as e:
                self.get_logger().error(f"Error offboard: {e}")

        
    def set_offboard_ned_position(self, msg):
        self.get_logger().info(f"Set NED Pos: {msg}")
        self.ned_set_pos = msg
        # Async Funktion thread-sicher ausführen
        try:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        self.ned_set_pos.x,        # North
                        self.ned_set_pos.y,        # Right
                        -1*self.ned_set_pos.z,        # Down
                        self.ned_set_pos.w         # Yaw 
                    )
                ),
                self.mavsdk_loop
            )
        except OffboardError as e:
            self.get_logger().error(f"Ned pos error {e}")
    
    def set_offboard_ned_acc_vel_pos(self,msg):
        self.get_logger().info(f"Set NED Pos: {msg}")
        self.ned_set_acc_vel_pos = msg
        acceleration = self.ned_set_acc_vel_pos.acceleration
        velocity = self.ned_set_acc_vel_pos.velocity
        position = self.ned_set_acc_vel_pos.position
        # Async Funktion thread-sicher ausführen
        try:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(
                        position.x,        # North [m]
                        position.y,        # East  [m]
                        -1*position.z,        # Down  [m]
                        position.w         # Yaw deg [°]
                    ),
                    VelocityNedYaw(
                        velocity.x,        # North [m/s]
                        velocity.y,        # East  [m/s]
                        -1*velocity.z,        # Down  [m/s]
                        velocity.w         # Yaw   [°]
                    ),
                    AccelerationNed(
                        acceleration.x,    # North [m/s²]
                        acceleration.y,    # East  [m/s²]
                        -1*acceleration.z     # Down  [m/s²]
                    )
                ),
                self.mavsdk_loop
            )
        except OffboardError as e:
            self.get_logger().error(f"error {e}")

    def set_offboard_ned_velocity(self, msg):
        self.get_logger().info("Set Ned velocity")
        self.velo = msg
        asyncio.run_coroutine_threadsafe(
            self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    self.velo.x,
                    self.velo.y,
                    -1*self.velo.z,
                    self.velo.w
                )
            ), self.mavsdk_loop)
        
    def set_offboard_body_velocity(self, msg):
        self.get_logger().info("Set Body velocity")
        self.velocity = msg
        asyncio.run_coroutine_threadsafe(
            self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    self.velocity.x,
                    self.velocity.y,
                    -1*self.velocity.z,
                    self.velocity.w
                )
            ), self.mavsdk_loop)
    
    def set_offboard_position_velocity_ned(self,msg):
        self.get_logger().info(f"Set NED Velo Pos: {msg}")
        self.ned_set_acc_vel_pos = msg
        velocity = self.ned_set_acc_vel_pos.velocity
        position = self.ned_set_acc_vel_pos.position
        # Async Funktion thread-sicher ausführen
        try:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(
                        position.x,        # North [m]
                        position.y,        # East  [m]
                        -1*position.z,        # Down  [m]
                        position.w         # Yaw deg [°]
                    ),
                    VelocityNedYaw(
                        velocity.x,        # North [m/s]
                        velocity.y,        # East  [m/s]
                        -1*velocity.z,        # Down  [m/s]
                        velocity.w         # Yaw   [°]
                    )
                ),
                self.mavsdk_loop
            )
        except OffboardError as e:
            self.get_logger().error(f"error {e}")

    def set_action_arm(self, msg):
        # Arm Vehicle
        self.get_logger().info(f"Set arm: {msg.data}")
        self.arm = msg.data
        # Async Funktion thread-sicher ausführen
        if self.arm and not self.action_arm_set:
            self.action_arm_set = True
            self.get_logger().info("Armed Drone")
            try: #TODO Action result
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.arm(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error arm: {e}")

        elif not self.arm and self.action_arm_set:
            self.action_arm_set = False
            self.get_logger().info("Disarmed Drone")
            try: #TODO Action result
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.disarm(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error arm: {e}")
        
    def set_action_land(self, msg):
        # Land Vehicle
        self.get_logger().debug(f"Set FM land: {msg.data}")
        self.land = msg.data
        # Async Funktion thread-sicher ausführen
        if self.land:
            try: #TODO Action result
                self.get_logger().info("Trigger Land")
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.land(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error land: {e}")
    
    def set_action_kill(self, msg):
        # Kill Vehicle
        self.get_logger().debug(f"Set Kill: {msg.data}")
        self.kill = msg.data
        # Async Funktion thread-sicher ausführen
        if self.kill:
            try: #TODO Action result
                self.get_logger("Trigger Kill drone")
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.kill(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error kill: {e}")
    
    def set_action_takeoff(self, msg):
        # Takeoff Vehicle
        self.get_logger().debug(f"Set takeoff: {msg.data}")
        self.takeoff = msg.data
        # Async Funktion thread-sicher ausführen
        if self.takeoff and self.arm :
            try: #TODO Action result
                self.get_logger().info("Trigger Takeoff")
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.takeoff(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error takeoff: {e}")
    
    def set_action_terminate(self, msg):
        # Arm Vehicle
        self.get_logger().debug(f"Set terminate: {msg.data}")
        self.terminate = msg.data
        # Async Funktion thread-sicher ausführen
        if self.terminate:
            self.arm = False
            try: #TODO Action result
                self.get_logger().info("Trigger Terminate Drone")
                asyncio.run_coroutine_threadsafe(
                    self.drone.action.terminate(),
                    self.mavsdk_loop
                )
                
            except ActionError as e:
                self.get_logger().error(f"Error terminate: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = FlyNode()

    # MultiThreadedExecutor erlaubt ROS2 Spin und Async Tasks parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PX4 Communication Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Stop asyncio loop
        pending = asyncio.all_tasks(loop=node.mavsdk_loop)
        for task in pending:
            task.cancel()
        try:
            node.mavsdk_loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        except Exception:
            pass
        node.mavsdk_loop.close()


if __name__ == "__main__":
    main()
