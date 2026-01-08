# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String, Float32

# Import OpenCV, Numpy, time, threading and config variables
import cv2
import threading
import numpy as np
import time
from ros_msg.msg import NEDPosVelAcc, Vector4, HEALTH
from geometry_msgs.msg import Vector3
import fly_main.config as cfg

# Stream Node
class RTSPPublisher(Node):
    def __init__(self):
        super().__init__('fly_drone')
        self.state_land = ""
        self.start_fly = False
        self.land_fly = False
        self.health = False
        self.in_air = False
        self.armed = False
        self.ready_to_arm = False
        self.takeoff_succ = False
        # Subscriber of camera Streams
        #TODO implement depth camera interface node / translation
        # Create publisher 
        self.pos_pub = self.create_publisher(Vector4, "/drone/in/offboard/pos/ned", cfg.ROS_QUE_SIZE)
        self.arm_pub = self.create_publisher(Bool, f'/drone/in/arm', cfg.ROS_QUE_SIZE)
        self.takeoff_pub = self.create_publisher(Bool, f'/drone/in/takeoff', cfg.ROS_QUE_SIZE)
        self.offboard = self.create_publisher(Bool, f'/drone/in/offboard/status', cfg.ROS_QUE_SIZE)
        self.pos_vel = self.create_publisher(Vector4,"/drone/in/offboard/body/vel", cfg.ROS_QUE_SIZE)
        self.land_pub = self.create_publisher(Bool, "/drone/in/land", cfg.ROS_QUE_SIZE)
        self.start_timer = self.create_timer(1,self.after_init)
    
    def after_init(self):
        # After Init for clean init of node
        self.destroy_timer(self.start_timer)

        # Create Subscriptions
        self.state_land_sub = self.create_subscription(String, "/drone/out/state/land", self.get_state_land, cfg.ROS_QUE_SIZE)
        self.state_health_sub = self.create_subscription(HEALTH, "/drone/out/health", self.get_state_health, cfg.ROS_QUE_SIZE)

        # Create Time for main Cycle
        self.main_timer = self.create_timer(1/cfg.MAIN_LOOP_HZ,self.check_main)
    
    def arm(self):
        # Trigger arm of Drone
        self.get_logger().info("Drone Armed")
        self.destroy_timer(self.arm_timer)
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
        self.armed = True
    
    def takeoff(self):
        # Trigger Takeoff of drone
        self.get_logger().info("Drone Takeoff")
        self.destroy_timer(self.takeoff_timer)
        self.takeoff_succ = True
        msg = Bool()
        msg.data = True
        self.takeoff_pub.publish(msg)

    def land(self):
        # Trigger Land of drone
        self.get_logger().info("Drone Land")
        self.destroy_timer(self.land_timer)
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
    
    def check_main(self):
        # Main Cycle
        
        if self.health == [True,True,True,True,True,True,True] and not self.ready_to_arm:
            self.get_logger().info("Drone Health ready")
            self.ready_to_arm = True
        
        if not self.armed and not self.start_fly and self.ready_to_arm:
            self.start_fly = True
            self.armed = True
            #self.get_logger().info("start arm")
            self.arm_timer = self.create_timer(5,self.arm)
            self.takeoff_timer = self.create_timer(8,self.takeoff)
            self.land_timer = self.create_timer(28,self.land)
        

        if self.state_land == "ON_GROUND" and self.in_air and not self.land_fly:
            self.land_fly = True
            self.armed = False
            msg = Bool()
            msg.data = False
            self.arm_pub.publish(msg)
        
        
    

    def get_state_land(self, msg):
        # Get state of landing
        #self.get_logger().info(f"sate land {msg.data}")
        self.state_land = msg.data
        if self.state_land == "IN_AIR":
            self.in_air = True
    
    def get_state_health(self, msg):
        #self.get_logger().info(f"health {msg}")
        self.health = [msg.gyro, msg.acc, msg.mag, msg.ned, msg.ned, msg.globalpos, msg.armable]

    

def main(args=None):
    rclpy.init(args=args)
    node = RTSPPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
