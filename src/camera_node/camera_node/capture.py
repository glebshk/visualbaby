#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import camera_node.config as cfg
import pyrealsense2 as rs
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('my_python_node')

        # Declare Parameters
        self.get_logger().info("Streaming Node start")
        self.declare_parameter("message_text", "Default message")
        self.declare_parameter("camera_topic", "camera/front/rgb")
        self.declare_parameter("camera_type", "USBCam")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("camera_serial", "0")
        self.declare_parameter("camera_rotation",0)        
        
        self.start_timer = self.create_timer(0.1, self.init_after_init)  # Delay init to allow parameter setting
    
    def init_after_init(self):

        self.destroy_timer(self.start_timer)
        self.msg_text = self.get_parameter("message_text").value

        # --- Publisher ---
        self.get_logger().info(f"Publishing to topic: {self.get_parameter('camera_topic').value}")  
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera = self.create_publisher(Image, self.camera_topic, cfg.ROS_MESSSAGE_QUEUE_SIZE)
        
        #Init Bridge for Image Conversion
        self.bridge = CvBridge()

        # --- Timer ---
        self.timer = self.create_timer(1.0 / cfg.CAM_FPS, self.timer_callback)
        

        # Get Camera Parameters
        self.camera_type = self.get_parameter("camera_type").value
        self.camera_ID = self.get_parameter("camera_id").value
        self.camera_serial = self.get_parameter("camera_serial").value
        self.camera_rotation = self.get_parameter("camera_rotation").value
        
        # Test camera_rotation value
        if not self.camera_rotation == 180 and not self.camera_rotation == 90 and not self.camera_rotation == 0:
           
            self.get_logger().info(f"Camera Rotation angle of {self.camera_rotation}° is not possible")
            self.get_logger().info("Camera angle is set of 0°")
            self.camera_rotation = 0

        # Initialize Camera
        if self.camera_type == "Realsense":
            camera_topic_depth = self.get_parameter("camera_topic").value.replace("rgb","") + "depth"
            self.get_logger().info(f"Publishing to topic: {self.get_parameter('camera_topic').value.replace('rgb','depth')}") 
            self.camera_depth = self.create_publisher(Image,camera_topic_depth, cfg.ROS_MESSSAGE_QUEUE_SIZE)

            # Setup IMMU Publishers
            self.camera_aceleration = self.create_publisher(Float32MultiArray,f"{self.get_parameter('camera_topic').value.replace('rgb','acceleration')}", cfg.ROS_MESSSAGE_QUEUE_SIZE)
            self.camera_gyro = self.create_publisher(Float32MultiArray,f"{self.get_parameter('camera_topic').value.replace('rgb','gyro')}", cfg.ROS_MESSSAGE_QUEUE_SIZE)
            
            if cfg.USB_HUB_AVAILABILITY:
                # IMU Timer callback
                self.imu_timer = self.create_timer(1.0 / cfg.IMU_FPS, self.imu_timer_callback)

            # Initialize Realsense Camera
            self.init_reasense()

        elif self.camera_type == "USBCam":
            self.init_USBCam()
        
        else:
            self.get_logger().error("Camera Type not recognized!")
            return
        

    def init_reasense(self):
        # 1. Pipeline und Konfiguration erstellen
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Enable Camera by Serial Number
        config.enable_device(self.camera_serial)
        config.enable_stream(rs.stream.color, cfg.CAM_PIXEL_WIDTH, cfg.CAM_PIXEL_HEIGHT, rs.format.bgr8, cfg.CAM_FPS)
        config.enable_stream(rs.stream.depth, cfg.CAM_PIXEL_WIDTH, cfg.CAM_PIXEL_HEIGHT, rs.format.z16, cfg.CAM_FPS)

        # USB Power limitation -> not possible without powered USB Hub
        if cfg.USB_HUB_AVAILABILITY:
            # Enable IMU streams
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, cfg.ACCEL_RATE_HZ)
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, cfg.GYRO_FPS_HZ)
        
        try:
            # 2. Pipeline starten
            self.pipeline.start(config)
            self.get_logger().info(f"Realsense camera {self.camera_serial} initialized")
        except:
            self.get_logger().error("Init Depth camera failed")


    def init_USBCam(self):
        try:
            self.cap_USB = cv2.VideoCapture(self.camera_ID)
            self.cap_USB.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.CAM_PIXEL_WIDTH)
            self.cap_USB.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.CAM_PIXEL_HEIGHT)
            self.cap_USB.set(cv2.CAP_PROP_AUTO_EXPOSURE, cfg.CAM_AUTO_EXPOSURE)  # manuell
            self.cap_USB.set(cv2.CAP_PROP_EXPOSURE, cfg.CAM_EXPOSURE)          # Belichtungszeit
            self.cap_USB.set(cv2.CAP_PROP_FPS, cfg.CAM_FPS)
        
        except:
            self.get_logger().error(" Init Camera failure {e}")


    def timer_callback(self):
        if self.camera_type == "Realsense":
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame:
                return
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Rotation of frame
            if self.camera_rotation == 180:
                color_image = cv2.rotate(color_image, cv2.ROTATE_180)
                depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)
            
            elif self.camera_rotation == 90:
                color_image = cv2.rotate(color_image, cv2.ROTATE_90)
                depth_image = cv2.rotate(depth_image, cv2.ROTATE_90)

            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.camera.publish(img_msg)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            self.camera_depth.publish(depth_msg)

        elif self.camera_type == "USBCam":
            ret, frame = self.cap_USB.read()
            if not ret:
                return

            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            #self.get_logger().info("push frame capture")
            self.camera.publish(img_msg)
    
    def imu_timer_callback(self):
        # Get IMU frames
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        
        # Publish Accel Data
        if accel_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            accel_msg = Float32MultiArray()
            accel_msg.data = [accel_data.x, accel_data.y, accel_data.z]
            self.camera_aceleration.publish(accel_msg)

        # Publish Gyro Data
        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            gyro_msg = Float32MultiArray()
            gyro_msg.data = [gyro_data.x, gyro_data.y, gyro_data.z]
            self.camera_gyro.publish(gyro_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
