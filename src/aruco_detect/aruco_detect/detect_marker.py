#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy_message_converter import message_converter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import aruco_detect.config as cfg

class DetectorNode(Node):
    def __init__(self):
        # Node init
        super().__init__('detect_node')

        # Declare Parameter
        self.get_logger().info("Detector Node start")
        self.declare_parameter("message_text", "Default message")
        self.declare_parameter("listen_on_topic", "camera/bottom/rgb")
        self.declare_parameter("camera_type","USB_CAM")
        self.declare_parameter("camera_serial","0")

        # Import CV Bridge for ROS to CV2 
        self.bridge = CvBridge()
        
        # Define timer for after init
        self.start_timer = self.create_timer(0.2, self.after_init)

    def after_init(self):
        # Init Parameter and Subscribtion
        # Destroy Timer
        self.destroy_timer(self.start_timer)

        # Get Parameter
        self.camera_topic = self.get_parameter("listen_on_topic").value
        self.camera_type = self.get_parameter("camera_type").value
        self.serial = self.get_parameter("camera_serial").value

        # Import Camera Parameter
        self.import_camera_parameter()

        # Setup Subscribtion
        self.create_subscription(Image, f'{self.camera_topic}', self.image_callback, cfg.ROS_QUEUE_SIZE)
        
        # Setup Publisher
        # Marker Dict
        self.pub_marker = self.create_publisher(String,f"marker/{self.camera_topic}", cfg.ROS_QUEUE_SIZE)
        self.get_logger().info(f'Publish marker dict on "marker/{self.camera_topic}"')
        
        # Image to Stream
        pub_img_topic = f"stream/{self.camera_topic.replace('camera/','').replace('/rgb','')}/rgb"
        self.pub_image = self.create_publisher(Image,f"{pub_img_topic}", cfg.ROS_QUEUE_SIZE)
        self.get_logger().info(f'Publish marker dict on "{pub_img_topic}"')

    def import_camera_parameter(self): # Import Camera parameter
        self.get_logger().info("Import Camera Parameters")
        try:
            # Import from yamlfile camera matrix
            cv_file = cv2.FileStorage(f"./src/camera_calibration/calibration_{self.camera_type}_{self.serial}.yaml", cv2.FILE_STORAGE_READ) 
            self.camera_matrix = cv_file.getNode('K').mat()
            self.distance_coefficients = cv_file.getNode('D').mat()
            cv_file.release()
        except:
            self.get_logger().error(f"No Calibration file found for {self.camera_type}")
        

    def image_callback(self,msg): # Image detection
        # Transform ROS Message to Open CV Frame
        self.get_logger().debug("aruco get image")
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Detect Aruco Marker
        self.aruco_corners, self.aruco_ids, _ = cv2.aruco.detectMarkers(self.frame,cfg.MARKER_DICT, parameters=cfg.MARKER_PARAMETER)

        self.marker_list = {}
        if self.aruco_ids is not None:
            self.get_logger().debug(f"Aruco marker ids {self.aruco_ids}")

            #Iterate throw the detected marker
            for i,marker_id in enumerate(self.aruco_ids.flatten()):
                
                # Check if Marker is in Marker list
                if marker_id in cfg.MARKER_SIZE_LIST:
                    self.get_logger().info(f"Marker ID {marker_id} detected")
                    # Calculate Position and Rotation matrix of Aruco Marker from Camera in mm
                    try:
                        marker_rvecs, marker_tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.aruco_corners[i],cfg.MARKER_SIZE_LIST[str(marker_id)],
                                                                                        self.camera_matrix,self.distance_coefficients)
                        
                        # Draw Axis and Info on Frame
                        cv2.aruco.drawAxis(self.frame, self.camera_matrix, self.distance_coefficients, marker_rvecs, marker_tvecs, 50)
                        cv2.putText(self.frame, f"ID: {marker_id}", (int(self.aruco_corners[i][0][0][0]), int(self.aruco_corners[i][0][0][1]-15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                        cv2.putText(self.frame, f"Pos: {marker_tvecs[0][0]}", (int(self.aruco_corners[i][0][0][0]), int(self.aruco_corners[i][0][0][1]-35)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                        
                        # Save marker info in dict
                        self.marker_list[f"{marker_id}"] = [marker_tvecs,marker_rvecs]
                    except:
                        self.get_logger().error("Position estimation not possible")
            
            self.get_logger().debug("send marker msg")
            # Publish Marker dict to ROS
            msg = message_converter.convert_dictionary_to_ros_message('std_msgs/msg/String', self.marker_list)
            self.pub_marker.publish(msg)

            # Draw Marker in CV Frame 
            cv2.aruco.drawDetectedMarkers(self.frame,self.aruco_corners,self.aruco_ids)
        
        # Convert Open CV Frame to ROS Message
        img_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        
        # Publish Image as ROS Message
        self.pub_image.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
