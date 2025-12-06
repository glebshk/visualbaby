# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import psutil

# GStreamer Imports
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GLib, GstRtspServer

# Import OpenCV, Numpy, time, threading and config variables
import cv2
import threading
import numpy as np
import time
import camera_stream.config as cfg
import datetime

# Define latest Frames
latest_frnt_rgb_frame = None
latest_frnt_depth_frame = None
latest_bttn_rgb_frame = None
latest_bttn_depth_frame = None

# Stream Node
class RTSPPublisher(Node):
    def __init__(self):
        super().__init__('RTSP_publisher')

        # Import CV Bridge (Connection between ROS Messages and OpenCV) 
        self.bridge = CvBridge()

        #Declare Parameter
        self.declare_parameter("stream_mount", "IMAV")
        self.declare_parameter("stream_port", 8555)
        
        self.start_timer = self.create_timer(0.2,self.after_init)
        # Subscriber of camera Streams
        #TODO implement depth camera interface node / translation
        self.create_subscription(Image, f'{cfg.ROS_CAMERA_TOPICS_FRNT}rgb', self.image_frnt_rgb_callback, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Image, f'camera/front/depth', self.image_frnt_depth_callback, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Image, f'{cfg.ROS_CAMERA_TOPICS_BTTN}rgb', self.image_bttn_rgb_callback, cfg.ROS_QUEUE_SIZE)
        self.create_subscription(Image, f"camera/bottom/depth",self.image_bttn_depth_callback, cfg.ROS_QUEUE_SIZE)
        
        # Colorizer for depth image
        self.colorizer = rs.colorizer()

        self.atime = 0
        self.btime = 0
        self.num_frames = 0
        self.fps = 0

        # Timer System Info
        self.timer_system_info = self.create_timer(1.0, self.get_system_info)

        # GStreamer Thread
        self.gst_thread = threading.Thread(target=self.gst_loop)
        self.gst_thread.daemon = True
        self.gst_thread.start()

    def after_init(self):
        #Get Parameter
        self.stream_mount = self.get_parameter("stream_mount").value
        self.stream_port = self.get_parameter("stream_port").value

    def image_frnt_rgb_callback(self, msg):
        # Image front Camera Callback Message
        global latest_frnt_rgb_frame
        self.get_logger().debug("get front rgb image")
        
        # ROS Message to CV frame
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.putText(cv_img, f'topic:"camera/front/rgb"',(30,30),cfg.FONT_INTERFACE,cfg.THICKNESS_TOPICS_FONT, cfg.COLOR_INTERFACE_TOPICS, cfg.THICKNESS_TOPICS_FONT)
        latest_frnt_rgb_frame = cv_img
    
    def image_frnt_depth_callback(self, msg):
        # Image front Camera Callback Message
        global latest_frnt_depth_frame
        self.get_logger().debug("get front depth image")
        # ROS Message to CV frame
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

        #Colorized Depth Image
        cv_img = self.colorizer.colorize(cv_img)
        cv2.putText(cv_img, 'topic:"camera/front/depth"',(30,30),cfg.FONT_INTERFACE, cfg.THICKNESS_TOPICS_FONT, cfg.COLOR_INTERFACE_TOPICS, cfg.THICKNESS_TOPICS_FONT)
        latest_frnt_depth_frame = cv_img

    def image_bttn_rgb_callback(self,msg):
        # Image Bottom Camera Callback Message
        global latest_bttn_rgb_frame
        self.get_logger().debug("get bottom rgb image")
        
        #ROS Message to CV frame
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.putText(cv_img, 'topic:"camera/bottom/rgb"',(30,30), cfg.FONT_INTERFACE, cfg.THICKNESS_TOPICS_FONT, cfg.COLOR_INTERFACE_TOPICS, cfg.THICKNESS_TOPICS_FONT)
        latest_bttn_rgb_frame = cv_img

    def image_bttn_depth_callback(self,msg):
        # Image Bottom Camera Callback Message
        global latest_bttn_depth_frame
        self.get_logger().debug("get bottom depth image")
        
        #ROS Message to CV frame
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        
        #Colorized Depth Image
        cv_img = self.colorizer.colorize(cv_img)
        cv2.putText(cv_img, 'topic:"camera/bottom/depth"', (30,30), cfg.FONT_INTERFACE, cfg.THICKNESS_TOPICS_FONT, cfg.COLOR_INTERFACE_TOPICS, cfg.THICKNESS_TOPICS_FONT)
        latest_bttn_depth_frame = cv_img

    def gst_loop(self):
        self.get_logger().info("Start GStreamer Main Loop")

        # Init Gsteamer Server Mainloop
        Gst.init(None)
        loop = GLib.MainLoop()
        self.frame_count = 0
        
        # Init RTSP Server
        server = GstRtspServer.RTSPServer()
        server.props.service = str(cfg.STREAM_PORT)
        server.attach(None)

        # Setup GStreamer Pipline
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(cfg.GSTREAMER_PIPLINE)
        factory.set_shared(True)

        # Mount RTSP Stream
        server.get_mount_points().add_factory(f"/{cfg.STREAM_MOUNT}", factory)
        self.get_logger().info(f"GStreamer Pipline mounted at rtsp://localhost:{cfg.STREAM_PORT}/{cfg.STREAM_MOUNT}")
        
        # Appsrc Zugriff
        def on_media_configure(factory, media):
            # Get Data for Stream
            appsrc = media.get_element().get_child_by_name('mysource')

            # Add frames to queue
            GLib.timeout_add(int(1000/cfg.STREAM_FPS), self.push_frame, appsrc)
        
        # Start Mounted RTSP Stream
        factory.connect("media-configure", on_media_configure)

        # Start GStreamer mainloop/ Thead
        loop.run()
    
    
    def pad_image_to_size(self, img, side):
        # Pad image to size
        h, w = img.shape[:2]
        border_width = max(0, cfg.STREAM_PIXEL_WIDTH - w)
        border_height = max(0, cfg.STREAM_PIXEL_HEIGHT - h)

        if side == "left":
            pad_left = border_width
            pad_right = 0
        else:
            pad_left = 0
            pad_right = border_width

        top = int(border_height / 2)
        bottom = border_height - top

        img_padded = cv2.copyMakeBorder(
            img, top, bottom, pad_left, pad_right,
            borderType=cv2.BORDER_CONSTANT, value=cfg.COLOR_BORDER
        )
        return img_padded
    
    def get_frame(self):
        # Get frame
        global latest_frnt_rgb_frame
        global latest_frnt_depth_frame
        global latest_bttn_rgb_frame
        global latest_bttn_depth_frame

        # Check which camera stream is available
        # camera available [frnt_rgb, frnt_depth, bttn_rgb, bttn_depth]
        # 0 not availabe 1 available
        status_image = [0, 0, 0, 0] 
        
        # Front RGB frame is available
        if latest_frnt_rgb_frame is not None:
            status_image[0] = 1
        
        # Front Depth frame is available
        if latest_frnt_depth_frame is not None:
            status_image[1] = 1

        # Bottom RGB frame is available
        if latest_bttn_rgb_frame is not None:
            status_image[2] = 1
        
        # Bottom Depth frame is available
        if latest_bttn_depth_frame is not None:
            status_image[3] = 1
        
        # No image available
        if status_image == [0,0,0,0]:
            self.get_logger().info("No camera is available")
            return False

        # RGB Cameras available
        if status_image == [1,0,1,0]:
            self.get_logger().debug("Both RGB Cameras are available")
            left = self.pad_image_to_size(latest_frnt_rgb_frame.copy(), "left")
            right = self.pad_image_to_size(latest_bttn_rgb_frame.copy(), "right")
            self.frame = np.vstack((left, right))
        
        # Front RGB Camera 
        elif status_image == [1,0,0,0]:
            self.get_logger().debug("Front RGB Camera available")
            self.frame = latest_frnt_rgb_frame.copy()
        
        # Bottom RGB Camera available
        elif status_image == [0,0,1,0]:
            self.get_logger().debug("Bottom RGB Camera available")
            self.frame = latest_bttn_rgb_frame.copy()
        
        # RGB + Depth Camera front available
        elif status_image == [1,1,0,0]:
            self.get_logger().debug("RGB and Depth Camera front available")
            left = self.pad_image_to_size(latest_frnt_rgb_frame.copy(), "left")
            right = self.pad_image_to_size(latest_frnt_depth_frame.copy(), "right")
            self.frame = np.vstack((left, right))
        
        # RGB + Depth Camera bottom available
        elif status_image == [0,0,1,1]:
            self.get_logger().debug("RGB and Depth Camera bottom available")
            left = self.pad_image_to_size(latest_bttn_rgb_frame.copy(), "left")
            right = self.pad_image_to_size(latest_bttn_depth_frame.copy(), "right")
            self.frame = np.vstack((left, right))

        # RBG front and RGB + depth bottom
        elif status_image == [1,0,1,1]:
            self.get_logger().debug("RGB and Depth Camera bottom + RGB front available")
            frame_left = np.hstack((latest_bttn_rgb_frame.copy(),latest_bttn_depth_frame.copy()))
            left = self.pad_image_to_size(latest_frnt_rgb_frame, "left")
            right = self.pad_image_to_size(frame_left, "right")
            self.frame = np.vstack((left, right))
        
        # RGB + Depth front and RGB bottom
        elif status_image == [1,1,1,0]:
            self.get_logger().debug("RGB and Depth Camera front + RGB bottom available")
            frame_left = np.hstack((latest_frnt_rgb_frame.copy(),latest_frnt_depth_frame.copy()))
            left = self.pad_image_to_size(frame_left, "left")
            right = self.pad_image_to_size(latest_bttn_rgb_frame.copy(), "right")
            self.frame = np.vstack((left, right))

        # RGB + Depth front and bottom
        elif status_image == [1,1,1,1]:
            self.get_logger().debug("RGB and Depth Camera front + RGB and Depth bottom available")
            frame_right = np.hstack((latest_bttn_rgb_frame.copy(),latest_bttn_depth_frame.copy()))
            frame_left = np.hstack((latest_frnt_rgb_frame.copy(),latest_frnt_depth_frame.copy()))
            left = self.pad_image_to_size(frame_left, "left")
            right = self.pad_image_to_size(frame_right, "right")
            self.frame = np.vstack((left, right))
        
        self.num_frames += 1

        if self.num_frames % 5 == 0:
            self.atime = time.time()
            self.fps = round(5/(self.atime - self.btime), 2)
            self.btime = self.atime
            self.num_frames = 0
        self.frame = cv2.resize(self.frame, (cfg.STREAM_PIXEL_WIDTH, cfg.STREAM_PIXEL_HEIGHT))

        self.add_system_info()
    
    
    def get_system_info(self):
        # Get system info and add to frame
        self.cpu_usage = psutil.cpu_percent()
        self.memory = psutil.virtual_memory()
        self.memory_usage = self.memory.percent
        self.cpu_temp = psutil.sensors_temperatures().get('coretemp', [])[0].current
    
    def add_system_info(self):
        # Add system info to frame
        cv2.putText(self.frame,f"TIME:{datetime.datetime.now()}",(30,30),cfg.FONT_INTERFACE,cfg.THICKNESS_FONT,cfg.COLOR_INTERFACE,cfg.THICKNESS_FONT)
        cv2.putText(self.frame,f"FPS:{self.fps}", (30,50),cfg.FONT_INTERFACE,cfg.THICKNESS_FONT,cfg.COLOR_INTERFACE,cfg.THICKNESS_FONT)
        cv2.putText(self.frame,f"CPU:{self.cpu_usage}%",(30,70),cfg.FONT_INTERFACE,cfg.THICKNESS_FONT,cfg.COLOR_INTERFACE,cfg.THICKNESS_FONT)
        cv2.putText(self.frame,f"RAM:{self.memory_usage}%",(30,90),cfg.FONT_INTERFACE,cfg.THICKNESS_FONT,cfg.COLOR_INTERFACE,cfg.THICKNESS_FONT)
        cv2.putText(self.frame,f"TEMP:{self.cpu_temp}C",(30,110),cfg.FONT_INTERFACE,cfg.THICKNESS_FONT,cfg.COLOR_INTERFACE,cfg.THICKNESS_FONT)  
        return True

    def push_frame(self, appsrc):
        ret = self.get_frame()
        if not ret:
            return
        # Push Frame to Gstreamer Pipline 
        # Color Conversion to RGB for Pipeline
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        frame = np.ascontiguousarray(frame)  # sicherstellen, dass contiguous

        # Numpy → Gst.Buffer
        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        
        # Timestamp for GStreamer Pipline
        buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, cfg.STREAM_FPS)
        timestamp = self.frame_count * buf.duration
        buf.pts = buf.dts = timestamp
        buf.offset = timestamp
        self.frame_count += 1

        #Push Frames to GStreamer Pipline
        self.get_logger().debug(f"push frame num frame {self.frame_count}")
        appsrc.emit('push-buffer', buf)
        return True  # für GLib.timeout_add -> wieder aufrufen

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
