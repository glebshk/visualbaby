#!/usr/bin/env python3
import faulthandler
faulthandler.enable(all_threads=True)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import threading
import camera_stream.variables as cfg
import cv2
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GstRtspServer
print("start (module)")

class MyNode(Node):
    def __init__(self):
        print("inti waierjawnode")
        super().__init__('cam_stream')
        print("inti node")
        # --- Parameter ---
        self.declare_parameter("message_text", "Hallo ROS2!")
        self.msg_text = self.get_parameter("message_text").value

        # --- Publisher ---
        self.publisher = self.create_publisher(String, "chatter", 10)

        # Init Bridge for Image Conversion
        self.bridge = CvBridge()

        # Subscription Bottom Camera Image
        self.sub_image_bttn = self.create_subscription(
            Image,
            "Camera/CAM_bottom",
            self.recieve_image_bottom,
            cfg.ROS_QUEUE_SIZE
        )

        # Subscription Front Camera Image
        self.subg_image_frnt = self.create_subscription(
            Image,
            "Camera/CAM_front",
            self.recieve_image_front,
            cfg.ROS_QUEUE_SIZE
        )

        # Start GStreamer RTSP server in background thread (safe)
        # Do the heavy GStreamer imports and init inside this method (not at module import)
        self.gst_ok = False
        self.init_gstreamer()

        self.get_logger().info("Streaming Node started")

    def init_gstreamer(self):
        # Import and initialize gi / Gst here to avoid top-level init-order problems
        try:
            import gi
            gi.require_version('Gst', '1.0')
            gi.require_version('GstRtspServer', '1.0')
            from gi.repository import Gst, GstRtspServer, GLib
        except Exception as e:
            self.get_logger().error(f"Failed to import GStreamer/GI: {e}")
            return

        try:
            Gst.init(None)
        except Exception as e:
            self.get_logger().error(f"Gst.init failed: {e}")
            return

        try:
            self._Gst = Gst
            self._GstRtspServer = GstRtspServer
            self._GLib = GLib

            self.get_logger().info("Starting RTSP Server...")

            # Init GStreamer RTSP Server
            self.STREAM = SensorFactory(self._Gst)
            self.STREAM.set_shared(True)
            self.server = self._GstRtspServer.RTSPServer()
            self.mounts = self.server.get_mount_points()
            self.mounts.add_factory(cfg.STREAM_MOUNT, self.STREAM)
            self.server.attach(None)

            # create and run GLib.MainLoop in background daemon thread
            self.loop = self._GLib.MainLoop()
            self.stream_thread = threading.Thread(target=self.loop.run, daemon=True)
            self.stream_thread.start()

            self.gst_ok = True
            self.get_logger().info("GStreamer main loop started in background thread")
        except Exception as e:
            self.get_logger().error(f"Failed to start GStreamer RTSP server: {e}")
            self.gst_ok = False

    def recieve_image_bottom(self, msg):
        # Convert ROS Image to OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cfg.latest_frame_bttn = np.ascontiguousarray(cv_image, dtype=np.uint8)
        except Exception as e:
            self.get_logger().error(f"receive_image_bottom: {e}")

    def recieve_image_front(self, msg):
        # Convert ROS Image to OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cfg.latest_frame_frnt = np.ascontiguousarray(cv_image, dtype=np.uint8)
        except Exception as e:
            self.get_logger().error(f"receive_image_front: {e}")

    def timer_callback(self):
        msg = String()
        msg.data = self.msg_text
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishe: '{msg.data}'")


class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, Gst):
        super().__init__()
        self.Gst = Gst
        self.number_frames = 0
        self.launch = cfg.GSTREAMER_PIPLINE

    def do_create_element(self, url):
        self.get_logger().info(f"Start RTSP stream")
        element = self.Gst.parse_launch(self.launch)
        appsrc = element.get_child_by_name(cfg.STREAM_OBJECT)
        if appsrc:
            appsrc.connect('need-data', self.on_need_data)
        return element

    def on_need_data(self, src, length):
        try:
            latest_b = cfg.latest_frame_bttn
            latest_f = cfg.latest_frame_frnt

            status_image = [0, 0]
            if latest_b is None:
                status_image[0] = 1
            if latest_f is None:
                status_image[1] = 1

            if status_image == [1, 1]:
                if cfg.debug:
                    self.get_logger().error("No camera stream available! both frames missing")
                return

            if status_image == [0, 0]:
                left = self.pad_image_to_size(latest_f.copy(), "left")
                right = self.pad_image_to_size(latest_b.copy(), "right")
                frame = np.vstack((left, right))
            elif status_image == [1, 0]:
                frame = latest_b.copy()
            elif status_image == [0, 1]:
                frame = latest_f.copy()
            else:
                return

            frame = np.ascontiguousarray(frame, dtype=np.uint8)

            size = frame.nbytes
            if size == 0:
                return

            buf = self.Gst.Buffer.new_allocate(None, size, None)
            buf.fill(0, frame.tobytes())
            buf.duration = self.Gst.util_uint64_scale_int(1, self.Gst.SECOND, cfg.STREAM_FPS)
            timestamp = self.number_frames * buf.duration
            buf.pts = buf.dts = timestamp
            buf.offset = timestamp
            self.number_frames += 1

            if cfg.debug:
                print(f"[#]Pushing frame #{self.number_frames}")

            src.emit('push-buffer', buf)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"on_need_data exception: {e}")

    def pad_image_to_size(self, img, side):
        h, w = img.shape[:2]
        border_width = max(0, cfg.STREAM_PIXEL_WIDTH - w)
        border_height = max(0, cfg.STREAM_PIXEL_HEIGHT - h)

        if side == "left":
            pad_left = border_width
            pad_right = 0
        else:
            pad_left = 0
            pad_right = border_width

        top = border_height // 2
        bottom = border_height - top

        img_padded = cv2.copyMakeBorder(
            img, top, bottom, pad_left, pad_right,
            borderType=cv2.BORDER_CONSTANT, value=cfg.COLOR_BORDER
        )
        return img_padded

def main(args=None):
    rclpy.init()
    print("init node")
    node = MyNode()
    print("node init")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
