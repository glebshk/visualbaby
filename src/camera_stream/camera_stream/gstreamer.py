#!/usr/bin/env python3
# GSTreamer RTSP Server for streaming camera frames
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
Gst.init(None)

import camera_stream.variables as cfg
import numpy as np
import cv2
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, cam):
        super().__init__()
        self.number_frames = 0
        self.cam = cam
        self.set_launch(cfg.GSTREAMER_PIPLINE)
    
    def do_create_element(self, url):
        element = Gst.parse_launch(self.get_launch())
        appsrc = element.get_child_by_name(cfg.STREAM_OBJECT)
        if appsrc:
            appsrc.connect('need-data', self.on_need_data)
        return element

    def on_need_data(self, src, length):
        global cfg
        
        # Check if frame is available
        status_image = [0,0]
        if cfg.latest_frame_bttn is None:
            status_image[0] = 1
        
        if cfg.latest_frame_frnt is None:
            status_image[1] = 1
        
        # Check if camera stream is available
        if 1 not in status_image:
            print("No camera stream available!")
            return

        # Create frame based on available cameras
        # Stack images vertically
        if status_image == [1,1]:
            # Padding to streaming size
            self.latest_frame_frnt = self.pad_image_to_size(cfg.latest_frame_frnt.copy(), "left")
            self.latest_frame_bttn = self.pad_image_to_size(cfg.latest_frame_bttn.copy(), "right")
            
            # Stack images
            frame = np.vstack((self.latest_frame_frnt, self.latest_frame_bttn))
        
        # Single camera available
        elif status_image == [1,0]:
            frame = cfg.latest_frame_bttn.copy()
        elif status_image == [0,1]:
            frame = cfg.latest_frame_frnt.copy()

        # Bytes vorbereiten
        buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
        buf.fill(0, frame.tobytes())
        buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, cfg.STREAM_FPS)
        timestamp = self.number_frames * buf.duration
        buf.pts = buf.dts = timestamp
        buf.offset = timestamp
        self.number_frames += 1

        # Debug: prüfen ob Frames gepusht werden
        if cfg.debug:
            print(f"[#]Pushing frame #{self.number_frames}")

        # Push buffer to gstreamer
        src.emit('push-buffer', buf)
    
    def pad_image_to_size(self, img, side):
        # Pad image to streaming size with border
        h, w = img.shape[:2]

        # Calculate padding
        border_width = cfg.STREAM_PIXEL_WIDTH - w
        if side == "left":
            pad_left = border_width
            pad_right = 0
        else:
            pad_left = 0
            pad_right = border_width    
        
        border_height = cfg.STREAM_PIXEL_HEIGHT - h
        
        if border_height < 0:
            border_height = 0
        if border_width < 0:
            border_width = 0    
        
        # Create padded image
        img_padded = cv2.copyMakeBorder(img, border_height/2, border_height/2, pad_left, pad_right,
                                            borderType=cv2.BORDER_CONSTANT, value=cfg.COLOR_BORDER)
        return img_padded