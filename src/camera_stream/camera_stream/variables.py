#!/usr/bin/env python3
import cv2
### Values of Streaming Node
debug = False

#Stream Parameters
STREAM_PIXEL_WIDTH = 1280           #Pixel
STREAM_PIXEL_HEIGHT = 720           #Pixel
STREAM_FPS = 20                     # 1/s
STREAM_BITRATE = 1.5                #[MBit/s]
STREAM_MOUNT = "/IMAV"               #Mount on RTSP rtsp://localhost:8554/{STREAM_MOUNT}
STREAM_PORT = 8554                  #Port RSTP stream
STREAM_SPEED_PRESET = "ultrafast"   #Speed Setting
STREAM_OBJECT = "IMAV_rtsp"         #Name of Gstreamer Pipline
STREAM_ENCODING = "x264enc"         #CPU Encoding (other not possible on RPI)
STREAM_FORMAT = "I420"              #Video Format intern


#GStreamer Pipline
GSTREAMER_PIPLINE = (f'appsrc name={STREAM_OBJECT} is-live=true block=true format=time '
                    f'caps=video/x-raw,format={STREAM_FORMAT},width={STREAM_PIXEL_WIDTH},height={STREAM_PIXEL_HEIGHT},'
                    f'framerate={STREAM_FPS}/1 ! {STREAM_ENCODING} speed-preset={STREAM_SPEED_PRESET} tune=zerolatency '
                    f'bitrate={int(STREAM_BITRATE*1000)} ! h264parse ! rtph264pay config-interval=1 name=pay0 pt=96'
)

#Interface Parameters
FONT_INTERFACE = cv2.FONT_HERSHEY_SIMPLEX #Font
COLOR_INTERFACE = (204, 24, 24)     #COLOR RED
COLOR_BORDER = [0, 0, 0]            #COLOR BLACK
THICKNESS_FONT = 1                  #Size Font

#ROS Parameters
ROS_QUEUE_SIZE = 10                  #ROS Queue Size


#Glonal Variables
latest_frame_bttn = None
latest_frame_frnt = None
