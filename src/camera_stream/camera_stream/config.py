#!/usr/bin/env python3
import cv2
### Values of Streaming Node
debug = False

#Stream Parameters
STREAM_PIXEL_WIDTH = 1280           #Pixel
STREAM_PIXEL_HEIGHT = 720           #Pixel
STREAM_FPS = 20                     # 1/s
STREAM_BITRATE = 3                  #[MBit/s]
STREAM_MOUNT = "IMAV"               #Mount on RTSP rtsp://localhost:8554/{STREAM_MOUNT}
STREAM_KEY_FRAMES = 30              #Keypframes for encoding example
STREAM_PORT = 8555                  #Port RSTP stream
STREAM_SPEED_PRESET = "ultrafast"   #Speed Setting
STREAM_OBJECT = "IMAV_rtsp"         #Name of Gstreamer Pipline
STREAM_ENCODING = "x264enc"         #CPU Encoding (other not possible on RPI)
STREAM_FORMAT = "I420"              #Video Format intern


#GStreamer Pipline
GSTREAMER_PIPLINE = (f'appsrc name=mysource is-live=true block=true format=time '
            f'caps=video/x-raw,format=RGB,width={STREAM_PIXEL_WIDTH},height={STREAM_PIXEL_HEIGHT},framerate={STREAM_FPS}/1 ! '
            f'videoconvert ! video/x-raw,format=I420 ! '
            f'x264enc speed-preset={STREAM_SPEED_PRESET} tune=zerolatency key-int-max={STREAM_KEY_FRAMES} byte-stream=true bitrate={int(STREAM_BITRATE*1000)} ! '
            f'h264parse config-interval=1 ! rtph264pay name=pay0 pt=96'
)
#Interface Parameters
FONT_INTERFACE = cv2.FONT_HERSHEY_PLAIN #Font
COLOR_INTERFACE = (0, 255, 100)     #COLOR RED
COLOR_INTERFACE_TOPICS = (0,0,0)    #Color Black
COLOR_BORDER = [0, 0, 0]            #COLOR BLACK
THICKNESS_FONT = 1                  #Size Font
THICKNESS_TOPICS_FONT = 2
#ROS Parameters
ROS_QUEUE_SIZE = 10                  #ROS Queue Size

ROS_CAMERA_TOPICS_BTTN = "camera/bottom/"
ROS_CAMERA_TOPICS_FRNT = "camera/front/"
