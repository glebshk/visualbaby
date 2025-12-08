# Config file for camera_node
USB_HUB_AVAILABILITY = False            # Set to True if a USB hub is used

# Camera Parameters
CAM_PIXEL_WIDTH = 640               #Pixel
CAM_PIXEL_HEIGHT = 480              #Pixel
CAM_FPS = 30                        # 1/s
CAM_EXPOSURE = 3700                  # Exposure time for USB Camera
CAM_AUTO_EXPOSURE = 1                # Auto Exposure for USB Camera (0=off, 1=on)

# IMU Parameters
GYRO_FPS_HZ = 200                      # IMU Frequency
ACCEL_RATE_HZ = 63                    # Acceleration Frequency
IMU_FPS = 63                        # IMU Frequency for publishing

# ROS2 Parameters
ROS_MESSSAGE_QUEUE_SIZE = 10         # Size of the message queue for publishers and subscribers
