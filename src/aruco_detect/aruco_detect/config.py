import cv2
# ---------- Open CV -----------------
MARKER_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)    # Define Dictionary of aruco detection
#TODO Open CV 4.6 Paramter not newer
MARKER_PARAMETER = cv2.aruco.DetectorParameters_create()                        # Get parameter of arucomarker only Opencv 4.6 
MARKER_SIZE_LIST = {
    "100": 50                      # [id, size in mm]
}

# ROS Parameter
ROS_QUEUE_SIZE = 10