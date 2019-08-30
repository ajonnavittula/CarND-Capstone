from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        RED_MIN = np.array([0, 120, 120],np.uint8)
        RED_MAX = np.array([10, 255, 255],np.uint8)
        binary = cv2.inRange(hsv_img, RED_MIN, RED_MAX)
        red = cv2.countNonZero(binary)
        if red > 50:
          return TrafficLight.RED

        YELLOW_MIN = np.array([30, 120, 120],np.uint8)
        YELLOW_MAX = np.array([50, 255, 255],np.uint8)
        binary = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
        y = cv2.countNonZero(binary)
        if yellow > 50:
          return TrafficLight.YELLOW

        GREEN_MIN = np.array([65, 120, 120],np.uint8)
        GREEN_MAX = np.array([100*255, 255, 255],np.uint8)
        binary = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)
        green = cv2.countNonZero(binary)
        if green > 50:
          return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
