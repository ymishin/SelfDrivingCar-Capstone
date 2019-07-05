from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #
        # Adapted from https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/        
        #

        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# Threshold HSV image and keep only red pixels
        lower_red_hue_range = cv2.inRange(hsv_image, np.array([0, 100, 100]), np.array([10, 255, 255]))
        upper_red_hue_range = cv2.inRange(hsv_image, np.array([160, 100, 100]), np.array([179, 255, 255]))

        # Combine images
        red_hue_image = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)

        # Gaussian blur to avoid false positives
        #red_hue_image = cv2.GaussianBlur(red_hue_image, (9, 9));

        # Apply Circle Hough Transform to detect circles
        circles = cv2.HoughCircles(red_hue_image, cv2.HOUGH_GRADIENT, 1, 30, param1=20, param2=10, minRadius=5, maxRadius=50)

        if circles is not None:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
