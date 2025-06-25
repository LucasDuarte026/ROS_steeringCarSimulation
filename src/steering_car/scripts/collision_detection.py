#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ultralytics import YOLO

# Import the necessary ROS message types
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge

# --- Configuration ---
# We will get the focal length from the /camera_info topic, so no need to hardcode it.
# We still need the known width of the objects we want to detect.
KNOWN_OBJECT_WIDTH_METERS = 1.8 

# --- Color Constants (BGR format for OpenCV) ---
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX

class DistanceEstimatorNode:
    def __init__(self):
        rospy.init_node('distance_estimator_node', anonymous=True)
        self.bridge = CvBridge()
        
        # --- Class Member Variables ---
        self.focal_length = None # Will be populated by the camera_info_callback
        self.cv_image = None

        # --- Load the YOLO model ---
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')
        rospy.loginfo("YOLOv8 model loaded.")

        # --- ROS Subscribers ---
        # Subscriber for the camera's intrinsic parameters
        self.info_sub = rospy.Subscriber(
            "/car/car_camera/camera_info",
            CameraInfo,
            self.camera_info_callback
        )

        # Subscriber for the image feed
        self.image_sub = rospy.Subscriber(
            "/car/car_camera/image_raw/compressed", 
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("Distance Estimator Node has started.")
        rospy.loginfo("Waiting for camera info...")

    def camera_info_callback(self, msg):
        """This callback receives the camera's intrinsic parameters."""
        # The focal length in pixels is the first element of the camera matrix 'K'
        if self.focal_length is None:
            self.focal_length = msg.K[0]
            rospy.loginfo(f"Camera focal length received: {self.focal_length:.2f} pixels")
            # Unregister subscriber after receiving the info once to save resources
            self.info_sub.unregister()

    def estimate_distance(self, object_pixel_width):
        """Estimates distance using the dynamically acquired focal length."""
        if object_pixel_width == 0 or self.focal_length is None:
            return float('inf')
        return (KNOWN_OBJECT_WIDTH_METERS * self.focal_length) / object_pixel_width

    def process_frame_for_collision(self, image, detected_objects):
        # This function is unchanged
        closest_distance = float('inf')
        closest_object_box = None

        for (x, y, w, h) in detected_objects:
            dist = self.estimate_distance(w)
            if dist < closest_distance:
                closest_distance = dist
                closest_object_box = (x, y, w, h)

        for (x, y, w, h) in detected_objects:
            is_closest = (x, y, w, h) == closest_object_box
            box_color = COLOR_RED if is_closest else COLOR_GREEN
            
            cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 2)
            dist = self.estimate_distance(w)
            distance_text = f"{dist:.2f}m"
            cv2.putText(image, distance_text, (x, y - 10), FONT, 0.7, box_color, 2)

        if closest_distance != float('inf'):
            corner_text = f"Closest Risk: {closest_distance:.2f}m"
        else:
            corner_text = "No objects detected"
        
        cv2.putText(image, corner_text, (30, 40), FONT, 1.0, COLOR_RED, 2)
        return image

    def image_callback(self, ros_data):
        # Wait until we have the focal length before processing images
        if self.focal_length is None:
            rospy.logwarn_throttle(5, "Waiting for camera info... Cannot process image yet.")
            return

        try:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
            return

        # --- REAL Object Detection using YOLO ---
        results = self.model(self.cv_image, classes=[2, 5, 7], verbose=False) # car, bus, truck
        
        detected_objects = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
                detected_objects.append((x, y, w, h))

        # Process the frame with the detected objects
        visual_frame = self.process_frame_for_collision(self.cv_image, detected_objects)
        
        cv2.imshow("Collision Distance Estimation (Live)", visual_frame)
        cv2.waitKey(1)

def main():
    try:
        DistanceEstimatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()