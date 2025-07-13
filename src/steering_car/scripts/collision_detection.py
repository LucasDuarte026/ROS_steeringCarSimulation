#!/usr/bin/env python3
# This script creates a ROS node that performs real-time object detection on a
# camera feed to estimate the distance to the detected objects, specifically targeting vehicles.

# Standard library imports
import rospy
import cv2
import numpy as np

# Third-party imports
from ultralytics import YOLO

# ROS-related imports
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge

# --- Configuration ---
# The known physical width of the objects we are detecting (e.g., an average car).
# This is a key parameter for the distance estimation formula.
KNOWN_OBJECT_WIDTH_METERS = 1.8 

# --- Constants for Visualization (BGR format for OpenCV) ---
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX

class DistanceEstimatorNode:
    """
    A ROS node to detect vehicles, estimate their distance, and visualize the results.
    It dynamically acquires the camera's focal length from a ROS topic.
    """
    def __init__(self):
        """Initializes the ROS node, loads the YOLO model, and sets up subscribers."""
        rospy.init_node('distance_estimator_node', anonymous=True)
        self.bridge = CvBridge()
        
        # --- Class Member Variables ---
        self.focal_length = None # Will be populated by the camera_info_callback
        self.cv_image = None     # Will hold the current camera frame for processing

        # --- Load the YOLOv8 model ---
        # The 'yolov8n.pt' is the nano version, which is fast and suitable for real-time processing.
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')
        rospy.loginfo("YOLOv8 model loaded successfully.")

        # --- ROS Subscribers ---
        # Subscriber for the camera's intrinsic parameters (e.g., focal length).
        # This allows the node to work with different cameras without hardcoding values.
        self.info_sub = rospy.Subscriber(
            "/car/car_camera/camera_info",
            CameraInfo,
            self.camera_info_callback
        )

        # Subscriber for the compressed image feed from the car's camera.
        # Using CompressedImage is more efficient over the network.
        self.image_sub = rospy.Subscriber(
            "/car/car_camera/image_raw/compressed", 
            CompressedImage, 
            self.image_callback,
            queue_size=1,     # Keep only the latest message
            buff_size=2**24   # Set a large buffer size to prevent message drops
        )
        
        rospy.loginfo("Distance Estimator Node has started.")
        rospy.loginfo("Waiting to receive camera info on topic /car/car_camera/camera_info...")

    def camera_info_callback(self, msg):
        """
        Callback to receive the camera's intrinsic parameters just once.
        Extracts the focal length and then unregisters the subscriber.
        """
        if self.focal_length is None:
            # The focal length in pixels (fx) is the first element (0,0) of the camera matrix 'K'.
            self.focal_length = msg.K[0]
            rospy.loginfo(f"Camera focal length acquired: {self.focal_length:.2f} pixels")
            # Unsubscribe after receiving the info to save system resources.
            self.info_sub.unregister()

    def estimate_distance(self, object_pixel_width):
        """
        Estimates the distance to an object based on its perceived width in pixels.
        This uses the triangle similarity principle.
        Formula: Distance = (Known_Width * Focal_Length) / Perceived_Pixel_Width
        """
        # Avoid division by zero if the object width is not detected or focal length is not yet available.
        if object_pixel_width == 0 or self.focal_length is None:
            return float('inf') # Return infinity if distance cannot be calculated
        return (KNOWN_OBJECT_WIDTH_METERS * self.focal_length) / object_pixel_width

    def process_frame_for_collision(self, image, detected_objects):
        """
        Draws bounding boxes and distance information on the image.
        Highlights the closest detected object as a potential collision risk.
        """
        closest_distance = float('inf')
        closest_object_box = None

        # First pass: find the closest object
        for (x, y, w, h) in detected_objects:
            dist = self.estimate_distance(w)
            if dist < closest_distance:
                closest_distance = dist
                closest_object_box = (x, y, w, h)

        # Second pass: draw all objects, highlighting the closest one
        for (x, y, w, h) in detected_objects:
            is_closest = (x, y, w, h) == closest_object_box
            # Use red for the closest object, green for others
            box_color = COLOR_RED if is_closest else COLOR_GREEN
            
            # Draw the bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 2)
            
            # Calculate and display the distance text above the box
            dist = self.estimate_distance(w)
            distance_text = f"{dist:.2f}m"
            cv2.putText(image, distance_text, (x, y - 10), FONT, 0.7, box_color, 2)

        # Display the distance of the closest risk in the corner of the screen
        if closest_distance != float('inf'):
            corner_text = f"Closest Risk: {closest_distance:.2f}m"
        else:
            corner_text = "No objects detected"
        
        cv2.putText(image, corner_text, (30, 40), FONT, 1.0, COLOR_RED, 2)
        return image

    def image_callback(self, ros_data):
        """
        The main callback for processing incoming image frames.
        It decodes the image, runs YOLO detection, and triggers processing and visualization.
        """
        # Do not process any images until the focal length has been acquired.
        if self.focal_length is None:
            rospy.logwarn_throttle(5, "Waiting for camera info... Cannot process image yet.")
            return

        # Decode the compressed image message into an OpenCV image
        try:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Failed to decode compressed image: {e}")
            return

        # --- Perform Object Detection using YOLO ---
        # We specify the classes to detect: [2: 'car', 5: 'bus', 7: 'truck']
        # 'verbose=False' prevents YOLO from printing detection stats to the console.
        results = self.model(self.cv_image, classes=[2, 5, 7], verbose=False)
        
        # Extract bounding box coordinates from the results
        detected_objects = []
        for result in results:
            for box in result.boxes:
                # Get coordinates in (x1, y1, x2, y2) format and convert to (x, y, w, h)
                x1, y1, x2, y2 = box.xyxy[0]
                x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
                detected_objects.append((x, y, w, h))

        # Process the frame with the detection results to add visualizations
        visual_frame = self.process_frame_for_collision(self.cv_image, detected_objects)
        
        # Display the final image in a window
        cv2.imshow("Collision Distance Estimation (Live)", visual_frame)
        cv2.waitKey(1)

def main():
    """Main execution function."""
    try:
        # Instantiate the node class
        DistanceEstimatorNode()
        # rospy.spin() keeps the node alive and listening for callbacks
        rospy.spin()
    except rospy.ROSInterruptException:
        # This block is executed if the node is shut down (e.g., by Ctrl+C)
        rospy.loginfo("ROS interrupt received. Shutting down.")
    finally:
        # Ensure all OpenCV windows are closed upon exiting
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()