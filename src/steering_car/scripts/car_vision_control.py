#!/usr/bin/env python3
# Standard library imports
import cv2
import time

# Third-party imports
import mediapipe as mp

# ROS-related imports
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class HandGestureController:
    """
    This class uses a webcam to detect hand gestures and translate them into
    control commands for a simulated car in ROS. It uses the MediaPipe library
    for hand tracking and publishes effort commands to ROS controllers.
    """
    def __init__(self):
        # --- ROS Initialization ---
        # Initialize the ROS node with a unique name.
        rospy.init_node('hand_gesture_controller', anonymous=True)

        # Publishers for the wheel and steering effort controllers.
        # These topics correspond to the controllers defined in 'controllers.yaml'.
        self.back_left_pub = rospy.Publisher('/car/back_left_wheel_effort_controller/command', Float64, queue_size=1)
        self.back_right_pub = rospy.Publisher('/car/back_right_wheel_effort_controller/command', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/car/front_wheels_effort_controller/command', Float64, queue_size=1)

        # Subscriber to get the current velocity of the wheels.
        # This is essential for the proportional braking logic.
        rospy.Subscriber('/car/joint_states', JointState, self.joint_state_callback, queue_size=1)

        # --- Control Parameters ---
        self.effort_level = 0.3      # Base power for acceleration/reverse.
        self.steering_ratio = 1.5    # Multiplier for how sharply the car turns.
        self.braking_constant = 0.01 # Proportional constant for braking when no hand is detected.

        # --- State Variables ---
        self.back_left_wheel_velocity = 0.0 # Stores the current wheel velocity.
        self.hand_detected = False          # Flag to track if a hand is currently in the frame.

        # --- MediaPipe and Camera Initialization ---
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0) # Initialize webcam capture.

        # Set the frequency of the control loop.
        self.rate = rospy.Rate(20) # 20 Hz

        # Register the cleanup function to be called on node shutdown (e.g., Ctrl+C).
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("=== Hand Gesture Controller Ready ===")
        rospy.loginfo("Move your hand into the camera frame to control the car.")
        rospy.loginfo("Close the window or press Ctrl+C to exit.")

    def joint_state_callback(self, msg):
        """
        Callback function for the /car/joint_states subscriber.
        Updates the current velocity of the back left wheel.
        """
        try:
            # Find the index of the back left wheel joint in the message.
            idx = msg.name.index('back_left_wheel_joint')
            # Store its velocity. We use this as a reference for braking.
            self.back_left_wheel_velocity = msg.velocity[idx]
        except (ValueError, IndexError):
            # This can happen if the joint_state message is not as expected.
            pass

    def detect_gesture_grid(self, x, y, width, height):
        """
        Determines the control command based on the hand's position in a 3x2 grid.
        Returns a tuple (throttle_command, steer_command).
        """
        # Define the vertical line for forward/backward separation (middle of the screen).
        middle_line_y = height * 0.5

        # Define the horizontal lines for left/center/right separation.
        left_bound = width * 0.33
        right_bound = width * 0.66

        # Top Zone (Forward)
        if y < middle_line_y:
            if x < left_bound:
                return "forward", "left"
            elif x > right_bound:
                return "forward", "right"
            else:
                return "forward", "center"
        # Bottom Zone (Backward)
        else:
            if x < left_bound:
                return "backward", "left"
            elif x > right_bound:
                return "backward", "right"
            else:
                return "backward", "center"

    def draw_grid(self, img, width, height):
        """Draws the control grid on the camera image for user feedback."""
        # Calculate grid boundaries.
        middle_line_y = int(height * 0.5)
        left_bound = int(width * 0.33)
        right_bound = int(width * 0.66)

        # Draw a thick horizontal line to separate forward/backward.
        cv2.line(img, (0, middle_line_y), (width, middle_line_y), (0, 255, 255), 2)

        # Draw vertical lines to separate left/center/right.
        cv2.line(img, (left_bound, 0), (left_bound, height), (0, 255, 255), 1)
        cv2.line(img, (right_bound, 0), (right_bound, height), (0, 255, 255), 1)

    def run(self):
        """Main control loop."""
        while not rospy.is_shutdown() and self.cap.isOpened():
            success, img = self.cap.read()
            if not success:
                continue

            # Flip the image horizontally for a more intuitive "mirror" effect.
            img = cv2.flip(img, 1)
            h, w, _ = img.shape
            # Convert the BGR image to RGB for MediaPipe processing.
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            # Default commands if no hand is detected
            throttle_cmd = "brake"
            steer_cmd = "center"
            self.hand_detected = False

            if results.multi_hand_landmarks:
                self.hand_detected = True
                # Get the landmarks for the first detected hand.
                hand_lms = results.multi_hand_landmarks[0]
                # Draw the hand skeleton on the image.
                self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)

                # Use the wrist (landmark 0) as the reference point for position.
                wrist = hand_lms.landmark[0]
                cx, cy = int(wrist.x * w), int(wrist.y * h)

                # Determine commands from the wrist's position in the grid.
                throttle_cmd, steer_cmd = self.detect_gesture_grid(cx, cy, w, h)

            # --- Effort Calculation Logic ---
            throttle_effort = 0.0
            steering_effort = 0.0

            if not self.hand_detected:
                # If no hand is detected, apply proportional braking.
                # The brake effort is opposite to the current velocity.
                throttle_effort = -self.braking_constant * self.back_left_wheel_velocity
                throttle_cmd = "braking (no hand)"
            else:
                # Map text commands to numerical effort values.
                if throttle_cmd == "forward":
                    throttle_effort = -self.effort_level # Negative for forward movement in Gazebo
                elif throttle_cmd == "backward":
                    throttle_effort = self.effort_level  # Positive for backward movement

                if steer_cmd == "left":
                    steering_effort = self.effort_level * self.steering_ratio
                elif steer_cmd == "right":
                    steering_effort = -self.effort_level * self.steering_ratio

            # --- Publish Commands ---
            # Publish the calculated effort to the respective controllers.
            self.back_left_pub.publish(Float64(throttle_effort))
            self.back_right_pub.publish(Float64(throttle_effort))
            self.steer_pub.publish(Float64(steering_effort))

            # --- Visualization ---
            self.draw_grid(img, w, h)
            # Display the current command status on the image.
            display_text = f"Throttle: {throttle_cmd} | Steer: {steer_cmd}"
            cv2.putText(img, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.imshow("Hand Gesture Control", img)

            # Exit the loop if 'q' is pressed.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Maintain the loop frequency.
            self.rate.sleep()

    def cleanup(self):
        """
        Function called on node shutdown to safely stop the car and release resources.
        """
        rospy.loginfo("Shutting down. Stopping the car...")
        # Send a zero effort command to all joints to stop the car.
        self.back_left_pub.publish(Float64(0.0))
        self.back_right_pub.publish(Float64(0.0))
        self.steer_pub.publish(Float64(0.0))

        # Release the camera and destroy all OpenCV windows.
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        controller = HandGestureController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure OpenCV windows are closed even if an unexpected error occurs.
        cv2.destroyAllWindows()