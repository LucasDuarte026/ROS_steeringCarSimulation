#!/usr/bin/env python3
# This script creates a ROS node to control the simulated car using keyboard inputs.
# It provides teleoperation for forward/backward movement, steering, and braking.

# Standard library imports
import threading
import time
import math

# Third-party imports
from pynput import keyboard # A library for monitoring keyboard events

# ROS-related imports
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 

class CarKeyboardController:
    """
    Manages keyboard inputs to publish control commands (effort) to the car's controllers.
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and control parameters."""
        rospy.init_node('car_keyboard_controller', anonymous=True)

        # --- ROS Publishers ---
        # These publishers send effort commands to the controllers defined in controllers.yaml
        self.back_left_pub = rospy.Publisher('/car/back_left_wheel_effort_controller/command', Float64, queue_size=10)
        self.back_right_pub = rospy.Publisher('/car/back_right_wheel_effort_controller/command', Float64, queue_size=10)
        self.steer_pub = rospy.Publisher('/car/front_wheels_effort_controller/command', Float64, queue_size=10)

        # --- ROS Subscriber ---
        # Subscribes to joint states to get current wheel velocity for proportional braking.
        rospy.Subscriber('/car/joint_states', JointState, self.joint_state_callback, queue_size=1)
        
        # --- Control Parameters ---
        self.max_effort = 2.0           # The absolute maximum effort allowed.
        self.effort_level = 0.6         # The base effort for movement, can be adjusted.
        self.steering_ratio = 1.5       # A multiplier to define steering sensitivity.
        self.current_steering = 0.0     # The current steering command value.
        self.effort_change_amount = 0.05 # How much to increase/decrease effort per key press.
        self.braking_constant = 0.01    # Proportional gain for the brake.

        # --- State Variables ---
        self.back_left_wheel_velocity = 0.0
        self.back_right_wheel_velocity = 0.0
        self.last_log_time = time.time() # Used for throttling log messages.
        self.keys_pressed = set()        # A set to keep track of currently held-down keys.
        self.rate = 20                   # The frequency of the main control loop in Hz.
        
        # Display the control instructions in the terminal.
        self.display_controls()

    def display_controls(self):
        """Prints the keyboard control layout to the console."""
        rospy.loginfo("=============================")
        rospy.loginfo("=== Car Keyboard Controller ===")
        rospy.loginfo("=============================")
        rospy.loginfo("Controls:")
        rospy.loginfo("  ↑     - Move forward")
        rospy.loginfo("  ↓     - Move backward")
        rospy.loginfo("  ←     - Turn left")
        rospy.loginfo("  →     - Turn right")
        rospy.loginfo("  [p]   - Apply proportional brake") 
        rospy.loginfo("  [+]   - Increase power (current: %.2f)", self.effort_level)
        rospy.loginfo("  [-]   - Decrease power (current: %.2f)", self.effort_level)
        rospy.loginfo("  [ESC] - Exit controller")
        rospy.loginfo("-----------------------------")

    def joint_state_callback(self, msg):
        """Callback to update wheel velocities from the /car/joint_states topic."""
        try:
            # Find the velocities for the back wheels by their joint names.
            idx_left = msg.name.index('back_left_wheel_joint')
            idx_right = msg.name.index('back_right_wheel_joint')
            self.back_left_wheel_velocity = msg.velocity[idx_left]
            self.back_right_wheel_velocity = msg.velocity[idx_right]
        except ValueError:
            # This might happen if the joint names are not in the message.
            pass
        except Exception as e:
            rospy.logerr(f"Error in joint_state_callback: {e}")

    def on_press(self, key):
        """
        Handles key press events. Adds the pressed key to the `keys_pressed` set
        and adjusts power levels if '+' or '-' is pressed.
        """
        self.keys_pressed.add(key)
        
        if hasattr(key, 'char'):
            if key.char == '+':
                # Increase power, capped at max_effort.
                self.effort_level = min(self.max_effort, self.effort_level + self.effort_change_amount)
                rospy.loginfo(f"Power increased to {self.effort_level:.2f}")
            elif key.char == '-':
                # Decrease power, with a floor value.
                self.effort_level = max(self.effort_change_amount, self.effort_level - self.effort_change_amount)
                rospy.loginfo(f"Power decreased to {self.effort_level:.2f}")

    def on_release(self, key):
        """
        Handles key release events. Removes the key from the `keys_pressed` set.
        Stops the listener if the ESC key is released.
        """
        self.keys_pressed.discard(key)
        if key == keyboard.Key.esc:
            rospy.loginfo("ESC key pressed. Exiting controller...")
            rospy.signal_shutdown("ESC key pressed")
            return False # Stops the pynput listener

    def calculate_wheel_efforts(self):
        """Calculates the forward/backward effort based on currently pressed keys."""
        throttle_effort = 0.0
        # If 'p' is held, apply strong proportional braking.
        if keyboard.KeyCode(char='p') in self.keys_pressed:      
            throttle_effort = -self.braking_constant * self.back_left_wheel_velocity
        # If 'up' is held, apply forward effort. Negative effort moves car forward in this model.
        elif keyboard.Key.up in self.keys_pressed:
            throttle_effort = -self.effort_level
        # If 'down' is held, apply backward effort.
        elif keyboard.Key.down in self.keys_pressed:
            throttle_effort = self.effort_level
        else:
            # If no movement key is pressed, apply a gentle proportional brake to slow down naturally.
            throttle_effort = -self.braking_constant * self.back_left_wheel_velocity * 0.50
        return throttle_effort

    def update_steering(self):
        """Calculates and publishes the steering command."""
        steering_command = 0.0
        # Check for left/right keys, avoiding conflicting inputs.
        if keyboard.Key.left in self.keys_pressed and keyboard.Key.right not in self.keys_pressed:
            steering_command = self.effort_level * self.steering_ratio
        elif keyboard.Key.right in self.keys_pressed and keyboard.Key.left not in self.keys_pressed:
            steering_command = -self.effort_level * self.steering_ratio
        
        self.current_steering = steering_command
        self.steer_pub.publish(Float64(self.current_steering))

    def control_loop(self):
        """
        Main control loop running at a fixed rate.
        It continuously checks pressed keys, calculates efforts, and publishes commands.
        """
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Calculate and publish throttle effort for both back wheels.
            throttle_effort = self.calculate_wheel_efforts()
            self.back_left_pub.publish(Float64(throttle_effort))
            self.back_right_pub.publish(Float64(throttle_effort))
            
            # Update and publish steering command.
            self.update_steering()
            
            rate.sleep()

    def run(self):
        """Starts the keyboard listener and the main control loop in separate threads."""
        # The keyboard listener runs in its own thread to not block the script.
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # The control loop also runs in a separate thread.
        control_thread = threading.Thread(target=self.control_loop)
        control_thread.daemon = True # Allows main thread to exit even if this thread is running
        control_thread.start()
        
        # Keep the main thread alive until ROS is shutdown or the listener stops.
        rospy.spin()

        # --- Cleanup ---
        # This code runs after rospy.spin() exits (e.g., on Ctrl+C or ESC).
        rospy.loginfo("Shutting down. Sending zero-effort commands to stop the car.")
        # Ensure the car stops by publishing zero effort to all controllers.
        self.back_left_pub.publish(Float64(0.0))
        self.back_right_pub.publish(Float64(0.0))
        self.steer_pub.publish(Float64(0.0))
        # Stop the keyboard listener thread.
        listener.stop()
        rospy.loginfo("Controller has been stopped.")

if __name__ == '__main__':
    try:
        controller = CarKeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass