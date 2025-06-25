#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 
from pynput import keyboard
import threading
import time
import math

class CarKeyboardController:
    def __init__(self):
        rospy.init_node('car_keyboard_controller', anonymous=True)

        # Publishers
        self.back_left_pub = rospy.Publisher('/car/back_left_wheel_effort_controller/command', Float64, queue_size=10)
        self.back_right_pub = rospy.Publisher('/car/back_right_wheel_effort_controller/command', Float64, queue_size=10)
        self.steer_pub = rospy.Publisher('/car/front_wheels_effort_controller/command', Float64, queue_size=10)

        # Subscriber
        rospy.Subscriber('/car/joint_states', JointState, self.joint_state_callback, queue_size=1)
        
        # Parameters
        self.max_effort = 2.0
        self.effort_level = 0.6
        self.steering_ratio = 1.5
        self.current_steering = 0.0
        self.effort_change_amount = 0.05
        self.braking_constant = 0.01

        # State Variables
        self.back_left_wheel_velocity = 0.0
        self.back_right_wheel_velocity = 0.0
        self.last_log_time = time.time()
        self.keys_pressed = set()
        self.rate = 20   # 20 Hz
        self.display_controls()

    def display_controls(self):
        """Display control instructions to the user"""
        rospy.loginfo("=== Car Keyboard Controller ===")
        rospy.loginfo("Controls:")
        rospy.loginfo("  ↑     - Move forward")
        rospy.loginfo("  ↓     - Move backward")
        rospy.loginfo("  ←     - Turn left")
        rospy.loginfo("  →     - Turn right")
        rospy.loginfo("  p     - Apply proportional brake") 
        rospy.loginfo("  +     - Increase power (current: %.1f)", self.effort_level)
        rospy.loginfo("  -     - Decrease power (current: %.1f)", self.effort_level)
        rospy.loginfo("  ESC   - Exit controller")

    def joint_state_callback(self, msg):
        """Callback function for joint state messages."""
        try:
            for i, name in enumerate(msg.name):
                if name == 'back_left_wheel_joint':
                    self.back_left_wheel_velocity = msg.velocity[i]
                elif name == 'back_right_wheel_joint':
                    self.back_right_wheel_velocity = msg.velocity[i]
        except Exception as e:
            rospy.logerr(f"Error in joint_state_callback: {e}")

    def on_press(self, key):
        """Handle key press events"""
        try:
            if (key in [keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right] or
                        (hasattr(key, 'char') and key.char in ['+', '-','p'])):
                self.keys_pressed.add(key)

                if hasattr(key, 'char'):
                    if key.char == '+':
                        self.effort_level = min(self.max_effort, self.effort_level + self.effort_change_amount)
                        rospy.loginfo(f"Power increased to {self.effort_level:.5f}")
                    elif key.char == '-':
                        self.effort_level = max(self.effort_change_amount, self.effort_level - self.effort_change_amount)
                        rospy.loginfo(f"Power decreased to {self.effort_level:.5f}")

        except Exception as e:
            rospy.logerr(f"Error on key press: {e}")

    def on_release(self, key):
        """Handle key release events"""
        try:
            if key in self.keys_pressed:
                self.keys_pressed.discard(key)


            if key == keyboard.Key.esc:
                rospy.loginfo("Exiting controller...")
                return False
        except Exception as e:
            rospy.logerr(f"Error on key release: {e}")

    def calculate_wheel_efforts(self):
        """Calculate wheel efforts based on pressed keys"""
        actual_effort = 0.0
        if keyboard.KeyCode(char='p') in self.keys_pressed:      
            actual_effort  = -self.braking_constant * self.back_left_wheel_velocity
        elif keyboard.Key.up in self.keys_pressed:
            actual_effort = -self.effort_level
        elif keyboard.Key.down in self.keys_pressed:
            actual_effort  = self.effort_level
        else:
            actual_effort  = -self.braking_constant * self.back_left_wheel_velocity * 0.50
        return actual_effort

    def update_steering(self):
        """Update steering angle based on pressed keys"""
        steering_command = 0.0
        if keyboard.Key.left in self.keys_pressed and keyboard.Key.right not in self.keys_pressed:
            steering_command = self.effort_level*self.steering_ratio
            self.current_steering = self.effort_level*self.steering_ratio
        elif keyboard.Key.right in self.keys_pressed and keyboard.Key.left not in self.keys_pressed:
            steering_command = -self.effort_level*self.steering_ratio
            self.current_steering = -self.effort_level*self.steering_ratio
        self.steer_pub.publish(Float64(steering_command))

    def control_loop(self):
        """Main control loop that publishes wheel commands"""
        rate = rospy.Rate(self.rate)
        prev_effort =  0.0
        prev_steering = 0.0

        try:
            while not rospy.is_shutdown():

                
                  # Calculate normal effort from arrow keys
                actual_effort = self.calculate_wheel_efforts()

                self.update_steering()

                self.back_left_pub.publish(Float64(actual_effort))
                self.back_right_pub.publish(Float64(actual_effort))
                
                if abs(self.current_steering - prev_steering) > 0.01:
                    rospy.loginfo(f"Steering effort: {self.current_steering:.1f}°")
                    prev_steering = self.current_steering

                current_time = time.time()
                if current_time - self.last_log_time >= 1.0:
                # print(f"Left wheel velocity: {self.back_left_wheel_velocity:.2f} | actual_effort : {actual_effort:.2f}")
                    self.last_log_time = current_time
                
                rate.sleep()
        
        except rospy.ROSInterruptException:
            pass

    def run(self):
        """Start the controller"""
        keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        keyboard_listener.start()

        control_thread = threading.Thread(target=self.control_loop)
        control_thread.daemon = True
        control_thread.start()

        try:
            while keyboard_listener.is_alive() and not rospy.is_shutdown():
                time.sleep(0.1)
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
        finally:
            self.back_left_pub.publish(Float64(0.0))
            self.back_right_pub.publish(Float64(0.0))
            self.steer_pub.publish(Float64(0.0))
            keyboard_listener.stop()
            rospy.loginfo("Controller stopped")

if __name__ == '__main__':
    try:
        controller = CarKeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass