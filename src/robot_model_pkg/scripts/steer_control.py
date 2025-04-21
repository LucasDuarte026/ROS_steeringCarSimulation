#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from math import sin

def car_control():
    rospy.init_node('car_control')
    
    # Steering control
    fl_steer = rospy.Publisher('/car/front_left_steering_controller/command', Float64, queue_size=1)
    fr_steer = rospy.Publisher('/car/front_right_steering_controller/command', Float64, queue_size=1)
    
    # Velocity control
    bl_vel = rospy.Publisher('/car/back_left_velocity_controller/command', Float64, queue_size=1)
    br_vel = rospy.Publisher('/car/back_right_velocity_controller/command', Float64, queue_size=1)
    
    rate = rospy.Rate(10)
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        
        # Steering (sinusoidal for testing)
        steer_angle = 0.3 * sin(t)
        fl_steer.publish(steer_angle)
        fr_steer.publish(steer_angle)
        
        # Constant velocity
        velocity = 2.0  # rad/s
        bl_vel.publish(velocity)
        br_vel.publish(velocity)
        
        rospy.loginfo(f"Steering: {steer_angle:.2f} rad, Velocity: {velocity:.2f} rad/s")
        rate.sleep()

if __name__ == '__main__':
    try:
        car_control()
    except rospy.ROSInterruptException:
        pass