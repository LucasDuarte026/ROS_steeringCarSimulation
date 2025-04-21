#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def steer_car():
    rospy.init_node('steer_control', anonymous=True)
    
    # Create publishers for each steering controller
    left_steer_pub = rospy.Publisher('/car/front_left_steering_controller/command', Float64, queue_size=10)
    right_steer_pub = rospy.Publisher('/car/front_right_steering_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    counter =0
    while not rospy.is_shutdown():
        # Example: simple sinusoidal steering pattern
        counter+=1
        angle = 0.3 * (rospy.get_time() % 6.28)  # Varies between -0.3 and 0.3
        print("testando ", counter, " Angle: ",angle)
        left_steer_pub.publish(angle)
        right_steer_pub.publish(angle)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        steer_car()
    except rospy.ROSInterruptException:
        pass