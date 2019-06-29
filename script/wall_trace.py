#!/usr/bin/env python
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallTrace():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.accel = rospy.get_param("/run_corridor/acceleration")
        self.max_speed = rospy.get_param("/run_corridor/max_speed")
        self.min_speed = rospy.get_param("/run_corridor/min_speed")
        self.servo_target = rospy.get_param("/run_corridor/servo_target")
        self.servo_kp = rospy.get_param("/run_corridor/servo_kp")
        
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self, messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()
        
        data.linear.x = 0.0
        while not rospy.is_shutdown():
            s = self.sensor_values
            data.linear.x += self.accel
            
            if s.sum_forward >= 50:
                data.linear.x = 0.0
            elif data.linear.x <= self.min_speed:
                data.linear.x = self.min_speed
            elif data.linear.x >= self.max_speed:
                data.linear.x = self.max_speed
            
            if data.linear.x < self.min_speed:
                data.angular.z = 0.0
            elif s.left_side < 10:
                data.angular.z = 0.0
            else:
                error = self.servo_target - s.left_side
                data.angular.z = error * self.servo_kp * math.pi / 180.0
            
            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_trace')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    WallTrace().run()
