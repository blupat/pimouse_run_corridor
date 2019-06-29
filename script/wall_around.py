#!/usr/bin/env python
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallAround():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.accel = rospy.get_param("/run_corridor/acceleration")
        self.max_speed = rospy.get_param("/run_corridor/max_speed")
        self.min_speed = rospy.get_param("/run_corridor/min_speed")
        self.servo_target = rospy.get_param("/run_corridor/servo_target")
        self.servo_kp = rospy.get_param("/run_corridor/servo_kp")
        self.servo_kd = rospy.get_param("/run_corridor/servo_kd")
        self.servo_off_threshold = rospy.get_param("/run_corridor/servo_off_threshold")
        self.wall_gain = rospy.get_param("/run_corridor/wall_gain")
        self.wall_threshold = rospy.get_param("/run_corridor/wall_threshold")
        
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self, messages):
        self.sensor_values = messages

    def wall_front(self, ls):
        return (ls.left_forward > self.wall_threshold) or (ls.right_forward > self.wall_threshold)
    
    def too_right(self, ls):
        return (ls.right_side > self.wall_threshold)
    
    def too_left(self, ls):
        return (ls.left_side > self.wall_threshold)

    def run(self):
        rate = rospy.Rate(50)
        data = Twist()
        previousError = 0.0
        previousTime = rospy.get_time()
        rate.sleep()
        
        data.linear.x = self.min_speed
        while not rospy.is_shutdown():
            ls = self.sensor_values
            
            nowTime = rospy.get_time()
            if self.wall_front(ls):
                data.linear.x -= self.accel
                data.angular.z = - math.pi * self.wall_gain
            elif self.too_right(ls):
                data.linear.x -= self.accel
                data.angular.z = math.pi * self.wall_gain
            elif self.too_left(ls):
                data.linear.x -= self.accel
                data.angular.z = - math.pi * self.wall_gain
            elif ls.left_side < self.servo_off_threshold:
                data.linear.x += self.accel
                data.angular.z = 0.0
            else:
                data.linear.x += self.accel
                error = self.servo_target - ls.left_side
                data.angular.z = error * self.servo_kp * math.pi / 180.0
                deltaError = error - previousError
                deltaTime = nowTime - previousTime
                rospy.loginfo("delta time %.6f",deltaTime)
                if deltaTime > 0.0000001:
                    data.angular.z += deltaError / deltaTime * self.servo_kd * math.pi / 180.0
            
            previousTime = nowTime
            if data.linear.x <= self.min_speed:
                data.linear.x = self.min_speed
            elif data.linear.x >= self.max_speed:
                data.linear.x = self.max_speed

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    WallAround().run()
