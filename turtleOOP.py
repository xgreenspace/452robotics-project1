# ros2 service type /turtle1/set_pen -> turtlesim/srv/SetPen
# ros2 interface show turtlesim/srv/SetPen

import math
import rclpy.node
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from turtlesim.msg import Color
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

import os
import sys


# class TurtleCircle(Node):

# 	def __init__(self):
# 		super().__init__('turtle_node')
# 		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
# 		self.my_subscriber = self.create_subscription(
# 			Color, 
# 			"/turtle1/color_sensor", 
# 			self.color_callback_sub, 
# 			10
# 		)
# 		self.my_subscriber
# 		timer_period = 0.5 # seconds
# 		self.timer = self.create_timer(timer_period, self.circular_callback_pub)
# 		self.i = 0
		
# 	def circular_callback_pub(self):
# 		"""
# 		Method called by node timer very timer_period.
# 		Creates turtle's circular motion.
# 		"""
# 		vel = Twist()
# 		line_vel = 3.0
# 		ang_vel = 2.5

# 		# Defines angular and linear velocity
# 		vel.linear.x, vel.linear.y, vel.linear.z = line_vel, 0.0, 0.0
# 		vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, ang_vel
	
# 		self.publisher_.publish(vel)
# 		self.get_logger().info(f"Publishing velocity: \n\t linear.x: {vel.linear.x}; \n\t linear.z: {vel.angular.z}")
			
# 	def color_callback_sub(self, msg):
# 		"""
# 		Method logs the color returning from the topic.
# 		"""
# 		self.get_logger().info(f'Turtle sees Color c: {msg}')
# 		sleep(1)

		
# def main(args=None):
# 	rclpy.init(args=args)
# 	turtle1 = TurtleCircle()
# 	rclpy.spin(turtle1)
# 	turtle1.destroy_node()
# 	rclpy.shutdown()
	
# if __name__ == '__main__':
# 	main()

""" Math to get distance to and angle between two points """


# vector from source to dest
def trans(source, dest):
    return (dest[0] - source[0], dest[1] - source[1])

# angle of vector
def angle(vector):
    return math.atan2(vector[1], vector[0])




class TAMUBot(Node):

    def __init__(self):
        super().__init__('tamuturtle')
        
        self.publisher_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscriber_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Pose()
        self.flag = False

    def callback(self, data):
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta = data.theta
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
        self.get_logger().info(msg)

        
    # returns distance between points, and angle to the horizontal of the source
    def euclidean(self, goal_pose):
        return math.hypot(*trans((self.pose.x, self.pose.y), (goal_pose.x, goal_pose.y)))
    

    def steering_angle(self, goal_pose):
        vector = trans((self.pose.x, self.pose.y), (goal_pose.x, goal_pose.y))
        return angle(vector)

    def linear_vel(self, goal_pose):
        return 2 * self.euclidean(goal_pose)

    def angular_vel(self, goal_pose):
        return 2 * (self.steering_angle(goal_pose)-self.pose.theta)


    def move2goal(self):
        goal_pose= Pose()
        goal_pose.x = float(sys.argv[1])
        goal_pose.y = float(sys.argv[2])
        goal_pose.theta = float(sys.argv[3])

        distance_tolerance = 0.1
        angular_tolerance = 0.01

        vel_msg = Twist()

        if abs(self.steering_angle(goal_pose)-self.pose.theta) > angular_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)
        else:
            vel_msg.angular.z = 0.0
            if self.euclidean(goal_pose)>=distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
            else:
                vel_msg.linear.x = 0.0
                self.flag = True
        
        if self.flag:
            vel_msg.angular.z = goal_pose.theta - self.pose.theta
            if abs(goal_pose.theta - self.pose.theta) <= angular_tolerance:
                quit()

        self.publisher_cmdvel.publish(vel_msg)



def main(args=None):
    rclpy.init(args=args)
    tamubot = TAMUBot()
    rclpy.spin(tamubot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()