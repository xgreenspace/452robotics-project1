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

""" Math to get distance to and angle between two points """


# vector from source to dest
def trans(source, dest):
    return (dest[0] - source[0], dest[1] - source[1])

# angle of vector
def angle(vector):
    return math.atan2(vector[1], vector[0])


# normalizes coordinates to fit between (1,1) and (10,10)
def rescale(x, y):
    x_range = 30
    y_range = 25

    x_out = 1 + (x / x_range) * 9
    y_out = 1 + (y / y_range) * (25/3 - 1)

    return x_out, y_out

def read_points(open_file):
    points = []
    with open(open_file, 'r') as file:
        for line in file:
            if(line[0] == "("):
                x, y = line.strip()[1:-1].split(',')
                points.append((float(x), float(y)))
            else: # lets just go with amari
                points.append(line.strip())
    return points

class TAMUBot(Node):

    def __init__(self):
        super().__init__('tamuturtle')
        
        self.publisher_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscriber_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Pose()
        self.flag = False

        self.points = read_points("points.txt")
        self.steps = 0 # Current point
        
        self.goal_pose= Pose()
        goal = rescale(*self.points[0])
        
        self.goal_pose.x, self.goal_pose.y = goal # first point on letter
        self.goal_pose.theta = self.steering_angle(self.goal_pose) # angle to letter

        self.pen = self.create_client(SetPen, 'turtle1/set_pen')

        self.set_pen(False)

    def set_pen(self, state):
        msg = SetPen.Request()
        msg.r = 255
        msg.g = 255
        msg.b = 255
        msg.width = 1
        msg.off = not state
        self.pen.call_async(msg)
    

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

        distance_tolerance = 0.1
        angular_tolerance = 0.01

        goal_pose = self.goal_pose

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
             # put that in here
            self.steps += 1
            
            if self.steps >= len(self.points):
                self.steps = len(self.points) - 1
            
            goal = self.points[self.steps]
            
            if goal == "STOP DRAWING":
                #turn off pen
                self.set_pen(False)
                self.steps += 1
                goal = self.points[self.steps]
                
            if goal == "START DRAWING": # this string is 13 characters long
                #turn on pen
                self.set_pen(True)
                self.steps += 1
                goal = self.points[self.steps]
            
            goal = rescale(*goal)  # a string is being passed into it


            self.goal_pose= Pose()
            self.goal_pose.x, self.goal_pose.y = goal # next point
            self.goal_pose.theta = self.steering_angle(self.goal_pose)  # should be angle from current point to next point
            self.flag = False
           
            
        
        # if we finished drawing, call quit()

    

        self.publisher_cmdvel.publish(vel_msg)
        



def main(args=None):
    rclpy.init(args=args)
    tamubot = TAMUBot()
    rclpy.spin(tamubot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()