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

""" Math to get distance to and angle between two points """

# vector from source to dest
def trans(source, dest):
    return (dest[0] - source[0], dest[1] - source[1])

# angle of vector
def angle(vector):
    return math.atan2(vector[1], vector[0])


# returns distance between points, and angle to the horizontal of the source
def pcoord(source, dest):
    vector = trans(source, dest)
    a = angle(vector)
    return math.hypot(*vector), a

# returns a Twist msg to turn from curr to next 
def turn(curr, next):
    turn_dist = (next - curr)

    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = turn_dist #should run for 1 sec
    publisher.publish(msg)

# returns a Twist msg to drive from curr to next. Assumes the angles align
def drive(dist):

    msg = Twist()
    msg.linear.x = dist #should run for 1 sec
    msg.angular.z = 0.0
    publisher.publish(msg)

# turtle drives from p1 to p2
def GoTo(p1, p2):
    dist, angle = pcoord(curr, next)
    turn(curr, angle)
    drive(dist)



def main(args=None):
    rclpy.init(args=args)

    #change background color to maroon
    node = rclpy.node.Node('atm')
    #node = ATMBot()
    #node.setParam(0, 0, 0)   # set red, green, blue


    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 100)
    # def timer_callback():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 2*math.radians(180)
    publisher.publish(msg)
    # timer = node.create_timer(0.5, timer_callback)
    

    # def color_sensor_callback(msg):
    #     print(msg)
    # subscription = node.create_subscription(Color, '/turtle1/color_sensor', color_sensor_callback, 10)


    # Clear all drawings
    # ros2 param set /turtlesim background_r 150
    # ros2 service call /clear std_srvs/srv/Empty

    # set background color to marooon
    # os.system("ros2 param set /turtlesim background_r 80")
    # os.system("ros2 param set /turtlesim background_g 0")
    # os.system("ros2 param set /turtlesim background_b 0")
    
    
    # turn pen on and off set_pen(r, g, b, width, off)
    pen = node.create_client(SetPen, 'turtle1/set_pen')


    def set_pen_toggle(toggle):
        msg = SetPen.Request()
        msg.r = 255
        msg.g = 255
        msg.b = 255
        msg.width = 1
        msg.off = not toggle
        pen.call_async(msg)


    set_pen_toggle(True)
    #set_pen_toggle(False)



# ros2 interface show turtlesim/msg/Pose
# float32 x
# float32 y
# float32 theta

# float32 linear_velocity
# float32 angular_velocity


# ros2 interface show turtlesim/srv/Spawn
# float32 x
# float32 y
# float32 theta
# string name # Optional.  A unique name will be created and returned if this is empty
# ---
# string name


    
    

    rclpy.spin(node)



if __name__ == '__main__':
    main()


# """
# - turn on and off pen
# - how cmd_vel works
# - how to make multipe turtles (maybe?)
# - how to go from point A to point B
# - how to orient the robot accurately
# ""





