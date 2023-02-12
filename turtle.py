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



# class ATMBot(Node):
#     def __init__(self):
#         super().__init__('cg_turtle')

#     def setParam(self, red, green, blue):
#         client = self.create_client(
#             SetParameters,
#             '/turtlesim/turtlesim/set_parameters'#.format_map(locals())
#         )

#         req_for_set = SetParameters.Request()

#         # setting for red
#         param = Parameter()                                 # prepare instance to access values
#         param.name = "background_r"                         # name
#         param.value.type = ParameterType.PARAMETER_INTEGER  # type
#         param.value.integer_value = red                     # value
#         req_for_set.parameters.append(param)                # add this to request_list to request

#         # setting for green
#         param = Parameter()                                 # prepare instance to access values
#         param.name = "background_g"                         # name
#         param.value.type = ParameterType.PARAMETER_INTEGER  # type
#         param.value.integer_value = green                   # value
#         req_for_set.parameters.append(param)                # add this to request_list to request

#         # setting for blue
#         param = Parameter()                                 # prepare instance to access values
#         param.name = "background_b"                         # name
#         param.value.type = ParameterType.PARAMETER_INTEGER  # type
#         param.value.integer_value = blue                    # value
#         req_for_set.parameters.append(param)                # add this to request_list to request

#         client.call_async(req_for_set)             # call service with request Asynchronously


# setdefup():
#     pass 


def main(args=None):
    rclpy.init(args=args)

    #change background color to maroon
    node = rclpy.node.Node('atm')
    #node = ATMBot()
    #node.setParam(0, 0, 0)   # set red, green, blue


    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    #def timer_callback():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = math.radians(180)
    publisher.publish(msg)
    #timer = node.create_timer(0.5, timer_callback)

    def color_sensor_callback(msg):
        print(msg)
    subscription = node.create_subscription(Color, '/turtle1/color_sensor', color_sensor_callback, 10)


    # Clear all drawings
    # ros2 param set /turtlesim background_r 150
    # ros2 service call /clear std_srvs/srv/Empty

    # set background color to marooon
    os.system("ros2 param set /turtlesim background_r 80")
    os.system("ros2 param set /turtlesim background_g 0")
    os.system("ros2 param set /turtlesim background_b 0")
    
    
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



