#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.843515, Y + 0.6581).
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from maze_solver.msg import Coords
from geometry_msgs.msg import Twist,Point
import time
import tf

"""class Robot:
    def __init__(self,x_coord,y_coord,theta,x_vel,y_vel,ang_theta) -> None:
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.theta = theta
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.ang_theta = ang_theta"""

x_coord = 0.0
y_coord = 0.0
theta = 0.0
#goal = Point()
Loco = Twist()
del_time = 5
goal_list = []

def getOdom(msg):
    global x_coord
    global y_coord
    global theta

    x_coord = msg.pose.pose.position.x
    y_coord = msg.pose.pose.position.x
    theta_q = msg.pose.pose.orientation
    (roll,pitch,theta) = tf.transformations.euler_from_quaternion([theta_q.x,theta_q.y,theta_q.z,theta_q.w])
    
def euclidian_dist(start:list,goal:list):
    X = start[0] - goal[0]
    Y = start[1] - goal[1]
    return math.sqrt(X**2 + Y**2)

def angle_adjustments(goal:list):
    angular_adjust = False
    kp_thta = 0.1
    #ki_thta = 0.0001
    kd_thta = 0.01
    thta_err_pre = math.atan2(goal[1]-y_coord,goal[0]-x_coord)
    #thta_err_sum = 0
    while abs(theta - thta_err_pre) > 0.01:
        Loco.linear.x = 0
        thta_err = math.atan2(goal[1]-y_coord,goal[0]-x_coord)
        #thta_err_sum += thta_err*del_time
        dthta_errdt = (thta_err - thta_err_pre)/del_time
        #Loco.angular.z = kp_thta*thta_err + ki_thta*thta_err_sum + kd_thta*dthta_errdt
        Loco.angular.z = kp_thta*thta_err + kd_thta*dthta_errdt
        thta_err_pre = thta_err
        pub.publish(Loco)
    angular_adjust = True
    Loco.angular.z = 0.00
    pub.publish(Loco)
    return angular_adjust

def linear_adjustment(goal:list):
    kp_lin = 1
    #ki_lin = 0.00001
    kd_lin = 1
    lin_err_pre = euclidian_dist([x_coord,y_coord],goal)
    #lin_err_sum = 0
    linear_adjust = False
    while euclidian_dist([x_coord,y_coord],goal) > 0.1:
        lin_err = euclidian_dist([x_coord,y_coord],goal)
        #lin_err_sum += lin_err*del_time
        dlin_errdt = (lin_err - lin_err_pre)/del_time
        Loco.linear.x = min(kp_lin*lin_err + kd_lin*dlin_errdt,0.15)
        lin_err_pre = lin_err
        pub.publish(Loco)
    linear_adjust = True
    #Loco.linear.x = 0.00
    pub.publish(Twist)
    print("current co-ordinates are:",(x_coord,y_coord,theta))
    print("goal reached")
    return linear_adjust
def coord_callback(msg):
    #global goal
    global goal_list
    for i in range(len(msg.x_coords)):
        goal_list.append([msg.x_coords[i]+1.843515,msg.y_coords[i]+0.6581])
    #return goal_list


rospy.init_node('not_actual_controller')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
sub1 = rospy.Subscriber('/odom',Odometry,getOdom)
sub2 = rospy.Subscriber('/path',Coords,coord_callback)
rate = rospy.Rate(10)
#print("current co-ordinates are:",(x_coord,y_coord,theta))

while not rospy.is_shutdown():
    for i in range(len(goal_list)):
        goal = goal_list[i]
        angle_adjustments(goal)
        linear_adjustment(goal)
    #pub.publish(Twist)
    #print("current co-ordinates are:",(x_coord,y_coord,theta))
        rate.sleep()




    
