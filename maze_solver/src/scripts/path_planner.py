#!/usr/bin/env python
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
import obstacle_detection as obsdet
import matplotlib.pyplot as plt
import numpy as np
import math
import random
from shapely.geometry import Point, Polygon, LineString
import time
from maze_solver.msg import Coords

import rospy
#obstacle_list=[[(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],[(3, 1), (3, 6), (4, 6), (4, 1)],[(7, 3), (7, 8), (9, 8), (9, 3)]]
#Bounds = 10

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None

class RRT:
    def __init__(self,start,goal):
        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])
        self.bias = 8
        self.dist_max = 0.2

    """def within(self,obstacle_list,rnd_node):

        inObstacle = False
        point = Point(rnd_node.x,rnd_node.y)
        if(point.within(Polygon(obstacle_list[0]))==True or point.within(Polygon(obstacle_list[1]))==True or point.within(Polygon(obstacle_list[2]))==True):
            inObstacle = True
        
        return inObstacle"""

    """def cross(self,obstacle_list,rnd_node,x,y):

        isCrosses = False
        point = Point(rnd_node.x,rnd_node.y)
        node = Point(x,y)
        coord=([rnd_node.x,rnd_node.y],node)
        line=LineString(coord)
        if(line.crosses(Polygon(obstacle_list[0]))==True or line.crosses(Polygon(obstacle_list[1]))==True or line.crosses(Polygon(obstacle_list[2]))==True or point.touches(Polygon(obstacle_list[0]))==True or point.touches(Polygon(obstacle_list[1])) ==True or point.touches(Polygon(obstacle_list[2]))==True):
            isCrosses = True

        return isCrosses"""

    def euclidian_dist(self,node1:Node,node2:Node):
        X = node1.x - node2.x
        Y = node1.y - node2.y

        return math.sqrt(X**2 + Y**2)

    def search(self):
        self.node_list = []
        self.node_list.append(self.start)
        goal_found = False
        counter = 0
        
        while not goal_found:
            counter += 1
            if counter%self.bias == 0:
                x = self.goal.x
                y = self.goal.y
            else:
                x = random.uniform(-6,6)
                y = random.uniform(-3,3)
            #if x < Bounds[0] and y < Bounds[1]:
            rnd_node = Node(x,y)
            dist = self.euclidian_dist(rnd_node,self.start)
            #print(dist)
            rnd_node.parent = self.start
            min_index = 0
            for i in range(len(self.node_list)):
                current_dist = self.euclidian_dist(rnd_node,self.node_list[i])
                if current_dist < dist:
                    dist = current_dist
                    rnd_node.parent = self.node_list[i]
                    min_index = i
                #dist.append(self.euclidian_dist(rnd_node,i))
            #min_dist = min(dist)
            #rnd_node.parent = self.node_list[dist.index(min_dist)]
            if current_dist > self.dist_max:
                rnd_node.x = self.node_list[min_index].x + (self.dist_max*(rnd_node.x - self.node_list[min_index].x)/dist)
                rnd_node.y = self.node_list[min_index].y + (self.dist_max*(rnd_node.y - self.node_list[min_index].y)/dist)
            #if self.within(obstacle_list,rnd_node) and self.cross(obstacle_list,rnd_node,self.node_list[min_index].x,self.node_list[min_index].y):
            if not obsdet.isValidPoint(rnd_node.parent.x,rnd_node.parent.y,rnd_node.x,rnd_node.y): 
                continue
                    #self.node_list.append(rnd_node)
                    #rnd_node.parent = self.node_list[dist.index(min_dist)]
            self.node_list.append(rnd_node)
                
            if self.euclidian_dist(rnd_node,self.goal) < self.dist_max:
                self.node_list.append(self.goal)
                self.goal.parent = rnd_node
                goal_found = True
            #print(goal_found)
        return self.node_list
    def backtrack(self):
        path = []
        
        current = self.goal
        while current.parent is not None:
            path.append(current)
            current = current.parent
        #for node in path:
         # plt.scatter(node.x,node.y,color='red')
        return path
    
    #def plot_path(self):
     #   poly1=Polygon(obstacle_list[0])
      #  poly2=Polygon(obstacle_list[1])
       # poly3=Polygon(obstacle_list[2])
       # x1,y1=poly1.exterior.xy
        #x2,y2=poly2.exterior.xy
        #x3,y3=poly3.exterior.xy
        #plt.plot(x1,y1)
        #plt.plot(x2,y2)
        #plt.plot(x3,y3)
        #for i in range(len(self.node_list)):

         # plt.plot(self.node_list[i].x,self.node_list[i].y,"r.-", markersize = 3, linewidth = 0.3)
        """current=len(self.node_list)-1
        plot_list=[]
        while(current!=0):
            plot_list.append(self.node_list[current])
            current=self.node_list[current].parent

        plot_list.append(Node(self.start[0],self.start[1],0,0,[],0))
        plot_list.reverse()
        i=0
        while i<len(plot_list)-1:
            plt.plot([plot_list[i].x,plot_list[i+1].x],[plot_list[i].y,plot_list[i+1].y],color='blue')
            i=i+1
        for node in plot_list:
            plt.scatter(node.x,node.y,color='blue',s=30)"""


if __name__ == '__main__':
    time.sleep(5)
    print('start planning')
    rospy.init_node('planner')
    coords = Coords()
    pub = rospy.Publisher('/path',Coords,queue_size=1)
    rate = rospy.Rate(1)
    rrt = RRT([-5.2,-2.6],[3.2,0.27])
    rrt.search()
    path = rrt.backtrack()
    path_x = []
    path_y = []
    i = len(path)-1
    while i>=0:
        path_x.append(path[i].x)
        path_y.append(path[i].y)
        i -= 1
    #time.sleep(5)
    #i=0
    while not rospy.is_shutdown():
        coords.x_coords = path_x
        coords.y_coords = path_y
        pub.publish(coords)
        rate.sleep()
           







