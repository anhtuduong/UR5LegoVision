

import rospy as ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Point
import roslaunch
import os
import rosnode

class RosPub():
    def __init__(self):



        #set MARKER  publisher
        self.marker_pub = ros.Publisher('/vis' , MarkerArray, queue_size=1)                                
        self.markerArray = MarkerArray()
        self.markerArray.markers = []     
        self.id = 0                             
                                


    def publishVisual(self):

        #publish also the markers if any
        if len(self.markerArray.markers)>0:                                                                                                                            
            self.marker_pub .publish(self.markerArray)  
        #reset the marker array making it ready for another round
        self.markerArray.markers = []                             
        self.id = 0
                                
    def add_marker(self, pos):
       marker = Marker()
       marker.header.frame_id = "world"
       marker.type = marker.SPHERE
       marker.action = marker.ADD
       marker.scale.x = 0.1
       marker.scale.y = 0.1
       marker.scale.z = 0.1
       marker.color.a = 0.5
       marker.color.r = 1.0
       marker.color.g = 1.0
       marker.color.b = 0.0                        
                               
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = pos[0]
       marker.pose.position.y = pos[1] 
       marker.pose.position.z = pos[2] 

       marker.id = self.id       
       self.id += 1                                        
       self.markerArray.markers.append(marker)
                            
    def add_arrow(self, start, vector, color = "green"):

       marker = Marker()
       if (color == "green"):                    
           marker.color.r = 0.0
           marker.color.g = 1.0
           marker.color.b = 0.0                    
       if (color == "blue"):                    
           marker.color.r = 0.0
           marker.color.g = 0.0
           marker.color.b = 1.0                                                

       marker.header.frame_id = "world"
       marker.type = marker.ARROW
       marker.action = marker.ADD  
       marker.points.append(Point(start[0], start[1], start[2]))    
       marker.points.append(Point(start[0] + vector[0], start[1] + vector[1], start[2] + vector[2]))                                                                                      
       marker.scale.x = 0.05
       marker.scale.y = 0.05
       marker.scale.z = 0.05
       marker.color.a = 1.0

       marker.id = self.id
       self.id += 1     
       self.markerArray.markers.append(marker)


    def isShuttingDown(self):
       return ros.is_shutdown()
                            

