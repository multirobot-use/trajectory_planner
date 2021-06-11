#!/usr/bin/env python
import sys
import rospy
import rospkg
import time
import math
import numpy
import os
import yaml
import threading
import numpy as np
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import GoToWaypoint
from uav_abstraction_layer.srv import GoToWaypointRequest
from uav_abstraction_layer.msg import State
from mission_planner.srv import WaypointSrvRequest
from mission_planner.srv import WaypointSrv
import signal
import sys
from collections import namedtuple


class Drone:
    state = 0
    def __init__(self, drone_ns):
                
        # subscribe topics
        rospy.Subscriber(drone_ns+"/ual/state", State, self.callbackState)
        
        # # Publishers (only one drone topic needed for each one)

        # wait for services
        activate_planner_url = drone_ns + "/mission_planner_ros/activate_planner"
        add_waypoint_url     = drone_ns + "/mission_planner_ros/add_waypoint"
        clear_waypoints_url  = drone_ns + "/mission_planner_ros/clear_waypoints"
        take_off_url   = drone_ns + "/ual/take_off"

        rospy.wait_for_service(activate_planner_url)
        self.activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)

        rospy.wait_for_service(add_waypoint_url)
        self.add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)

        rospy.wait_for_service(clear_waypoints_url)
        self.clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)


        # TakeOff service --> make a for loop when necessary
        rospy.wait_for_service(take_off_url)
        self.take_off_service = rospy.ServiceProxy(take_off_url, TakeOff)

        # GoToWaypoint service
        go_to_waypoint_url      = drone_ns + "/ual/go_to_waypoint"
        rospy.wait_for_service(go_to_waypoint_url)
        self.go_to_waypoint_service  = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)

    def take_off(self, height):
        try:
            take_off            = TakeOffRequest()
            take_off.height     = height
            take_off.blocking   = True
            
            # Leader drone service
            self.take_off_service(take_off)
            print "Taking off the drone"
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def go_to_waypoint(self, waypoint):
        try:
            waypoint              = GoToWaypointRequest()
            waypoint.blocking     = False
            waypoint.waypoint.pose.position.x     = waypoint[0]
            waypoint.waypoint.pose.position.y     = waypoint[1]
            waypoint.waypoint.pose.position.z     = waypoint[2]
            
            self.go_to_waypoint_service(waypoint)
            print "LEADER: Going to initial waypoint"
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def start_mission(self):
        if self.state == State.FLYING_AUTO:
            try:
                req = SetBoolRequest()
                req.data = True
                self.activate_planner_service(req)
                print "Mission has started!"

            except rospy.ServiceException, e:
                print("Failed calling start_mission service")
        else:
            print("Start mission aborted, drone is not flying auto")

    def add_one_waypoint(self, waypoint):
        
        add_waypoint_req = WaypointSrvRequest()
        add_waypoint_req.waypoint.pose.pose.position.x      = waypoint[0]
        add_waypoint_req.waypoint.pose.pose.position.y      = waypoint[1]
        add_waypoint_req.waypoint.pose.pose.position.z      = waypoint[2]
        print "Waypoint sent"
        print(waypoint)
        try:
            self.add_waypoint_service(add_waypoint_req)
            print "Waypoint added!"
            
        except:
            print("Failed calling add_waypoint service")

    def callbackState(self, data):
        self.state = data.state

    def stop_mission(self):
        try:
            req = SetBoolRequest()
            req.data = False
            self.activate_planner_service(req)
            print "Mission has been stopped!"
        
        except rospy.ServiceException, e:
            print("Failed calling stop_mission service")

    def clear_all_waypoints(self):
        try:
            self.clear_waypoints_service()
            print "Waypoints cleared!"
        except:
            print("Failed calling clear_waypoints service")

   
# Menu function
def show_menu(drone):

    # Menu
    print "\n\nWelcome to the main menu. Put the number of the desired option:\n"
    print "\t1. Take off and send the drones to their initial points"
    print "\t2. Start the mission"
    print "\t3. Stop the mission"
    print "\t4. Add waypoint"
    print "\t5. Clear all the waypoints"
    
    option = ord(raw_input (">> "))
    while (option < (48+1) or option > (48+9)): # ASCII for make sure there is no error of inputs. Zero --> 48
        option = ord(raw_input("Please, choose a valid option: "))

    option = option - 48

    if option == 1: #prepare drones
        height = raw_input("Please, introduce a height to take off: ")
        #taking of drones
        drone.take_off(float(height))

    elif option == 2: #start mission
        drone.start_mission()
            
    elif option == 3:
        drone.stop_mission()
            
    elif option == 4: # add waypoint
        px = float(raw_input("X pose (meters): "))
        py = float(raw_input("Y pose (meters): "))
        pz = float(raw_input("Z pose (meters): "))
        wp = [px, py, pz]
        drone.add_one_waypoint(wp)
        
    elif option == 5:
        drone.clear_all_waypoints()
        
    else:
        print ("Option n " + str(option) + " does not exist!")

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)    
        

# Main function
if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    # Create the node
    rospy.init_node("operator", anonymous=True)

    drone = Drone("drone_1")
    #menu mode
    while (not rospy.is_shutdown()):
        show_menu(drone)            
        time.sleep(1)