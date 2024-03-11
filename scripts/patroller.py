#!/usr/bin/env python3

import rospy
import math
import pandas as pd
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32, Bool
import threading
import random
import os
from datetime import datetime
import csv
import socket

plasticMSG = 0
tagMSG = False
pauseMSG = False
timeList = []
beHaveList =[]



def plasticCallback(msg):

    global plasticMSG

    plasticMSG = msg.data

def tagCallback(msg):

    global tagMSG
    
    tagMSG = msg.data

def pauseCallback(msg):

    global pauseMSG

    pauseMSG = msg.data



class Patroller():

    def __init__(self):

        global robot_ns

        rospy.init_node('patroller')  # initialize node

        # preprocessing --------------------------------------------------

        robot_ns = rospy.get_namespace()

        # gets csv file path
        rp = rospkg.RosPack()
        package_path = rp.get_path('multi_sim')
        route = rospy.get_param(robot_ns + 'route')
        CSV_path = (package_path + "/waypoints/" + route)

        # converts waypoints text file into a list of points to follow
        df = pd.read_csv(CSV_path, sep=',', header=None)
        self.theta = list(df.loc[:, 3].values)
        wayp = df.loc[:, 0:2]
        self.waypoints = []
        wayp = wayp.values.tolist()
        for sublist in wayp:
            for item in sublist:
                self.waypoints.append(item)

        points_seq = self.waypoints  # heading angle for each waypoint
        yaweulerangles_seq = self.theta  # coordinates for each waypoint

        # Convert waypoint & heading values into a list of robot poses (quaternions?) -----------
        quat_seq = list()
        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = rospy.get_param(robot_ns + 'start_node')
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1

        # Create action client -------------------------------------------
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(robot_ns + "Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr(robot_ns + "Action server not available!")
            rospy.signal_shutdown("robot_ns + Action server not available!")
            return
        rospy.loginfo(robot_ns + "Connected to move base server")
        rospy.loginfo(robot_ns + "Starting goals achievements ...")

        # Initiate status subscriber
        self.status_subscriber = rospy.Subscriber(
            robot_ns + "move_base/status", GoalStatusArray, self.status_cb
        )

        # Create a Twist publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(robot_ns + 'controllers/diff_drive/cmd_vel', Twist, queue_size=10)


        self.patrol_count = 0
        self.tick = 0
        self.movebase_client()

    def movebase_client(self):

        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo(robot_ns + "Sending goal pose " +
                      str(self.goal_cnt+1)+" to Action Server")
        #rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal)
        rospy.loginfo("==========*" + robot_ns + "GOAL SENT *==========")
        
        

    def status_cb(self, msg):

        status = self.client.get_state()
        #rospy.loginfo(status)

        robot_ns = rospy.get_namespace()

        if status == 3:
            if self.tick == 1: # tick is one, returned home, done.
                rospy.loginfo(robot_ns + "FIN")
                rospy.signal_shutdown(robot_ns + "FIN")
                exit()
            self.goal_cnt +=1
            rospy.loginfo(robot_ns + "Goal pose "+str(self.goal_cnt)+" reached")
            if rospy.get_param("/speed") == "slow":
                    rospy.loginfo(robot_ns + "Spinning...")
                    self.spin_robot()
            if self.goal_cnt != rospy.get_param(robot_ns + "start_node"):#
                if self.goal_cnt == len(self.pose_seq):
                    self.goal_cnt = 0
                rospy.loginfo(robot_ns + "Moving onto next goal...")
                self.movebase_client()
            else:
                rospy.loginfo(robot_ns + "Final goal pose reached!")
                self.patrol_count += 1
                print(self.patrol_count)
                if self.patrol_count == rospy.get_param(robot_ns + "patrols"): # if done all the patrols return home, set tick to 1
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = self.pose_seq[rospy.get_param(robot_ns + "start_node")]
                    # rospy.loginfo("Returning to first waypoint")
                    # rospy.loginfo(str(self.pose_seq[rospy.get_param("~start_node")]))
                    self.client.send_goal(goal)
                    self.tick = 1
                    rospy.loginfo("==========*" + robot_ns + "Returning Home *==========")
                else:
                    rospy.loginfo(robot_ns + "Repeating patrol ...")
                    self.goal_cnt = rospy.get_param(robot_ns + "start_node")
                    self.movebase_client()

    def check_and_pause(self):

        global plasticMSG
        global tagMSG
        global pauseMSG
        global timeList
        global beHaveList


        # use_plasticity = rospy.get_param("/use_plasticity")

        # if use_plasticity:

        while not rospy.is_shutdown():  # Continuously check while the node is active

            rospy.Subscriber(robot_ns + 'plasticTopic', Bool, plasticCallback)

            rospy.Subscriber(robot_ns + 'tag_topic', Bool, tagCallback)

            if plasticMSG == 1 and tagMSG == True:
                
                rospy.loginfo(robot_ns + 'Plastic set to: %s', str(plasticMSG))

                rospy.loginfo(robot_ns + "Behaving")

                ranDomNo = random.randrange(0,2)

                t_end = rospy.Time.now() + rospy.Duration(5)  # Wait for 10 seconds
                while rospy.Time.now() < t_end:
                    vel_msg = Twist()

                    

                    if ranDomNo == 1:
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = 1.0

                        beHave = 'Spinning'

                    else:
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = 0.0

                        beHave = 'Pausing'
                    
                    self.vel_pub.publish(vel_msg)

                tagMSG = False

                
                # Stop the robot
                vel_msg = Twist()
                self.vel_pub.publish(vel_msg)

                rospy.loginfo(str(beHave) + robot_ns + " Behaviour Done")

                # Publish 'tag' as a ROS topic
                tagPub = rospy.Publisher(robot_ns + 'tagTopic', Int32, queue_size=10)
                tagPub.publish(False)
        

    def run(self):
        # Start the check_and_pause method in a separate thread
        check_and_pause_thread = threading.Thread(target=self.check_and_pause)
        check_and_pause_thread.daemon = True  # Daemonize the thread so it stops when the main program exits
        check_and_pause_thread.start()

        # Continue with the rest of your logic (e.g., moving between waypoints)
        self.patrol_count = 0
        self.tick = 0
        self.movebase_client()



if __name__ == '__main__':
    try:

        patroller = Patroller()
        patroller.run()
        rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo(robot_ns + "Navigation finished.")