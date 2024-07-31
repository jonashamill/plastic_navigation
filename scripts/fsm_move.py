#!/usr/bin/env python3


import rospy
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import tf
import rospkg 


def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line:  # Skip empty lines
                try:
                    x, y, z, yaw_deg = map(float, line.split(','))
                    yaw_rad = math.radians(yaw_deg)
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
                    waypoints.append([(x, y, z), quaternion])
                except ValueError:
                    rospy.logwarn(f"Skipping invalid line: {line}")
    return waypoints



# def read_waypoints(file_path):
#     waypoints = []
#     with open(file_path, 'r') as file:
#         for line in file:
#             x, y, z, yaw_deg = map(float, line.strip().split(','))
#             yaw_rad = math.radians(yaw_deg)
#             quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
#             waypoints.append([(x, y, z), quaternion])
#     return waypoints

def main():
    robot_ns = rospy.get_namespace()

    route = rospy.get_param(robot_ns + 'route')

    patrols = rospy.get_param(robot_ns + 'patrols')

    rp = rospkg.RosPack()
    package_path = rp.get_path('multi_sim')
    CSV_path = (package_path + "/waypoints/" + route)

    waypoints = read_waypoints(CSV_path)

    for i in range (patrols):
        rospy.loginfo(robot_ns + " Patrol number: " + str(i))

        patrol = StateMachine(['succeeded', 'aborted', 'preempted'])
        with patrol:
            for i,w in enumerate(waypoints):

                
                goal_pose = MoveBaseGoal()
                goal_pose.target_pose.header.frame_id = 'map'

                

                goal_pose.target_pose.pose.position.x = w[0][0]
                goal_pose.target_pose.pose.position.y = w[0][1]
                goal_pose.target_pose.pose.position.z = w[0][2]

                goal_pose.target_pose.pose.orientation.x = w[1][0]
                goal_pose.target_pose.pose.orientation.y = w[1][1]
                goal_pose.target_pose.pose.orientation.z = w[1][2]
                goal_pose.target_pose.pose.orientation.w = w[1][3]

                StateMachine.add(w[0], 
                    SimpleActionState('move_base',MoveBaseAction, goal=goal_pose), 
                    transitions={'succeeded':waypoints[(i+1) % len(waypoints)][0]})
                
                
        rospy.loginfo(robot_ns + " Headed to waypoint")

    
        outcome = patrol.execute()
        rospy.loginfo("State machine transitioning, robot_ns: {}, waypoint: {}. {} moving to waypoint {}".format(robot_ns, i, outcome, (i+1) % len(waypoints)))




if __name__ == '__main__':
    
    try:
        rospy.init_node('patroller')

        main()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Sim Finished")