#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String, Bool
import random
from rostopic import get_topic_list

current_marker = 999
marker = False
marker_qty = 0


def init_ros():

    rospy.init_node("activity_calc")
    

    rospy.loginfo("Starting Sim _")


    neo_threshold = int(rospy.get_param("/neo_threshold_init"))
    use_plasticity = rospy.get_param("/use_plasticity")
    robot_ns = rospy.get_namespace()
    
    rospy.loginfo(f"Neo Threshold: {neo_threshold}, Plasticity: {use_plasticity}, Robot_NS: {robot_ns}")

    
    rospy.Subscriber(robot_ns + "tag_topic", Bool, tag_callback)

    rospy.Subscriber(robot_ns + "tag_qty_topic", Int32, tag_qty_callback)


    
    return neo_threshold, use_plasticity, robot_ns


def tag_callback(msg):
    global marker

    marker = msg.data

def tag_qty_callback(msg):
    global marker_qty

    marker_qty = msg.data

def choose_behaviour(neo_threshold):
        
    rand_no = random.randrange(0,101)

    if neo_threshold < rand_no:

        neophilia = 0

        activity_output = 'Patrol - (Neophobic)'

    else:
        
        neophilia = 1

        activity_output = 'Explore - (Neophilic)'
    
    rospy.loginfo(f'Behavioural Tendancy: {activity_output}')

    return neophilia



def main():

    global marker

    rospy.loginfo("Starting Sim")

    neo_threshold, use_plasticity, robot_ns = init_ros()

    
    # sets ros to run at a rate of 1Hz
    rate = rospy.Rate(1)


    start = int(rospy.get_time())


    while not rospy.is_shutdown():

        if use_plasticity:

            if marker:

                neo_threshold += 10

            else:
            
                neo_threshold -= 1

            neo_threshold = max(0, min(100, neo_threshold))
            
    

        neophilia = choose_behaviour(neo_threshold)

        finish = int(rospy.get_time())

        elapsed = (finish - start)

        rospy.loginfo(f"Elapsed time: {elapsed}")

        rospy.loginfo(f"Neo Threshold: {neo_threshold}, Neophilia: {neophilia}, Robot_NS: {robot_ns}")


        metricsOutput = metricsOutput = "{},{},{},{},{}".format(elapsed, neo_threshold, neophilia, marker, marker_qty)
        metricsMessage = String()
        metricsMessage.data = metricsOutput

        metricsPub = rospy.Publisher(robot_ns + 'metrics_topic', String, queue_size=10)
        metricsPub.publish(metricsMessage)   
        
        marker = False

        plastic_pub = rospy.Publisher(robot_ns + 'plasticTopic', Bool, queue_size=10)
        plastic_pub.publish(neophilia)

        rate.sleep()




if __name__ == '__main__':
    
    main()
