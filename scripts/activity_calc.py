#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
import random





def init_ros():

    rospy.init_node("activity_calc")

    rospy.loginfo("Starting Sim _")



    neo_threshold = int(rospy.get_param("/neo_threshold_init"))
    use_plasticity = rospy.get_param("/use_plasticity")
    robot_ns = rospy.get_namespace()


    rospy.loginfo(f"Neo Threshold: {neo_threshold}, Plasticity: {use_plasticity}, Robot_NS: {robot_ns}")

    return neo_threshold, use_plasticity, robot_ns



# def marker_callback(msg):

#     for marker in msg.markers:
#         global idList
#         global currentMarker

#     return



def choose_behaviour(neo_threhold):


    while not rospy.is_shutdown():
        
        ranDomNo = random.randrange(0,101)

        if neo_threhold < ranDomNo:

            neophilia = 0

            activity_output = 'Patrol - (Neophobic)'



        else:
            
            neophilia = 1

            activity_output = 'Explore - (Neophilic)'
        
        rospy.loginfo(f'Behavioural Tendancy: {activity_output}')


   



def main():

    rospy.loginfo("Starting Sim")

    print("Starting simpy")

    neo_threshold, use_plasticity, robot_ns = init_ros()

    
    
    rate = rospy.Rate(1)
    marker = 1

    start = int(rospy.get_time())


    while not rospy.is_shutdown():

        if use_plasticity:

            if marker:

                neo_threshold += 1

            else:
            
                neo_threshold -= 1
    


        finish = int(rospy.get_time())

        elapsed = (finish - start)

        rospy.loginfo(f"Elapsed time: {elapsed}")

        rand_no = random.randrange(0,101)

        if neo_threshold < rand_no:

            neophilia = 0

            activity_output = 'Patrol - (Neophobic)'



        else:
            
            neophilia = 1

            activity_output = 'Explore - (Neophilic)'
        
        rospy.loginfo(f'Behavioural Tendancy: {activity_output}')


        rospy.loginfo(f"Neo Threshold: {neo_threshold}, Neophilia: {neophilia}, Robot_NS: {robot_ns}")

        

        rate.sleep()




if __name__ == '__main__':
    
    main()
    rospy.spin()