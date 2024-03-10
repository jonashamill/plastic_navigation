#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
import random

rospy.init_node("activity_calc")

rospy.loginfo("workk")


def init_ros():

    rospy.init_node("activity_calc")


    neo_threhold = rospy.get_param("/neo_threshold_init")
    use_plasticity = rospy.get_param("/use_plasticity")
    robot_ns = rospy.get_namespace()

    rospy.loginfo(f"Init: Neo Threshold: {neo_threhold}, Plasticity: {use_plasticity}, Robot_NS: {robot_ns}")

    return neo_threhold, use_plasticity, robot_ns



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

    # neo_threhold, use_plasticity, robot_ns = init_ros()

    rospy.init_node("activity_calc")


    neo_threhold = rospy.get_param("/neo_threshold_init")
    use_plasticity = rospy.get_param("/use_plasticity")
    robot_ns = rospy.get_namespace()

    rospy.loginfo(f"Init: Neo Threshold: {neo_threhold}, Plasticity: {use_plasticity}, Robot_NS: {robot_ns}")

    rospy.loginfo(f"Neo Threshold: {neo_threhold}, Plasticity: {use_plasticity}, Robot_NS: {robot_ns}")
    
    rate = rospy.Rate(1)
    marker = True

    # start = int(rospy.get_time())


    

    if (use_plasticity):

        print("plastic")
        
        # choose_behaviour(neo_threhold)

        

        # while not rospy.is_shutdown():

        #     # finish = int(rospy.get_time())

        #     # elapsed = (finish - start)

        #     # rospy.loginfo(f"Elapsed time: {elapsed}")

        #     if marker:

        #         neo_threhold += 1
        #     else:
        #         neo_threhold -= 1


        #     rand_no = random.randrange(0,101)

        #     if neo_threhold < rand_no:

        #         neophilia = 0

        #         activity_output = 'Patrol - (Neophobic)'



        #     else:
                
        #         neophilia = 1

        #         activity_output = 'Explore - (Neophilic)'
            
        #     rospy.loginfo(f'Behavioural Tendancy: {activity_output}')

        #     neo_threhold -= 1

    
        #     rospy.loginfo(f"Neo Threshold: {neo_threhold}, Neophilia: {neophilia}, Robot_NS: {robot_ns}")

            

        #     rate.sleep()

    else:

        print ("not plastic")



if __name__ == '__main__':
    print("if main")


    main()

    # rospy.spin()