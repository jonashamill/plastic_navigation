#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
import random

def init_ros():

    rospy.init_node("activity_calc")

    neo_threhold = rospy.get_param("/neo_threshold_init")
    use_plasticity = rospy.get_param("/use_plasticity")
    robot_ns = rospy.get_namespace()

    return neo_threhold, use_plasticity, robot_ns    



def marker_callback(msg):

    return



def choose_behaviour(neo_threhold, robot_ns):


    while not rospy.is_shutdown():
        
        ranDomNo = random.randrange(0,101)

        finish = round(rospy.get_time(0), 0)

   
        timeTaken = round(finish-start, 2)

        


        if neo_threhold < ranDomNo:

            neophilia = 0

            activity_output = 'Patrol - (Neophobic)'



        else:
            
            neophilia = 1

            activity_output = 'Explore - (Neophilic)'
        
        rospy.loginfo(f'Behavioural Tendancy: {activity_output}')


        time.sleep(1)
    



def main():

    neo_threhold, use_plasticity, robot_ns = init_ros()
    
    

    # if (use_plasticity):
        
    #     neo



if __name__ == '__main__':
    

    main()

    rospy.spin()