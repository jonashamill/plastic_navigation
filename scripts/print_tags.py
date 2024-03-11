#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from ar_track_alvar_msgs.msg import AlvarMarkers


# This script scans for AR tags and publishes True 
# Whenever one is spotted



def rosInit():
     
    rospy.init_node("ar_logger")

    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback_ar_pose)
     

def callback_ar_pose(msg):    

    tag_pub = rospy.Publisher('tag_topic', Bool, queue_size=10) 
    # rate = rospy.Rate(1)
   

    for marker in msg.markers:

        if marker.id < 19:

            currentMarker = marker.id

            tag = True

            
            tag_pub.publish(tag)



if __name__ == "__main__":
        
        rosInit()

        rospy.spin()