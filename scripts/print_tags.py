#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32
from ar_track_alvar_msgs.msg import AlvarMarkers


# This script scans for AR tags and publishes True 
# Whenever one is spotted

idListBuffer = []
idQTY = 0


def rosInit():
     
    rospy.init_node("ar_logger")
    

    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback_ar_pose)


def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True




def callback_ar_pose(msg):    

    global idListBuffer
    global idQTY
    robot_ns = rospy.get_namespace()


    tag_pub = rospy.Publisher(robot_ns + 'tag_topic', Bool, queue_size=10) 
    tag_qty_pub = rospy.Publisher(robot_ns + 'tag_qty_topic', Int32, queue_size=10)
    # rate = rospy.Rate(1)
   

    for marker in msg.markers:

        if marker.id < 19:

            currentMarker = marker.id

            tag_pub.publish(True)

            if checkDuplicate(idListBuffer, currentMarker):
                continue
            else:
                idListBuffer.append(currentMarker)

                idQTY += 1          
                

    if len(idListBuffer) > 1: #this will be replaced with time
        idListBuffer =[]
                 
    tag_qty_pub.publish(idQTY)


if __name__ == "__main__":
        
        rosInit()

        rospy.spin()