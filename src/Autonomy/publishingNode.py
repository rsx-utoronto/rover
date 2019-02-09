#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def talker(distance, angle):
    pub = rospy.Publisher('ballInfo', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        distance = str(dis) % rospy.get_time()
        angle = str(angle) % rospy.get_time()
        rospy.loginfo(distance)
        pub.publish(distance)
        rospy.loginfo(angle)
        pub.publih(angle)
        rate.sleep()
      
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass