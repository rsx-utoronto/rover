#!/usr/bin/python3
import roslib
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import tf2_geometry_msgs

def odom_handler(msg, topic):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    tf_found = False
    while not tf_found:
        try:
            t_msg = tfBuffer.lookup_transform("body", "imu", rospy.Time())
            tf_found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    msg_t = tf2_geometry_msgs.do_transform_pose(msg.pose, t_msg)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "body"
    t.transform.translation.x = msg_t.pose.position.x
    t.transform.translation.y = msg_t.pose.position.y
    t.transform.translation.z = msg_t.pose.position.z
    t.transform.rotation = msg_t.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_broadcaster')
    topic = rospy.get_param('~topic')
    rospy.Subscriber(topic,
                     Odometry,
                     odom_handler,
                     topic)
    rospy.spin()