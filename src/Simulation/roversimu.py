#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates 
rospy.init_node('roversimu', anonymous=True)
def simu():
	publisher = rospy.Publisher('/rover/gazebo/set_link_state', LinkStates, queue_size=10)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		#linkstate=gazebo_msgs/LinkStates.msg(rover::right_bogie_front_wheel,'[0,0,0],[0,0,0]','[1,0,0],[0,0,0]')
		#linkstate = /gazebo/set_link_state '{model_state: { model_name: rover::right_bogie_front_wheel, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 1.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
		linkstate = LinkStates()
		linkstate.name = 'rover::right_bogie_front_wheel'   
		linkstate.twist = [1,0,0],[0,0,0]
		publisher.publish(linkstate)
		rate.sleep()
if __name__ == '__main__':
	try:
		simu()
	except rospy.ROSInterruptException:
		pass






























































