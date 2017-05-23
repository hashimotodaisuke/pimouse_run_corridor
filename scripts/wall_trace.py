#!/usr/bin/env python
import rospy, copy
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

DISTANCE_VALUE = 50

class WallTrace():
	def __init__(self):
		# Active "cmd_vel" publisher to broadcast motor frequency
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# Get initail lightsensor values
		self.sensor_values = LightSensorValues()

		# Active lightsensor subscriver to ger rightsensor values
		rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

	def callback(self, message):
		self.sensor_values = message

	def run(self):
		# wake up every 20Hz(50ms)and check lightsensor
		rate = rospy.Rate(20)
		data = Twist()

		accel = 0.02

		while not rospy.is_shutdown():
			s = self.sensor_values
			data.linear.x += accel
	
			if   s.sum_forward >= 50: 
				data.linear.x = 0.0
				rospy.loginfo("stop by det front wall="+str(s.sum_forward))
			elif data.linear.x <= 0.2:data.linear.x = 0.2
			elif data.linear.x >= 0.8:data.linear.x = 0.8 
			   
			# pimouse run in only left side

			# pimouse should not run around when it stops
			if data.linear.x < 0.2:   data.angular.z = 0.0
			# pimouse should not turn left when there is no wall
			elif s.left_side < 10:    data.angular.z = 0.0
			else:
				# target means the distance between wall and pimouse is 1cm
				# this value is decided in examination
				target = DISTANCE_VALUE
				# if s.left_side is +150(error=-2), the distance is 2cm(near)
				# if s.left_side is -150(error=+2), the distance is 2cm(far)
				error = (target - s.left_side)/50.0
				# set 3 degree/distance(cm) with radian change
				data.angular.z = error*3*math.pi / 180.0
				rospy.loginfo("err="+str(error)+",angl="+str(data.angular))
			
			# publish for travis test
			self.cmd_vel.publish(data)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('wall_trace')
	
	# Motor class@pimouse_ros provid /motor_on and /motor_off service
	rospy.wait_for_service('/motor_on')  # wait for motor on service wake up
	rospy.wait_for_service('/motor_off') # wait for motor off service wake up

	# If this node was terminated, the motor will be off( call rospyServiceProxy with "motor_off"
	rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
	# Above is same as following
	# off = rospy.ServiceProxy('/motor_off', Trigger)
	# rospy.on_shutdown(off())

	# call "motor_on" service handler
	rospy.ServiceProxy('/motor_on', Trigger).call()

	#run until terminate
	WallTrace().run()


