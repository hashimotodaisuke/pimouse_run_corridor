#!/usr/bin/env python
import rospy, copy
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

DISTANCE_VALUE = 50

class WallAround():
	def __init__(self):
		# Active "cmd_vel" publisher to broadcast motor frequency
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# Get initail lightsensor values
		self.sensor_values = LightSensorValues()

		# Active lightsensor subscriver to ger rightsensor values
		rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

	def callback(self, message):
		self.sensor_values = message

	def wall_front(self, ls):
		return ls.left_forward > 50 or ls.right_forward > 50

	def too_right(self, ls):
		return ls.right_side > 50

	def too_left(self, ls):
		return ls.left_side > 50

	def run(self):
		# wake up every 20Hz(50ms)and check lightsensor
		rate = rospy.Rate(20)
		data = Twist()

		data.linear.x = 0.3
		data.angular.z = 0.0
		
		while not rospy.is_shutdown():
			if self.wall_front(self.sensor_values):
				data.angular.z = - math.pi
			elif self.too_right(self.sensor_values):
				data.angular.z = math.pi
			elif self.too_left(self.sensor_values):
				data.angular.z = - math.pi
			else:
				error = DISTANCE_VALUE - self.sensor_values.left_side
				data.angular.z = error * math.pi / 180.0
			
			# publish for travis test
			self.cmd_vel.publish(data)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('wall_around')
	# rospy.init_node('wall_around', log_level=rospy.DEBUG)
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
	WallAround().run()


