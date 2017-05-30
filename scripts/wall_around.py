#!/usr/bin/env python
import rospy, copy
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
# from pimouse_ros.msg import LightSensorValues
from geometry_msgs.msg import WrenchStamped
#WrenchStamped structure is
#Header header
	#uint32 seq
	#time stamp
	#string frame_id
#Vector3 force
	#float64 x
	#float64 y
	#float64 z
#Vector3 torque
	#float64 x
	#float64 y
	#float64 z

DISTANCE_VALUE = 5
THRESHOLD_FRONT_WALL = 5
THRESHOLD_LEFT_WALL = 5
THRESHOLD_RIGHT_WALL = 5

class WallAround():
	def __init__(self):
		# Active "cmd_vel" publisher to broadcast motor frequency
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# set force sensor initail values
		self.sensor = WrenchStamped()

		self.force_x = 0
		self.force_y = 0
		self.force_z = 0

		# Active force_torque  subscriver to ger force sensor values
		rospy.Subscriber('/leptrino_force_torque/force_torque', WrenchStamped, self.callback)

	def callback(self, message):
		self.sensor = message
		if self.sensor.wrench.force.x < 0:
			self.sensor.wrench.force.x = -1 * self.sensor.wrench.force.x
		self.force_x = int(self.sensor.wrench.force.x)
		if self.sensor.wrench.force.y < 0:
			self.sensor.wrench.force.y = -1 * self.sensor.wrench.force.y
		self.force_y = int(self.sensor.wrench.force.y)
		if self.sensor.wrench.force.z < 0:
			self.sensor.wrench.force.z = -1 * self.sensor.wrench.force.z
		self.force_z = int(self.sensor.wrench.force.z)

	def wall_front(self):
		return self.force_x > THRESHOLD_FRONT_WALL  or self.force_y > THRESHOLD_FRONT_WALL

	def too_right(self):
		return self.force_x > THRESHOLD_RIGHT_WALL

	def too_left(self):
		return self.force_y > THRESHOLD_LEFT_WALL

	def run(self):
		# wake up every 20Hz(50ms)and check force sensor
		rate = rospy.Rate(20)
		data = Twist()

		data.linear.x = 0.3
		data.angular.z = 0.0
		
		while not rospy.is_shutdown():
			if self.wall_front():
				data.angular.z = - math.pi
			elif self.too_right():
				data.angular.z = math.pi
			elif self.too_left():
				data.angular.z = - math.pi
			else:
				error = DISTANCE_VALUE - self.force_z
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


