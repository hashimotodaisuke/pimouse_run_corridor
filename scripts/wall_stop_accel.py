#!/usr/bin/env python
import rospy, copy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallStop():
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
		rate = rospy.Rate(10)
		data = Twist()

		accel = 0.02
		data.linear.x = 0.0

		while not rospy.is_shutdown():
			data.linear.x += accel
	
			if   self.sensor_values.sum_all >= 200: data.linear.x = 0.0
			elif data.linear.x <= 0.2:              data.linear.x = 0.2
			elif data.linear.x >= 0.8:              data.linear.x = 0.8    
			
			self.cmd_vel.publish(data)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('wall_stop')
	
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
	WallStop().run()


