#!/usr/bin/env python
import unittest, rostest
import rosnode, rospy
import time

class WallStopTest(unittest.TestCase):
	def set_and_get(self, left_front, left_side, right_side, right_front):
		with open ("/dev/rtlightsensor0", 'w') as f:
			f.write("%d %d %d %d\n" % (right_front, right_side, left_side, left_front))

		time.sleep(0.3)
		
		with open("/dev/rtmotor_raw_l0", 'r') as left_front,\
			open("/dev/rtmotor_raw_r0", 'r') as right_front:
			left = int(left_front.readline().rstrip())
			right = int(right_front.readline().rstrip())
		return left, right

	def test_io(self):
		# If sum of rtsensor value was more than 500, rtmotor will be stoped
                # If sum of rtseosor value was less than 499, rtmotor will be moved

		left, right = self.set_and_get(400, 100, 100, 0) #total: 600
		self.assertTrue(left==right==0, "can't stop")
		
		left, right = self.set_and_get(100, 0, 0, 99) #total 199
		self.assertTrue(0<left==right<1000, "can't move again")
		
		time.sleep(5.0)	
		left, right = self.set_and_get(0, 0, 100, 99) #total 199
		self.assertTrue(2000<left==right, "can't accerelate")

		left, right = self.set_and_get(50, 50, 50, 50) #toral 200
		self.assertTrue(left==right==0, "can't stop again")

if __name__ == '__main__':
	time.sleep(3)
	rospy.init_node('travis_test_wall_stop')
	rostest.rosrun('pimouse_run_corridor', 'travis_test_wall_stop', WallStopTest)

	
