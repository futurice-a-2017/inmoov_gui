import rospy
import threading
import math
import config
from sensor_msgs.msg import JointState

# Threading class for listening to a ROS topic
class listenerThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		# Create a subscriber
		rospy.Subscriber('joint_states', JointState, self.callback)

	def run(self):
		# Start listening
		rospy.spin()

	# Handling a new message when received
	def callback(self, data):
		# Save angle value from ROS-message to a global variable
		config.angles[0] = int(list(data.position)[0] * 180 / math.pi)
		config.angles[1] = int(list(data.position)[17] * 180 * 3 / math.pi)

