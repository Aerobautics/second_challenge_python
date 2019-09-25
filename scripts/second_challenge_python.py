#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped

def secondChallenge():
	waypointPublisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 10)
	rospy.init_node('second_challenge_python', anonymous = True)
	publishingRate = 10
	waypointRate = rospy.Rate(publishingRate) # Input in Hertz, must be greater than 2 Hz
	# Waypoint generation
	targetSpeed = 0.25 # speed in m/s
	waypoint_1 = [-2.5, 0, 3]
	waypoint_2 = [2.5, 0, 3]
	waypointLoiterTime = 15 # time in seconds
	dimensionality = 3
	waypointDistance = 0
	for var in list(range(dimensionality)):
		waypointDistance += (waypoint_1[var] - waypoint_2[var])**2
	waypointDistance = math.sqrt(waypointDistance)
	# Generate intermediate points
	numberOfPoints = int((waypointDistance * publishingRate)/ targetSpeed)
	moveIncrements = [0, 0, 0]
	for var in list(range(dimensionality)):
		moveIncrements[var] = (waypoint_2[var] - waypoint_1[var]) / numberOfPoints
	intermediatePoints = []
	for var in range(numberOfPoints):
		offset = [x * var for x in moveIncrements]
		intermediatePoints.append([sum(x) for x in zip(waypoint_1, offset)])
	intermediatePointsReversed = intermediatePoints[::-1]
	firstLoiterPoints = [waypoint_1 for var in range(waypointLoiterTime * publishingRate)]
	secondLoiterPoints = [waypoint_2 for var in range(waypointLoiterTime * publishingRate)]
	flightPath = firstLoiterPoints + intermediatePoints + secondLoiterPoints + intermediatePointsReversed
	messageCount = 0
	message = PoseStamped()
	while not rospy.is_shutdown():
		# Publish the points in the list
		message.header.seq = messageCount
		message.header.stamp = rospy.Time.now()
		indexx = messageCount % len(flightPath)
		message.pose.position.x = flightPath[indexx][0]
		message.pose.position.y = flightPath[indexx][1]
		message.pose.position.z = flightPath[indexx][2]
		waypointPublisher.publish(message)
		messageCount += 1
		rospy.loginfo("Published point %f, %f, %f" % (flightPath[indexx][0], flightPath[indexx][1], flightPath[indexx][2]))
		waypointRate.sleep()

if __name__ == '__main__':
	try:
		secondChallenge()
	except rospy.ROSInterruptException:
		pass
