#!/usr/bin/env python

from multi_map_robot_navigator.elevator_manager import ElevatorControl
import rospy

if __name__ == '__main__':
    rospy.init_node("elevator_blast")

    blast = ElevatorControl()
    rospy.spin()
