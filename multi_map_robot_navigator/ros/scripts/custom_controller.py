#!/usr/bin/env python3

from multi_map_robot_navigator.multimap_transition import MultiMapControl
import rospy

if __name__ == '__main__':
    rospy.init_node("elevator_blast")

    blast = MultiMapControl()
    rospy.spin()
