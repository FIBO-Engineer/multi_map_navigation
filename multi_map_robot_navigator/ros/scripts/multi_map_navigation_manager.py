#!/usr/bin/env python3
import rospy
from multi_map_robot_navigator.srv import *
from std_srvs.srv import Empty
from multi_map_robot_navigator.map_manager import MultiMapManager
from multi_map_robot_navigator.map_navigator import MultiMapNavigationNavigator

def handle_reinit_call(req):
    rospy.loginfo("Re-initializing Multi-Map navigator...")
    #manager.create_map_db()

    if (not manager.loadyaml(manager.definition_file)):
        rospy.logerr("Re-initialization FAILED. Could not load updated YAML file")
        return Empty
    return Empty

if (__name__ == "__main__"):
    rospy.init_node('multi_map_navigation')
    manager = MultiMapManager()
    navigator = MultiMapNavigationNavigator(manager)

    reinit_service = rospy.Service('map_manager/reinit', ReinitManager, handle_reinit_call)

    while not rospy.is_shutdown():
        manager.publish_markers()
        rospy.sleep(1.0)
