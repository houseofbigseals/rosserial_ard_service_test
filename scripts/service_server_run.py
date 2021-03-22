#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosserial_ard_service_test.srv import ButtonCheck, ButtonCheckResponse

# http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# roslib arduino custom messages
# rosrun rosserial_arduino make_libraries.py <path/to/targetdir> <package_with_crazy_msgs>


def button_check_handler(req):
    print("we got request about button {} state".format(req.pin))
    return ButtonCheckResponse(True)


def add_two_ints_server():
    rospy.init_node('check_button_server')
    s = rospy.Service('check_button', ButtonCheck, button_check_handler)
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
