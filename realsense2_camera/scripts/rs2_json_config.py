#!/usr/bin/env python2

# rs2_json_config.py - A simple script to allow changing of 
# realsense2_camera parameters on the command line. 
# 
# This script was written because the 'rosservice' command 
# parses arguments using the YAML parser, so inputting a JSON configuration 
# string is non-trivial. 
# 
# Author: Paul Belanger
# Date: 2020-08-06

# This script adapted from the ROS python service client tutorial: 
# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

from __future__ import print_function

import sys
import rospy
from realsense2_camera.srv import * 


def json_config_client(ns, new_config_json): 
    """
    Calls the json_config service, passing the new json configuration. 

    :param ns: The namespace of the node to modify, without the
               'realsense2_camera/json_config' suffix.  This usually 
               corresponds to the 'ns' argument passed when starting a
               realsense2_camera node. 

    :param new_config_json: A JSON string containing the new configuration to
                            apply. May be an empty string to obtain the 
                            current configuration. 

    :return: A string containing the current configuration of the
             realsense2_camera node. 
    """

    service_name = ns + "/realsense2_camera/json_config"
    rospy.wait_for_service(service_name)

    try:
        proxy = rospy.ServiceProxy(service_name, JsonConfig)
        result = proxy(new_config_json)
        return result.new_config

    except rospy.ServiceException as e:
        print("Service Call Failed: {}".format(e))

if(__name__ == "__main__"):
    if len(sys.argv) == 3:
        ns = sys.argv[1]
        new_config = sys.argv[2]

        result = json_config_client(ns, new_config)
        print(result)

    elif len(sys.argv) == 2:
        ns = sys.argv[1]

        result = json_config_client(ns, "")
        print(result)

    else:
        print("usage: {} <camera ns> [new json config string] ")
        sys.exit(1);


