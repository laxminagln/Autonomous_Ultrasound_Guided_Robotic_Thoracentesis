#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse

def handle(req):
    rospy.loginfo("Received request from PC to start UNet.")
    return TriggerResponse(success=True, message="Jetson dummy service reached successfully.")

if __name__ == "__main__":
    rospy.init_node("jetson_dummy_unet_service")
    srv = rospy.Service("/jetson/start_unet", Trigger, handle)
    rospy.loginfo("Dummy service /jetson/start_unet is ready.")
    rospy.spin()