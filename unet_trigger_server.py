#!/usr/bin/env python3
import os
import subprocess
import threading

import rospy
from std_msgs.msg import Empty


class UnetTriggerSubscriber:
    def __init__(self):
        self.topic_name = rospy.get_param("~topic_name", "/jetson/start_unet")
        self.unet_command = rospy.get_param(
            "~unet_command",
            "python3 /home/laxmi/sonon_unet/realtime_scrcpy_unet.py",
        )
        self.working_dir = rospy.get_param("~working_dir", "/home/laxmi/sonon_unet")
        self.log_file = rospy.get_param("~log_file", "/home/laxmi/unet_runtime.log")
        self.allow_restart = rospy.get_param("~allow_restart", False)

        self._lock = threading.Lock()
        self._process = None
        self._trigger_count = 0

        self._sub = rospy.Subscriber(self.topic_name, Empty, self._callback, queue_size=1)
        rospy.loginfo("UNet trigger subscriber ready on %s", self.topic_name)
        rospy.loginfo("UNet command: %s", self.unet_command)
        rospy.loginfo("UNet log file: %s", self.log_file)

    def _is_running(self):
        return self._process is not None and self._process.poll() is None

    def _callback(self, _msg):
        with self._lock:
            self._trigger_count += 1
            rospy.loginfo("Received UNet trigger message #%d", self._trigger_count)

            if self._is_running() and not self.allow_restart:
                rospy.loginfo("UNet already running. Ignoring trigger.")
                return

            if self._is_running() and self.allow_restart:
                rospy.logwarn("Restart requested. Terminating previous UNet process.")
                self._process.terminate()
                try:
                    self._process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()

            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            log_handle = open(self.log_file, "a")
            self._process = subprocess.Popen(
                self.unet_command,
                shell=True,
                cwd=self.working_dir,
                stdout=log_handle,
                stderr=log_handle,
                executable="/bin/bash",
            )
            rospy.loginfo("Started UNet process with PID %s", self._process.pid)
            rospy.loginfo("Check progress with: tail -f %s", self.log_file)


if __name__ == "__main__":
    rospy.init_node("unet_trigger_subscriber")
    UnetTriggerSubscriber()
    rospy.spin()