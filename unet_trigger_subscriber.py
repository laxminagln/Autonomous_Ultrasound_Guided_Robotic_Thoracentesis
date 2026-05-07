#!/usr/bin/env python3
import os
import signal
import subprocess
import time

import rospy


class UnetTriggerParamWatcher:
    def __init__(self):
        self.trigger_param = rospy.get_param('~trigger_param', '/jetson/start_unet_seq')
        self.trigger_time_param = rospy.get_param('~trigger_time_param', '/jetson/start_unet_stamp')
        self.unet_command = rospy.get_param('~unet_command', 'python3 /home/laxmi/sonon_unet/realtime_scrcpy_unet_param_signal.py')
        self.working_dir = rospy.get_param('~working_dir', '/home/laxmi/sonon_unet')
        self.log_file = rospy.get_param('~log_file', '/home/laxmi/unet_runtime.log')
        self.poll_hz = float(rospy.get_param('~poll_hz', 4.0))
        self.kill_on_shutdown = bool(rospy.get_param('~kill_on_shutdown', True))

        self.last_seq = int(rospy.get_param(self.trigger_param, 0))
        self.unet_process = None
        self.log_handle = None

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo('UNet param watcher ready.')
        rospy.loginfo('Trigger param: %s', self.trigger_param)
        rospy.loginfo('Trigger time param: %s', self.trigger_time_param)
        rospy.loginfo('UNet command: %s', self.unet_command)
        rospy.loginfo('Kill UNet on watcher shutdown: %s', self.kill_on_shutdown)

    def _start_unet(self):
        if self.unet_process is not None and self.unet_process.poll() is None:
            rospy.logwarn('UNet is already running with PID %s', self.unet_process.pid)
            return

        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        self.log_handle = open(self.log_file, 'a', buffering=1)
        self.log_handle.write('\n===== START %s =====\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
        self.log_handle.flush()

        # Start in a fresh process group so we can terminate the whole tree.
        self.unet_process = subprocess.Popen(
            self.unet_command,
            shell=True,
            cwd=self.working_dir,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        rospy.loginfo('Started UNet process with PID %s', self.unet_process.pid)
        rospy.loginfo('Check progress with: tail -f %s', self.log_file)

    def _stop_unet(self, reason='requested'):
        if self.unet_process is None:
            rospy.loginfo('No UNet process to stop (%s).', reason)
            return

        if self.unet_process.poll() is not None:
            rospy.loginfo('UNet process already exited with code %s (%s).', self.unet_process.returncode, reason)
            self.unet_process = None
            if self.log_handle:
                self.log_handle.flush()
                self.log_handle.close()
                self.log_handle = None
            return

        pid = self.unet_process.pid
        try:
            pgid = os.getpgid(pid)
            rospy.loginfo('Stopping UNet process group %s (%s)...', pgid, reason)
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            rospy.logwarn('SIGTERM failed for UNet process %s: %s', pid, e)

        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.unet_process.poll() is not None:
                break
            time.sleep(0.1)

        if self.unet_process.poll() is None:
            try:
                pgid = os.getpgid(pid)
                rospy.logwarn('UNet still running. Sending SIGKILL to process group %s...', pgid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception as e:
                rospy.logwarn('SIGKILL failed for UNet process %s: %s', pid, e)

        # Give it a moment to settle.
        time.sleep(0.2)
        rc = self.unet_process.poll()
        rospy.loginfo('UNet process stopped with return code %s.', rc)
        self.unet_process = None

        if self.log_handle:
            try:
                self.log_handle.write('===== STOP %s =====\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
                self.log_handle.flush()
                self.log_handle.close()
            except Exception:
                pass
            self.log_handle = None

    def _on_shutdown(self):
        if self.kill_on_shutdown:
            self._stop_unet(reason='watcher shutdown')

    def spin(self):
        rate = rospy.Rate(self.poll_hz)
        while not rospy.is_shutdown():
            try:
                current_seq = int(rospy.get_param(self.trigger_param, 0))
            except Exception:
                current_seq = self.last_seq

            if current_seq > self.last_seq:
                stamp = rospy.get_param(self.trigger_time_param, '')
                rospy.loginfo('Detected new UNet trigger sequence: %s -> %s (stamp=%s)', self.last_seq, current_seq, stamp)
                self.last_seq = current_seq
                self._start_unet()

            # Housekeeping log if process has exited by itself.
            if self.unet_process is not None and self.unet_process.poll() is not None:
                rospy.loginfo('UNet process exited on its own with code %s.', self.unet_process.returncode)
                self.unet_process = None
                if self.log_handle:
                    self.log_handle.flush()
                    self.log_handle.close()
                    self.log_handle = None

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('unet_trigger_param_watcher')
    watcher = UnetTriggerParamWatcher()
    watcher.spin()
