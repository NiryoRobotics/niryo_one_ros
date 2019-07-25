import time
import os
import subprocess
import rospy


class JevoisCamera:

    def __init__(self, video_devices_path, video_config_path):
        self.guvcview = None
        self.video_devices_path = video_devices_path
        self.video_config_path = video_config_path
        self.camera_device = self.find_jevois_device()

    def find_jevois_device(self):
        device_list = os.listdir(self.video_devices_path)
        for device in device_list:
            name_path = self.video_devices_path + device + '/name'
            if os.path.exists(name_path):
                with open(name_path, 'r') as f:
                    camera_name = f.readline().rstrip()
                    if camera_name.find('JeVois') != -1:
                        rospy.loginfo("Found JeVois camera device!")
                        return device
        return ''

    def set_guvcview_fps(self, fps):
        if self.camera_device == '':
            return False
        filename = self.video_config_path + self.camera_device

        with open(filename, "r+") as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if line.startswith("fps_num"):
                    lines[i] = "fps_num=1\n"
                if line.startswith("fps_denom"):
                    lines[i] = "fps_denom=" + str(fps) + "\n"

            f.seek(0)
            text_to_write = ''.join(lines)
            f.write(text_to_write)
            f.truncate()

        return True

    def start_guvcview(self):
        device_path = '/dev/' + self.camera_device
        self.guvcview = subprocess.Popen([
            'guvcview', '-d', device_path,
            '-g', 'none', '-a', 'none'#, '-r', 'none'
        ])

    def stop_guvcview(self):
        if self.guvcview:
            rospy.loginfo("Shutdown camera")
            self.guvcview.terminate()
