from PyQt5 import QtWidgets, QtGui
from mainwindow import Ui_MainWindow
import sys
import rospy
import subprocess
import os

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.status = False
        self.path = '..'
        self.ui.btnRecord.clicked.connect(self.recordBag)

    def recordBag(self):
        if self.status == False:
            self.ui.btnRecord.setText('Recording..')
            self.status=True
            rospy.loginfo(rospy.get_name() + ' start recording.')
            command = "rosbag record -a test --split --duration 5s"
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.path)
            rospy.loginfo(self.p.pid)


        else:
            self.ui.btnRecord.setText('Record')
            self.status=False
            self.stop_recording_handler()

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()

        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def stop_recording_handler(self):
        rospy.loginfo(rospy.get_name() + ' stop recording.')
        self.terminate_ros_node("/record")

def main():
    rospy.init_node('rosbag_record')
    rospy.loginfo(rospy.get_name() + ' start')

    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()