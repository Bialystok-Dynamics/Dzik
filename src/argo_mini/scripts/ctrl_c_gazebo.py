#!/usr/bin/python3

import atexit
import rospy
import subprocess


def exit_handler():
    print('Application is ending...')

    bash_cmd = "killall -9 gzclient"
    process = subprocess.Popen(bash_cmd.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    print(f'Bash\toutput: {output}\n'
          f'\terror: {error}')

    bash_cmd = "killall -9 gzserver"
    process = subprocess.Popen(bash_cmd.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    print(f'Bash\toutput: {output}\n'
          f'\terror: {error}')


if __name__ == "__main__":
    atexit.register(exit_handler)
    rospy.init_node('ctrl_c_gazebo')

    rospy.spin()
