#! /usr/bin/env python

import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray


def callback(input_msg):
    print(input_msg)


def main():
    try:
        raw_input("Press Enter to build a tower")     # waits for Enter
       
        # Write your code here
        # 1. You can write as many additional functions as you need.
        #    But please keep all your code in this file.
        #
        # 2. Necessary modules have been imported.
        #    You may only import additional system or standard python modules.
        #    If importing any special modules, please check with your tutors if you are allowed to do so.
        #    Importing 'tf' and 'tf2' packages for quaternion/rotation transformations is allowed.
        #
        # 3. Please write clean code and comment is sufficiently.
        #
        # 4. Be aware that the path planner in MoveIt! is inherently non-deterministic.
        #    Paths between different runs may be completely different.
        #
        # 5. You may be tempted to use the pick-and-place example from the MoveIt! tutorials online,
        #    but be warned that that will not work.
        #
        # 6. Subscribe to ROS topic "/gazebo/model_states" to get the blocks and goal positions. 
        #

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
