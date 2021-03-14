import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
import tf
import numpy as np

from assignment_1.scene import Scene


class RobotMotion():

    def __init__(self):
        self.scene = Scene()
        self.move_group = MoveGroupPythonInteface() 
        self.tf_listener = tf.TransformListener()


    @property
    def ee_pose(self):
        """[The latest robot pose from the scene]

        Returns:
            [geometry_msgs/Pose]: [description]
        """
        return self.scene.robot_pose


    def run_build(self):
        while not rospy.is_shutdown():
            scene_objects = self.scene.get_non_stacked_objects()
            
            if len(scene_objects) == self.scene.num_objects:
                object_to_stack = scene_objects[0]
                rospy.loginfo("No stacked objects. Picking {} to start stacking".format(object_to_stack.name))
                self.move_object(object_to_stack, self.scene.goal_pose)
            else:
                rospy.loginfo("We have some stacked")
            break

    def move_object(self, scene_object, pose):
        """[Moves and object to a pose and udates the gazebo interface]

        Args:
            scene_object ([SceneObject]): [description]
            pose ([[geometry_msgs/Pose]): [description]
        """
        initial_pose = scene_object.pose

        ## move just above the object
        initial_pose.position.z += 0.1
        initial_pose.orientation = self._get_downfacing_orientation()
        self._move_to_pose(initial_pose)
        rospy.sleep(0.5)

        initial_pose.position.z = scene_object.pose.position.z + 0.05
        self._move_to_pose(initial_pose)

        self.close_gripper(scene_object)

        initial_pose.position.z = 0.2
        self._move_to_pose(initial_pose)

        self._move_to_pose(pose)
        self.open_gripper(scene_object)

    def goto_each_object(self):
        scene_objects = self.scene.get_dynamic_objects()
        for scene_object in scene_objects:
            rospy.loginfo("Going to object {}".format(scene_object))
            pose = scene_object.pose

            pose.position.z += 0.1


            pose.orientation = self._get_downfacing_orientation()
            self._move_to_pose(pose)
            rospy.sleep(2)

    def goto_objects(self, scene_objects):
        for scene_object in scene_objects:
            rospy.loginfo("Going to object {}".format(scene_object))
            pose = scene_object.pose

            pose.position.z += 0.1


            pose.orientation = self._get_downfacing_orientation()
            self._move_to_pose(pose)
            rospy.sleep(2)
            
    def _get_downfacing_orientation(self):
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.707
        quat.z = 0.0
        quat.w = 0.707

        return quat


    def _move_to_pose(self, pose):
        """[Moves the EE to a pose]

        Args:
            pose ([geometry_msgs/Pose]): [The pose msg describing the point to move to.]
        """
        self.move_group.move_eef_to_pose(pose)

    def open_gripper(self, scene_object):
        """[Opens the gripper and puts the scene object down]

        Args:
            scene_object ([SceneObject]): [The scene object to release]
        """
        self.move_group.open_gripper(scene_object.name)

    def close_gripper(self, scene_object):
        """[Opens the gripper and picksup the scene object down]

        Args:
            scene_object ([SceneObject]): [The scene object to grab]
        """
        self.move_group.close_gripper(scene_object.name)
