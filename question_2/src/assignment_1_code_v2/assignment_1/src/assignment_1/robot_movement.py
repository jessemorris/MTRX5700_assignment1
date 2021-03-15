import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
import tf
import numpy as np

from assignment_1.scene import Scene
from assignment_1.utils import euclid_distance



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
            ## no objects have been stacked
            if len(scene_objects) == self.scene.num_objects:
                object_to_stack = scene_objects[0]
                goal = self.scene.goal_pose
                rospy.loginfo("No stacked objects. Picking {} to start stacking".format(object_to_stack.name))

            else:
                goal = self.scene.get_actual_goal_pose()
                object_to_stack = scene_objects[0]
                rospy.loginfo("We have some stacked")

            rospy.loginfo("To goal pose: {}".format(goal))
            self.move_object(object_to_stack, goal)

    def move_object(self, scene_object, pose):
        """[Moves and object to a pose and udates the gazebo interface]

        Args:
            scene_object ([SceneObject]): [description]
            pose ([[geometry_msgs/Pose]): [description]
        """
        initial_pose = scene_object.pose

        motion_pose = initial_pose

        ## move just above the object
        motion_pose.position.z += 0.1
        motion_pose.orientation = self._get_downfacing_orientation()
        self._move_to_pose(motion_pose)
        rospy.sleep(0.5)


        # motion_pose.position.z = initial_pose.position.z + 0.048
        motion_pose.position.z = initial_pose.position.z + 0.086
        self._move_to_pose(motion_pose)
        rospy.sleep(1) ## wait for update from topic

        rospy.loginfo(self.ee_pose.position)
        rospy.loginfo(initial_pose.position)
        offset = self.ee_pose.position.z + initial_pose.position.z
        rospy.loginfo("Offset between robot pose and object: {}".format(offset))
        ## move to pick up
        self.close_gripper(scene_object)

        ## get object
        rospy.loginfo("Collection object {}".format(scene_object.name))

        ## go up
        motion_pose.position.z += (0.05 + offset)
        self._move_to_pose(motion_pose)
        rospy.sleep(0.5)

        #go accross at same height
        motion_pose.position.x = pose.position.x
        motion_pose.position.y = pose.position.y
        motion_pose.position.z = pose.position.z + (0.086+ offset)
        self._move_to_pose(motion_pose)
        rospy.sleep(0.5)

        rospy.loginfo("Moving across above goal")

        # # go down to object before release
        pose.orientation = self._get_downfacing_orientation()
        # pose.position.z -= 0.1
        # self._move_to_pose(pose)

        rospy.loginfo("Correcting z before release")
        rospy.loginfo(pose.position.z)        
        # if pose.position.z > 0.005:
        pose.position.z += (0.086+ offset)
        self._move_to_pose(pose)
        rospy.sleep(0.1)

        self.open_gripper(scene_object)
        rospy.loginfo("Placed object {}".format(scene_object.name))

        ## go up and away
        pose.position.z += (0.1)
        self._move_to_pose(pose)
        rospy.loginfo("Moving away...")

    def update_gripper_orientation(self, target_object):
        """[Updates the gipper to an orientation over the specified scene object such that it can 
        descend over the object without collision]

        Args:
            target_object ([SceneObject]): [The scene object to check]
        """
        scene_objects = self.scene.get_dynamic_objects()
        for scene_object in scene_objects:
            if scene_object.name != target_object.name:
                check_x = scene_object.pose.position.x
                check_y = scene_object.pose.position.y

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


    def _move_to_pose(self, pose, t=0.1):
        """[Moves the EE to a pose]

        Args:
            pose ([geometry_msgs/Pose]): [The pose msg describing the point to move to.]
        """
        self.move_group.move_eef_to_pose(pose)
        rospy.sleep(t)

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
