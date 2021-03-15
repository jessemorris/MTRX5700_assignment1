import rospy
from gazebo_msgs.msg import ModelStates
from assignment_1.scene_object import SceneObject
from assignment_1.utils import euclid_distance
import math


class Scene():

    def __init__(self):

        ## subscriber to get the state of the environment.
        self.model_status_sub = rospy.Subscriber("/gazebo/model_states",ModelStates, self.model_states_callback)
        self.dist_threshold = float(rospy.get_param("/build_tower/dist_threshold"))
        self._is_first_callback = True
        self.goal_pose = None
        self.robot_pose = None

        ## We will use these partially for setting constrains on motion
        self.ground_plane = None
        self.table = None

        # Dictionary of dynamic objects, key is name
        self.object_dict = {}

    @property
    def num_objects(self):
        return len(self.object_dict)

    def model_states_callback(self, model_states):
        """[Callback for /gazebo/model_states]

        Args:
            model_states ([gazebo_msgs/ModelStates]): [Latest information about the environment]
        """
        names = model_states.name
        poses = model_states.pose


        for name, pose in zip(names, poses):
            if name == "goal":
                self.goal_pose = pose
                rospy.loginfo_once("Setting goal pose: {}".format(self.goal_pose))
            elif name == "ground_plane":
                self.ground_plane = pose
            elif name == "table":
                self.table = pose
            elif name == "robot":
                self.robot_pose = pose
            else:
                if self._is_first_callback:
                    self.object_dict[name] = SceneObject(pose, name)
                else:
                    self.object_dict[name].update_pose(pose)

        self._is_first_callback = False

    def get_dynamic_objects(self):
        """[Returns list of objects to move]

        Returns:
            [list[SceneObjects]]: [All dynamic scene objects]
        """
        return list(self.object_dict.values())

    def get_actual_goal_pose(self):
        """[Gets the pose of the object that we want to stack ontop of. This is the heightest object
        that is near the goal pose.]

        Returns:
            [geometry_msgs/Pose]: [Pose of the object]
        """
        scene_objects = self.get_dynamic_objects()

        ## sort objects by height
        objects_by_height = Scene.sort_scene_objects(scene_objects)

        poses_at_goal = []

        for scene_object in objects_by_height:
            pose = scene_object.pose 
            x = pose.position.x 
            y = pose.position.y

            dist_from_goal = euclid_distance(x, y, self.goal_pose.position.x, self.goal_pose.position.y)
            if dist_from_goal < self.dist_threshold:
                poses_at_goal.append(scene_object)
                rospy.loginfo("Scene object {} is at goal".format(scene_object.name))

        if len(poses_at_goal) > 0:
            heighest_object = Scene.get_heighest_object(poses_at_goal)
            rospy.loginfo("Heightest object {}".format(heighest_object))
            return heighest_object.pose
        else:
            rospy.logfatal("No objects a goal pose. Using default pose {}".format(self.goal_pose))
            #should never get here due to check
            return self.goal_pose


    def get_non_stacked_objects(self):
        scene_objects = self.get_dynamic_objects()

        ## sort objects by height
        objects_by_height = Scene.sort_scene_objects(scene_objects)

        non_stacked_objects = []

        for scene_object in objects_by_height:
            pose = scene_object.pose 
            x = pose.position.x 
            y = pose.position.y

            dist_from_goal = euclid_distance(x, y, self.goal_pose.position.x, self.goal_pose.position.y)
            if dist_from_goal > self.dist_threshold:
                rospy.loginfo("Scene object {} is {}m away from goal".format(scene_object.name, dist_from_goal))
                non_stacked_objects.append(scene_object)

        return non_stacked_objects

    @staticmethod
    def sort_scene_objects(scene_objects, reverse=True):
        """[Sorts a list of scene objects by height (object.pose.z). ]

        Args:
            scene_objects ([List[SceneObjects]]): [description]
            reverse (bool, optional): [If true, orders the sorting such that the largest object is at the front]. Defaults to True.

        Returns:
            [List[SceneObjects]]: [description]
        """
        return sorted(scene_objects, key=lambda object: object.pose.position.z, reverse=reverse)

    @staticmethod
    def get_heighest_object(scene_objects):
        """[Gets heighest object from the list]

        Args:
            scene_objects ([[List[SceneObjects]]): [description]
        """
        return Scene.sort_scene_objects(scene_objects)[0]







