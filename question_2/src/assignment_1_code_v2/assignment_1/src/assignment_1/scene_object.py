
class SceneObject():

    def __init__(self, starting_pose, name):
        """[Constructor for scene object. Represents some object in the scene that needs to be moved. ]

        Args:
            starting_pose ([geometry_msgs/Point]): [The starting position on the table]
        """
        self._name = name
        self._starting_pose = starting_pose
        self._current_pose = self._starting_pose

    def __repr__(self):
        return str("Scene Object [{}] : {}".format(self._name, self._current_pose))

    @property
    def pose(self):
        """[Current object pose]

        Returns:
            [geometry_msgs/Pose]: [description]
        """
        return self._current_pose

    @property
    def name(self):
        """[Name of the scene object. Used to tell the gazebo model which item this is.]

        Returns:
            [str]: [description]
        """
        return self._name


    def update_pose(self, pose):
        """[Updates current pose of object in the scene]

        Args:
            pose ([geometry_msgs/Pose]): [Pose for this object]
        """
        self._current_pose = pose

