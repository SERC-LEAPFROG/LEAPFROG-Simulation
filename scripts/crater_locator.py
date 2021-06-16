#!/usr/bin/env python
"""
Script to determine which craters are with a certain radius of the vehicle and
publish the craters respective locations for ROS.
"""
import math
import rospy

from geometry_msgs.msg import PoseArray, Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class CraterLocator:
    CRATER_VIEW_RADIUS = 40  # meters

    def __init__(self):
        rospy.init_node("crater_locator", anonymous=True)

        self.craters = []
        self.vehicle_position = None

        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_callback
        )

        self.crater_locations_pub = rospy.Publisher(
            "/gazebo/crater_locations", PoseArray, queue_size=10
        )

        rospy.spin()

    def model_states_callback(self, model_states):
        """Extract the location the number and locations craters in Gazebo.

        While this runs as a callback function, it only needs to extract the
        location once and will return early once the crater locations are found.

        """
        # string used to identify crater models in Gazebo
        crater_identifier = "crater"
        vehicle_identifier = "lander"
        craters_info = []
        crater_locations = PoseArray()

        # get positions of all craters and the vehicle itself
        for i in range(len(model_states.name)):
            if crater_identifier in model_states.name[i]:
                craters_info.append(
                    (model_states.pose[i].position, model_states.name[i])
                )
            if vehicle_identifier in model_states.name[i]:
                vehicle_position = model_states.pose[i].position

        # determine distance and location of craters with respect to vehicle
        for crater, crater_name in craters_info:
            crater_distance, relative_position = self.dist2crater(
                vehicle_position, crater
            )
            if crater_distance <= self.CRATER_VIEW_RADIUS:
                # reinitialize crater relative position every iteration
                crater_position = Pose()

                # unpack position into new Pose msg
                crater_position.position.x = relative_position[0]
                crater_position.position.y = relative_position[1]

                # z component of Pose msg corresponds to type of crater
                crater_position.position.z = self.determine_depth(crater_name)

                crater_locations.poses.append(crater_position)

        self.crater_locations_pub.publish(crater_locations)

    @staticmethod
    def dist2crater(vehicle: Pose, crater: Pose):
        """Determine distance and relative position between a vehicle and
        crater.

        Args:
            crater: position of crater to compare to

        Returns:
            distance: xy distance from the vehicle to the desired crater
            relative_position: the position of the crater relative to the vehicle

        """
        # distance in xy plane from vehicle to crater
        distance = math.sqrt((crater.x - vehicle.x) ** 2 + (crater.y - vehicle.y) ** 2)

        # relative xy position of crater with respect to vehicle
        relative_position = [vehicle.x - crater.x, vehicle.y - crater.y]

        return distance, relative_position

    @staticmethod
    def determine_depth(crater_name: str) -> int:
        """Determine depth of crater based off of crater name.

        Craters are named crater_[x_n] where "x" is the depth of the crater and
        "n" incrementes the amount of craters alphabetically. The first crater
        of a type will ignore "n"

        Ex.
        crater_1, crater_1_b, crater_1_c, ...
        ...
        crater_4, crater_4_b, crater_4_c, ...

        Args:
            crater_name: Name of crater for determination

        """
        if "1" in crater_name:
            depth = 1
        elif "2" in crater_name:
            depth = 2
        elif "3" in crater_name:
            depth = 3
        elif "4" in crater_name:
            depth = 4
        else:
            depth = -1

        return depth


if __name__ == "__main__":
    try:
        cl = CraterLocator()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping crater_locator node")
        pass
