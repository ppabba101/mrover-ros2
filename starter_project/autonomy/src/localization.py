#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3


class Localization(Node):
    pose: SE3

    def __init__(self):
        super().__init__("localization")
        # create subscribers for GPS and IMU data, linking them to our callback functions
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 10)

        # create a transform broadcaster for publishing to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # initialize pose to all zeros
        self.pose = SE3()

    def gps_callback(self, msg: NavSatFix):
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordinates, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """
        self.pose = SE3.from_pos_quat(Localization.spherical_to_cartesian(np.array([msg.latitude, msg.longitude]), np.array([38.4225202, -110.7844653])), self.pose.rotation.quaternion)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "rover_base_link", rclpy.time.Time())


    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        self.pose = SE3.from_pos_quat(self.pose.position, np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "rover_base_link", rclpy.time.Time())

    @staticmethod
    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        """
        This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a numpy array [latitude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latitude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a numpy array [x, y, z]
        """
        
        # radius of the Earth in meters
        R = 6371000

        # convert latitude and longitude to radians
        lat = np.radians(spherical_coord[0])
        lon = np.radians(spherical_coord[1])
        ref_lat = np.radians(reference_coord[0])
        ref_lon = np.radians(reference_coord[1])

        # calculate the differences in latitude and longitude
        dlat = lat - ref_lat
        dlon = lon - ref_lon

        # calculate the x, y, and z coordinates
        x = R * dlon * np.cos(ref_lat)
        y = R * dlat
        z = 0

        return np.array([x, y, z])


def main():
    # initialize the node
    rclpy.init()

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rclpy.spin(localization)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
