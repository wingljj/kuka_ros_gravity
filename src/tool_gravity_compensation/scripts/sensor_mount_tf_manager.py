#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tool_gravity_compensation.srv import SetSensorMount, SetSensorMountResponse

MAX_TRANSLATION_M = 5.0
MAX_ROTATION_RAD = 2.0 * math.pi


def make_transform(parent_frame, child_frame, xyz, rpy, stamp=None):
    transform = TransformStamped()
    transform.header.stamp = stamp if stamp is not None else rospy.Time(0)
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(xyz[0])
    transform.transform.translation.y = float(xyz[1])
    transform.transform.translation.z = float(xyz[2])
    q = tf.transformations.quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]
    return transform


def vector_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = [float(item) for item in value.replace(",", " ").split()]
    if len(value) != 3:
        raise ValueError("{} must have exactly 3 values".format(name))
    return [float(item) for item in value]


class SensorMountTfManager:

    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "tool0")
        self.child_frame = rospy.get_param("~child_frame", "sri_ft_sensor")
        self.xyz = vector_param("~xyz", [0.0, 0.0, 0.0])
        self.rpy = vector_param("~rpy", [0.0, 0.0, 0.0])
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))

        # Validate bounds
        for i, val in enumerate(self.xyz):
            if not math.isfinite(val) or abs(val) > MAX_TRANSLATION_M:
                raise ValueError("xyz[{}]={} exceeds allowed range +/-{} m".format(i, val, MAX_TRANSLATION_M))
        for i, val in enumerate(self.rpy):
            if not math.isfinite(val) or abs(val) > MAX_ROTATION_RAD:
                raise ValueError("rpy[{}]={} exceeds allowed range +/-{} rad".format(i, val, MAX_ROTATION_RAD))
        if self.publish_rate <= 0.0 or self.publish_rate > 200.0:
            raise ValueError("publish_rate must be positive and <= 200 Hz")

        self.broadcaster = tf.TransformBroadcaster()
        self.transform = make_transform(self.parent_frame, self.child_frame, self.xyz, self.rpy)
        self.service = rospy.Service("set_sensor_mount", SetSensorMount, self.handle_set_sensor_mount)

    def handle_set_sensor_mount(self, request):
        response = SetSensorMountResponse()
        parent = request.parent_frame or self.parent_frame
        child = request.child_frame or self.child_frame
        if not parent or not child:
            response.success = False
            response.message = "parent_frame and child_frame are required"
            response.transform = self.transform
            return response

        self.parent_frame = parent
        self.child_frame = child
        self.xyz = [request.x, request.y, request.z]
        self.rpy = [request.roll, request.pitch, request.yaw]
        self.transform = make_transform(self.parent_frame, self.child_frame, self.xyz, self.rpy)
        if request.save_to_params:
            rospy.set_param("~parent_frame", self.parent_frame)
            rospy.set_param("~child_frame", self.child_frame)
            rospy.set_param("~xyz", self.xyz)
            rospy.set_param("~rpy", self.rpy)
            rospy.set_param("/tool_gravity_compensation/sensor_mount/parent_frame", self.parent_frame)
            rospy.set_param("/tool_gravity_compensation/sensor_mount/child_frame", self.child_frame)
            rospy.set_param("/tool_gravity_compensation/sensor_mount/xyz", self.xyz)
            rospy.set_param("/tool_gravity_compensation/sensor_mount/rpy", self.rpy)

        response.success = True
        response.message = "Updated sensor mount transform {} -> {}".format(self.parent_frame, self.child_frame)
        response.transform = self.transform
        return response

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.transform.header.stamp = rospy.Time.now()
            t = self.transform.transform.translation
            q = self.transform.transform.rotation
            self.broadcaster.sendTransform(
                (t.x, t.y, t.z),
                (q.x, q.y, q.z, q.w),
                self.transform.header.stamp,
                self.transform.child_frame_id,
                self.transform.header.frame_id,
            )
            rate.sleep()


def main():
    rospy.init_node("sensor_mount_tf_manager")
    SensorMountTfManager().spin()


if __name__ == "__main__":
    main()
