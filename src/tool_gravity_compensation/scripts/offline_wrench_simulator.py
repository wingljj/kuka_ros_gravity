#!/usr/bin/env python3
import math

import rospy
import tf
from geometry_msgs.msg import WrenchStamped


STANDARD_GRAVITY = 9.80665
MAX_MASS_KG = 200.0
MAX_COM_DISTANCE_M = 2.0
MAX_FORCE_N = 10000.0
MAX_TORQUE_NM = 1000.0


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def transpose_mat_vec(matrix, vector):
    return [
        matrix[0] * vector[0] + matrix[3] * vector[1] + matrix[6] * vector[2],
        matrix[1] * vector[0] + matrix[4] * vector[1] + matrix[7] * vector[2],
        matrix[2] * vector[0] + matrix[5] * vector[1] + matrix[8] * vector[2],
    ]


def rotation_matrix_from_quaternion(q):
    matrix4 = tf.transformations.quaternion_matrix(q)
    return [
        matrix4[0][0], matrix4[0][1], matrix4[0][2],
        matrix4[1][0], matrix4[1][1], matrix4[1][2],
        matrix4[2][0], matrix4[2][1], matrix4[2][2],
    ]


def compute_wrench(base_r_sensor, mass_kg, com_sensor_m,
                   force_bias=None, torque_bias=None, gravity=STANDARD_GRAVITY):
    force_bias = force_bias if force_bias is not None else [0.0, 0.0, 0.0]
    torque_bias = torque_bias if torque_bias is not None else [0.0, 0.0, 0.0]
    gravity_force_base = [0.0, 0.0, -mass_kg * gravity]
    force_sensor = transpose_mat_vec(base_r_sensor, gravity_force_base)
    torque_sensor = cross(com_sensor_m, force_sensor)
    return (
        [force_sensor[i] + force_bias[i] for i in range(3)],
        [torque_sensor[i] + torque_bias[i] for i in range(3)],
    )


def get_vector_param(name, default, expected_len=3):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = [float(item) for item in value.replace(",", " ").split()]
    if len(value) != expected_len:
        raise ValueError("{} must contain {} values".format(name, expected_len))
    return [float(item) for item in value]


class OfflineWrenchSimulator:

    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.sensor_frame = rospy.get_param("~sensor_frame", "sri_ft_sensor")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/sri_ft_sensor/wrench")
        self.publish_rate = float(rospy.get_param("~publish_rate", 100.0))
        self.mass_kg = float(rospy.get_param("~mass_kg", 4.0))
        self.com_sensor_m = get_vector_param("~com_sensor_m", [0.10, -0.04, 0.20])
        self.force_bias = get_vector_param("~force_bias", [0.0, 0.0, 0.0])
        self.torque_bias = get_vector_param("~torque_bias", [0.0, 0.0, 0.0])
        self.noise_force = float(rospy.get_param("~noise_force", 0.0))
        self.noise_torque = float(rospy.get_param("~noise_torque", 0.0))
        self.broadcast_fallback_tf = bool(rospy.get_param("~broadcast_fallback_tf", False))
        self.fallback_xyz = get_vector_param("~fallback_base_to_sensor_xyz", [0.0, 0.0, 0.0])
        self.fallback_rpy = get_vector_param("~fallback_base_to_sensor_rpy", [0.0, 0.0, 0.0])

        # Validate physical parameter bounds
        if self.mass_kg <= 0.0 or self.mass_kg > MAX_MASS_KG:
            raise ValueError("mass_kg must be positive and <= {} kg, got {}".format(MAX_MASS_KG, self.mass_kg))
        com_dist = math.sqrt(sum(v * v for v in self.com_sensor_m))
        if com_dist > MAX_COM_DISTANCE_M:
            raise ValueError("com_sensor_m distance {} m exceeds max {} m".format(com_dist, MAX_COM_DISTANCE_M))
        if self.noise_force < 0.0 or self.noise_torque < 0.0:
            raise ValueError("noise_force and noise_torque must be non-negative")
        if self.publish_rate <= 0.0 or self.publish_rate > 1000.0:
            raise ValueError("publish_rate must be positive and <= 1000 Hz")

        self.fallback_quaternion = tf.transformations.quaternion_from_euler(
            self.fallback_rpy[0], self.fallback_rpy[1], self.fallback_rpy[2])
        self.fallback_base_r_sensor = rotation_matrix_from_quaternion(self.fallback_quaternion)
        self.pub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=20)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

    def lookup_base_r_sensor(self, stamp):
        try:
            _, quaternion = self.listener.lookupTransform(self.base_frame, self.sensor_frame, rospy.Time(0))
            return rotation_matrix_from_quaternion(quaternion)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            if self.broadcast_fallback_tf:
                rospy.logwarn_throttle(2.0, "Using fallback base->sensor TF: %s", exc)
                self.broadcaster.sendTransform(
                    self.fallback_xyz,
                    self.fallback_quaternion,
                    stamp,
                    self.sensor_frame,
                    self.base_frame,
                )
                return self.fallback_base_r_sensor
            rospy.logwarn_throttle(2.0, "Waiting for TF %s -> %s: %s",
                                   self.base_frame, self.sensor_frame, exc)
            return None

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            base_r_sensor = self.lookup_base_r_sensor(now)
            if base_r_sensor is None:
                rate.sleep()
                continue
            force, torque = compute_wrench(
                base_r_sensor,
                self.mass_kg,
                self.com_sensor_m,
                self.force_bias,
                self.torque_bias,
            )
            msg = WrenchStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.sensor_frame
            msg.wrench.force.x = force[0] + self.noise_force * math.sin(now.to_sec() * 3.1)
            msg.wrench.force.y = force[1] + self.noise_force * math.sin(now.to_sec() * 2.3)
            msg.wrench.force.z = force[2] + self.noise_force * math.sin(now.to_sec() * 1.7)
            msg.wrench.torque.x = torque[0] + self.noise_torque * math.sin(now.to_sec() * 2.9)
            msg.wrench.torque.y = torque[1] + self.noise_torque * math.sin(now.to_sec() * 2.1)
            msg.wrench.torque.z = torque[2] + self.noise_torque * math.sin(now.to_sec() * 1.3)

            # NaN/Inf guard: skip publishing non-finite wrench values
            wrench_vals = [
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ]
            if any(not math.isfinite(v) for v in wrench_vals):
                rospy.logwarn_throttle(2.0, "Skipping simulated wrench with NaN/Inf values")
                rate.sleep()
                continue
            if any(abs(v) > MAX_FORCE_N for v in wrench_vals[:3]) or \
               any(abs(v) > MAX_TORQUE_NM for v in wrench_vals[3:]):
                rospy.logwarn_throttle(2.0, "Skipping simulated wrench exceeding sensor range")
                rate.sleep()
                continue

            self.pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("offline_wrench_simulator")
    OfflineWrenchSimulator().spin()


if __name__ == "__main__":
    main()
