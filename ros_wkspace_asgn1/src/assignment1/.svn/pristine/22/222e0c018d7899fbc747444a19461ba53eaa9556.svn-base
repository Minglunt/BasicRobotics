#!/usr/bin/env python  
import rospy

import numpy
import math

import tf
import tf2_ros
import geometry_msgs.msg


def homeworkone():

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    T1 = numpy.dot(tf.transformations.euler_matrix(0.64, 0.64, 0.0),
                   tf.transformations.translation_matrix((1.5, 0.8, 0.0)))
    tr1 = tf.transformations.translation_from_matrix(T1)
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]
    q1 = tf.transformations.quaternion_from_matrix(T1)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    br.sendTransform(t1)


    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    T2 = numpy.dot(tf.transformations.euler_matrix(0, 1.5, 0),
                   tf.transformations.translation_matrix((0.0, 0.0, -2.0)))
    q2 = tf.transformations.quaternion_from_matrix(T2)
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    tr2 = tf.transformations.translation_from_matrix(T2)
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]
    br.sendTransform(t2)


    T3 = numpy.dot(tf.transformations.euler_matrix(0, 0, 0),
                   tf.transformations.translation_matrix((0.3, 0.0, 0.3)))
    T3_inverse = tf.transformations.inverse_matrix(T3)
    T2_inverse = tf.transformations.inverse_matrix(T2)
    T4 = numpy.dot(T3_inverse,T2_inverse)
    T5 = numpy.dot(T4,T1)
    aim = tf.transformations.translation_from_matrix(T5)
    xaxis = numpy.cross([1,0,0],aim)
    aimm = numpy.dot([1,0,0],aim)
    aimn = numpy.linalg.norm(aim)
    cbeta = aimm/aimn
    beta = math.acos(cbeta)
    q5 = tf.transformations.quaternion_about_axis(beta, xaxis)
    t5 = geometry_msgs.msg.TransformStamped()
    t5.header.stamp = rospy.Time.now()
    t5.header.frame_id = "robot_frame"
    t5.child_frame_id = "camera_frame"
    t5.transform.translation.x = 0.3
    t5.transform.translation.y = 0.0
    t5.transform.translation.z = 0.3
    t5.transform.rotation.x = q5[0]
    t5.transform.rotation.y = q5[1]
    t5.transform.rotation.z = q5[2]
    t5.transform.rotation.w = q5[3]
    br.sendTransform(t5)


if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        homeworkone()
        rospy.sleep(0.1)
