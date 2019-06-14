#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock


'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	# get b_T_ee_desire
	trans = numpy.empty(3)
	trans[0] = command.x_target.translation.x
	trans[1] = command.x_target.translation.y
	trans[2] = command.x_target.translation.z
	rot = numpy.empty(4)
	rot[0] = command.x_target.rotation.x
	rot[1] = command.x_target.rotation.y
	rot[2] = command.x_target.rotation.z
	rot[3] = command.x_target.rotation.w
	T_trans = tf.transformations.translation_matrix(trans)
	T_rot = tf.transformations.quaternion_matrix(rot)
	T_d = numpy.dot(T_trans, T_rot)
	
	# get b_T_ee_current
	joint_transforms, T_c = self.forward_kinematics(self.q_current)
	
	# delta x
	deltax = numpy.dot(numpy.linalg.inv(T_c), T_d)

	# scale
	v_r, axis = self.rotation_from_matrix(deltax)
	v_r = numpy.dot(v_r, axis)
	p = 2
	v_t = tf.transformations.translation_from_matrix(deltax)
	v_t = numpy.dot(p, v_t)
	v_r = numpy.dot(p, v_r)
	if numpy.linalg.norm(v_r) != 0:
	    v_r = v_r / numpy.linalg.norm(v_r)
	if numpy.linalg.norm(v_r) != 0:
	    v_t = 0.1 * v_t / numpy.linalg.norm(v_t)
	# v_ee
	v_ee = numpy.hstack((v_t, v_r))
	v_ee = numpy.transpose(v_ee)
	# inverse Jacobian
	J = self.get_jacobian(T_c, joint_transforms)
	J_p = numpy.linalg.pinv(J)
	u,s,v = numpy.linalg.svd(J)
	epsilon = numpy.zeros((6,self.num_joints))
	ss = max(s)
	for i in range(len(s)):
	    if s[i] < 0.01*ss:
	        s[i] = 0
	    epsilon[i][i] = s[i]
	J_s = numpy.dot(numpy.dot(u,epsilon),v)
	J_sp = numpy.linalg.pinv(J_s)
	# q_dot_desire
	q_des = numpy.dot(J_sp, v_ee)
	# secondary objective
	q_null = numpy.zeros((self.num_joints))
	if command.secondary_objective is True:
	    q_sec = numpy.zeros((self.num_joints))
	    q_sec[0] = numpy.dot(3, (command.q0_target - self.q_current[0]))
	    q_null = numpy.dot(numpy.eye(self.num_joints) - numpy.dot(J_p, J), q_sec)
	# final q_dot_desire
	q_des = q_des + q_null
	q_des = q_des/numpy.linalg.norm(q_des)
	# publish q_dot_desire
	self.joint_velocity_msg.name = self.joint_names
	self.joint_velocity_msg.velocity = q_des
	self.velocity_pub.publish(self.joint_velocity_msg)
	
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
	for j in range(self.num_joints):
	    # j_T_ee
	    j_T_ee = numpy.dot(numpy.linalg.inv(joint_transforms[j]), b_T_ee)
	    # ee_R_j
	    ee_T_j = numpy.linalg.inv(j_T_ee)
	    ee_R_j = numpy.delete(ee_T_j, 3, axis = 0)
	    ee_R_j = numpy.delete(ee_R_j, 3, axis = 1)
	    # second term of V_j
	    t = numpy.array((j_T_ee[0][3],j_T_ee[1][3],j_T_ee[2][3]))
	    S_t = numpy.array(([0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]))
	    V_2 = numpy.dot(-ee_R_j, S_t)
	    V_j2 = numpy.vstack((V_2, ee_R_j))
	    V_j_5 = numpy.dot(V_j2, self.joint_axes[j])
	    J[:,j] = V_j_5
	    

        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
	trans = numpy.empty(3)
	trans[0] = command.translation.x
	trans[1] = command.translation.y
	trans[2] = command.translation.z
	rot = numpy.empty(4)
	rot[0] = command.rotation.x
	rot[1] = command.rotation.y
	rot[2] = command.rotation.z
	rot[3] = command.rotation.w
	T_trans = tf.transformations.translation_matrix(trans)
	T_rot = tf.transformations.quaternion_matrix(rot)
	T_d = numpy.dot(T_trans, T_rot)
	for i in range(3):
	    print(i)
	    q_c = 2 * numpy.pi * numpy.random.rand(self.num_joints)
	    t0 = rospy.Time.now()
	    while rospy.Time.now() - t0 < rospy.Duration(10):
	        joint_transforms, T_c = self.forward_kinematics(q_c)
		J = self.get_jacobian(T_c, joint_transforms)
	        deltaT = numpy.dot(numpy.linalg.inv(T_c), T_d)
	        x_r, axis = self.rotation_from_matrix(deltaT)
	        x_r = numpy.dot(x_r, axis)
	        x_t = tf.transformations.translation_from_matrix(deltaT)
	        delta_x = numpy.hstack((x_t, x_r))
		print(delta_x)
	        deltaq = numpy.dot(numpy.linalg.pinv(J), delta_x)
		q_c += deltaq
		if delta_x[0]<0.001 and delta_x[1]<0.001 and delta_x[2]<0.001 and\
delta_x[3]<0.001 and delta_x[4]<0.001 and delta_x[5]<0.001:
		    print("yes")
		    break
	    if delta_x[0]<0.001 and delta_x[1]<0.001 and delta_x[2]<0.001 and\
delta_x[3]<0.001 and delta_x[4]<0.001 and delta_x[5]<0.001:
		print("congrats")
		self.joint_command_msg.name = self.joint_names
		self.joint_command_msg.position = q_c
		self.joint_command_pub.publish(self.joint_command_msg)
		break
	
	
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
