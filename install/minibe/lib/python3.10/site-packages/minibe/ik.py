#!/usr/bin/env python3

import math
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from urdf_parser_py.urdf import URDF
import random
import transforms3d
import transforms3d._gohlketransforms as tf
from threading import Thread, Lock

'''This is a class which will perform inverse
   kinematics'''
class IK(Node):
    def __init__(self):
        super().__init__('ik')
    #Load robot from parameter server
        # self.robot = URDF.from_parameter_server()
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        # print(robot_desription_text)
        self.robot = URDF.from_xml_string(robot_desription_text)

    #Subscribe to current joint state of the robot
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.get_joint_state, 10)

    #This is a mutex
        self.mutex = Lock()
    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()


        #Subscribers and publishers for numerical IK
        self.ik_command_sub = self.create_subscription(
            Transform, '/ik_command', self.get_ik_command, 10)
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
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

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = np.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        # Implement your code here
        # joint_transforms is a list of 4x4 homogeneous transformation matrices from 
        # the base to each joint
        # to get jacobian, we need to construct V_j for each joint and extract
        # the column corresponding to that joint's axis
        # to construct V_j, we need Ree_j the rotation matrix from the end effector
        # to the joint and S(t_j_ee)
        #--------------------------------------------------------------------------
        #get transformation matrix from ee to base
        ee_T_b = self.get_inverse_matrix(b_T_ee)

        for j in range(self.num_joints):
            b_T_j = joint_transforms[j]
            ee_T_j = np.dot(ee_T_b, b_T_j)
            ee_R_j = ee_T_j[:3, :3]
            j_t_ee = np.dot(-ee_R_j.T, ee_T_j[:3, 3])
            x, y, z = j_t_ee
            S_j_t_ee = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            neg_ee_R_j_S_j_t_ee = np.dot(-ee_R_j, S_j_t_ee)
            top = np.concatenate((ee_R_j, neg_ee_R_j_S_j_t_ee), axis=1)
            bottom = np.concatenate((np.zeros((3, 3)), ee_R_j), axis=1)
            V_j = np.concatenate((top, bottom), axis=0)
            axis = np.argwhere(self.joint_axes[j])[0][0]+3#find axis of rotation
            J[:, j] = V_j[:, axis]
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
        # Implement your code here
        np.random.seed(0) #fix seed
        b_T_ee_des = self.transform_to_hom(command) #desired end effector transformation
        angle, axis = self.rotation_from_matrix(b_T_ee_des[:3, :3])
        rot_des = angle * axis
        trans_des = b_T_ee_des[:3, 3]
        x_des = np.concatenate((trans_des, rot_des))#desired pose wrt robot base
        

        max_iterations = 3 #try 3 times max
        max_time = 10 #10 seconds per iteration
        current_iteration = 1
        error_mag = 9999 #current error
        max_error = 0.01
        LR = 1
        q_c = np.zeros(self.num_joints)
        
        while(error_mag >= max_error and current_iteration <= max_iterations):
            q_c = 2*np.pi*np.random.rand(self.num_joints) #initial guess
            ts = time.time() #get start time
            tc = ts #get current time
            print("STARTING IK")
            while(tc - ts < max_time):
                joint_transforms, b_T_ee_cur = self.forward_kinematics(q_c)
                ee_T_b_cur = self.get_inverse_matrix(b_T_ee_cur)
                ee_cur_T_ee_des = np.dot(ee_T_b_cur, b_T_ee_des) #get transformation from current to desired
                angle, axis = self.rotation_from_matrix(ee_cur_T_ee_des[:3,:3])
                rot_des = angle * axis #get canonical axis rotations
                deltax = np.concatenate((ee_cur_T_ee_des[:3, 3], rot_des))
 
                J = self.get_jacobian(b_T_ee_cur, joint_transforms)
                Jp = np.linalg.pinv(J) 
                deltaq = np.dot(Jp, deltax)
            
                error_mag = np.linalg.norm(deltax)
                if(error_mag < max_error):
                    break
                #print(error_mag)
                q_c = q_c + LR*deltaq
                tc = time.time()
            current_iteration += 1
        
        #publish solution
        joint_positions = [float(i) for i in q_c]
        self.joint_command_msg.name = self.joint_names
        self.joint_command_msg.position = joint_positions
        self.joint_command_pub.publish(self.joint_command_msg)
        
        #-----------------------------------------------,---------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)''' 

    def get_inverse_matrix(self, a_T_b):
        b_R_a = a_T_b[:3, :3].T #inverse rotation matrix is its transpose
        b_t_a = np.dot(-b_R_a, a_T_b[:3, 3])
        b_T_a = transforms3d.affines.compose(b_t_a, b_R_a, (1,1,1))
        return b_T_a
    def transform_to_hom(self, t):
        x = t.translation.x
        y = t.translation.y
        z = t.translation.z
        rx = t.rotation.x
        ry = t.rotation.y
        rz = t.rotation.z
        w = t.rotation.w
 
        rot = transforms3d.quaternions.quat2mat((w, rx, ry, rz))
        T = transforms3d.affines.compose((x, y, z), rot, (1,1,1))
        return T
    def rotation_from_matrix(self, matrix):
        R = np.array(matrix, dtype=np.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = np.linalg.eig(R33.T)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = np.linalg.eig(R)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (np.trace(R33) - 1.0) / 2.0
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
       from the base to each link of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = np.dot(tf.translation_matrix(joint.origin.xyz), tf.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'rxyz'))
            T = np.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.rotation_matrix(joint_values[q_index], np.asarray(joint.axis))
                T = np.dot(T, T_j)

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


def main(args = None):
    rclpy.init()
    ik = IK()
    rclpy.spin(ik)
    ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
