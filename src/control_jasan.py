#!/usr/bin/env python3
import rospy
import pandas as pd
import rospkg
import numpy as np
import time as timing

from geometry_msgs.msg import TransformStamped
from control_utils.msg import ScalarStamped, VectorStamped
from control_utils.feedback_ctrl.state_feedback import StateFeedbackController
from control_utils.feedback_ctrl.integrator import ScalarIntegralController

from control_utils.general.utilities import quaternion_to_normal_vector
from control_utils.general.utilities import angles_from_normal_vector
from control_utils.general.utilities import init_MPEM_model, get_actuation_matrix_from_model

from control_utils.general.utilities import init_system, RT

from control_utils.general.helper import magnetic_dipole_moment_magnitude, volume_zylinder
from control_utils.general.torque_utils import matrix_gradfield_to_torqueforce

from scipy.signal import iirfilter 
from control_utils.general.utilities import LiveLFilter


# CURRENT_MAX = 4.0 # [A]
# CURRENT_MIN = -CURRENT_MAX 






class InvertedPendulum3DControlNode:
    def __init__(self):
        rospy.init_node('oct_3d_pend_node', anonymous=True)
        
        # parameters: 
        N_magnets = 10  # [1] # number of magnets
        self.l_mag = 38e-3 # [m] # magnet lever arm wrt to pivot point 
        V = volume_zylinder(h=4.0e-3, d_a = 8.0e-3, d_i =3.0e-3)*N_magnets
        self.m_tilde = magnetic_dipole_moment_magnitude(V, B_r = 1.4)
        
        self.distance_mag_vicon = 95e-3 # [m] # distance from magnet to vicon position (center of the bottom marker of the segment)

        self.T_stabilize  = 15.0 # [s] # during this time, all setpoints are zero (after release of pendulum)
        self.T_transition = 5.0  # [s] # time required to transition from 0 to setpoint amplitude
        
        # timing of offset calibs: 
        self.T_offset_from_start = 10.0  # [s] # just delays the all switches (to ensure that all topics are already publishing simult.) # set to 10 secs in paper fig
        self.T_convergence       = 13.0 # [s] # time for offset calibration to converge # stored value during stabilization phase, such that won't be updated during SP tracking phase
        
        assert self.T_convergence < self.T_stabilize, "T_convergence must be smaller than T_stabilize"

        self.read_ROS_parameters()
        self.assign_flags()
        self.init_publishers()
        self.init_payload_variables()
        self.init_controllers()
        self.init_angular_velocity_filters()
        self.init_offset_calib()

        if self.system_type == "JECB":
            self.active_drivers = [0, 1, 2,  3, 4, 5,  7, 8]
            self.model = init_MPEM_model(self.calibration_file_name)
            self.actuation_matrix = get_actuation_matrix_from_model(self.model, position = np.array([0.0, 0.0, 0.0]))  
            self.N_coils = 9
            self.CURRENT_MAX = 4.0 # [A]
            self.CURRENT_MIN = -self.CURRENT_MAX 
        elif self.system_type == "Navion":
            self.active_drivers = [0, 1, 2]
            self.model = init_MPEM_model(self.calibration_file_name)
            self.actuation_matrix = get_actuation_matrix_from_model(self.model, np.zeros(3)) 
            self.N_coils = 3
            self.CURRENT_MAX = 10.0 # [A]
            self.CURRENT_MIN = -self.CURRENT_MAX 
      
        self.current_SP = np.zeros(self.N_coils)

        self.current_msg, self.current_SP_pub, self.publish_currents = init_system(self.system_type, self.b_hardware_connected, coil_nrs = self.active_drivers)




    def read_ROS_parameters(self):
        self.vicon_callback_topic_actuator = rospy.get_param('~vicon_callback_topic_actuator', "/vicon/actuator_20cm/catheter_tip")
        self.vicon_callback_topic_pendulum = rospy.get_param('~vicon_callback_topic_pendulum', "/vicon/inv_pend/pendulum_base")
        self.system_type                   = rospy.get_param('~system_type', "JECB")

        self.angle_actuator_topic_rot_y    = rospy.get_param('~angle_actuator_topic_rot_y', "/alpha")
        self.angle_pendulum_topic_rot_y    = rospy.get_param('~angle_pendulum_topic_rot_y', "/phi")
        self.angle_actuator_topic_rot_x    = rospy.get_param('~angle_actuator_topic_rot_x', "/beta")
        self.angle_pendulum_topic_rot_x    = rospy.get_param('~angle_pendulum_topic_rot_x', "/theta")

        self.torque_topic_rot_y            = rospy.get_param('~torque_topic_rot_y', "/torque_alpha")
        self.torque_topic_rot_x            = rospy.get_param('~torque_topic_rot_x', "/torque_beta")

        self.angle_pendulum_ss_topic_rot_y = rospy.get_param('~angle_pendulum_ss_topic_rot_y', "/phi_ss")
        self.angle_pendulum_ss_topic_rot_x = rospy.get_param('~angle_pendulum_ss_topic_rot_x', "/theta_ss")

        self.calibration_file_name         = rospy.get_param('~calibration_file_name', "octomag_5point.yaml")
        self.b_use_angle_correction        = rospy.get_param('~b_use_angle_correction', False)
        self.b_use_integral_control        = rospy.get_param('~b_use_integral_control', False)

        self.b_hardware_connected          = rospy.get_param('~b_hardware_connected', False)
        self.b_southpole_up                = rospy.get_param('~b_southpole_up', False)
        self.b_update_actuation_matrix     = rospy.get_param('~b_update_actuation_matrix', False)
        self.b_flip_actuation_matrix       = rospy.get_param('~b_flip_actuation_matrix', False)

        self.process_time_actuator_topic = rospy.get_param('~process_time_actuator_topic', "/process_time_a_b")
        self.process_time_pendulum_topic = rospy.get_param('~process_time_pendulum_topic', "/process_time_p_t")

        self.Ts = rospy.get_param('~sampling_time', 0.005) # [s]
        print("Ts: ", self.Ts)

        self.condition_number_topic = rospy.get_param('~condition_number_topic', "/condition_number")

        if self.system_type == "JECB" and self.calibration_file_name == "octomag_5point.yaml":
            self.b_flip_actuation_matrix = True 

            
    def init_controllers(self):
        self.ctrl_a = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/data/alpha/state_feedback_denorm.csv") # _a = alpha phi
        self.ctrl_b = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/data/beta/state_feedback_denorm.csv") # _b = beta theta

        # self.ctrl_a = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/livedemo_MPI_gains/alpha/state_feedback_denorm.csv") # _a = alpha phi
        # self.ctrl_b = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/livedemo_MPI_gains/beta/state_feedback_denorm.csv") # _b = beta theta
        # if self.system_type == "Navion":
        #     self.ctrl_a = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/data_navion/alpha/state_feedback_denorm.csv") # _a = alpha phi
        #     self.ctrl_b = StateFeedbackController(package_name="oct_3d_pend_torque", relative_path_to_ctrl_prm="/data_navion/beta/state_feedback_denorm.csv") # _b = beta theta

        self.ctrl_int_a = ScalarIntegralController(-0.01, self.Ts)
        self.ctrl_int_b = ScalarIntegralController(-0.01, self.Ts)
        

        
        
    def init_payload_variables(self):
        
        # state variables: 
        self.alpha      = 0.0
        self.phi        = 0.0
        self.alphaD     = 0.0
        self.phiD       = 0.0
        self.prev_alpha_hat = 0.0
        self.prev_phi_hat   = 0.0

        self.beta       = 0.0
        self.theta      = 0.0
        self.betaD      = 0.0
        self.thetaD     = 0.0
        self.prev_beta_hat  = 0.0
        self.prev_theta_hat = 0.0

        self.alpha_hat = 0.0
        self.phi_hat = 0.0

        self.beta_hat = 0.0
        self.theta_hat = 0.0

        # processing times: 
        self.callback_time_a_b = 0.0
        self.callback_time_p_t = 0.0

        # torque vars: 
        self.normal_vector_actuator = np.array([0.0, 0.0, 1.0])
        self.torque_vec = np.zeros(3)
        self.vicon_position = np.zeros(3)
        self.magnet_position = np.zeros(3)

        self.torque_int_alpha = 0.0
        self.torque_int_beta  = 0.0
                

    def assign_flags(self):

        if self.b_use_angle_correction:
            self.get_state = self.get_corrected_state
        else:
            self.get_state = self.get_raw_state
                

    def init_publishers(self): 
        self.pub_a  = rospy.Publisher(self.angle_actuator_topic_rot_y, ScalarStamped, queue_size=10) 
        self.pub_p  = rospy.Publisher(self.angle_pendulum_topic_rot_y, ScalarStamped, queue_size=10)
        self.pub_aD = rospy.Publisher(self.angle_actuator_topic_rot_y + "D", ScalarStamped, queue_size=10)
        self.pub_pD = rospy.Publisher(self.angle_pendulum_topic_rot_y + "D", ScalarStamped, queue_size=10)

        self.msg_alpha  = ScalarStamped()
        self.msg_phi    = ScalarStamped()
        self.msg_alphaD = ScalarStamped()
        self.msg_phiD   = ScalarStamped()

        self.pub_b  = rospy.Publisher(self.angle_actuator_topic_rot_x, ScalarStamped, queue_size=10)
        self.pub_t  = rospy.Publisher(self.angle_pendulum_topic_rot_x, ScalarStamped, queue_size=10)
        self.pub_bD = rospy.Publisher(self.angle_actuator_topic_rot_x + "D", ScalarStamped, queue_size=10)
        self.pub_tD = rospy.Publisher(self.angle_pendulum_topic_rot_x + "D", ScalarStamped, queue_size=10)

        self.msg_beta  = ScalarStamped()
        self.msg_theta = ScalarStamped()
        self.msg_betaD = ScalarStamped()
        self.msg_thetaD= ScalarStamped()

        self.pub_a_hat = rospy.Publisher(self.angle_actuator_topic_rot_y + '_hat', ScalarStamped, queue_size=10)
        self.pub_p_hat = rospy.Publisher(self.angle_pendulum_topic_rot_y + '_hat', ScalarStamped, queue_size=10)
        self.pub_b_hat = rospy.Publisher(self.angle_actuator_topic_rot_x + '_hat', ScalarStamped, queue_size=10)
        self.pub_t_hat = rospy.Publisher(self.angle_pendulum_topic_rot_x + '_hat', ScalarStamped, queue_size=10)
        
        self.msg_alpha_hat = ScalarStamped()
        self.msg_phi_hat   = ScalarStamped()
        self.msg_beta_hat  = ScalarStamped()
        self.msg_theta_hat = ScalarStamped()
        
        self.pub_torque_a  = rospy.Publisher(self.torque_topic_rot_y, ScalarStamped, queue_size=10)
        self.msg_torque_a  = ScalarStamped()
 
        self.pub_u_b      = rospy.Publisher(self.torque_topic_rot_x, ScalarStamped, queue_size=10)
        self.msg_u_beta   = ScalarStamped()

        self.pub_p_ss     = rospy.Publisher(self.angle_pendulum_ss_topic_rot_y, ScalarStamped, queue_size=10)
        self.msg_phi_ss   = ScalarStamped()
 
        self.pub_t_ss     = rospy.Publisher(self.angle_pendulum_ss_topic_rot_x, ScalarStamped, queue_size=10)
        self.msg_theta_ss = ScalarStamped()

        self.pub_process_time_a_b  = rospy.Publisher(self.process_time_actuator_topic, ScalarStamped, queue_size=10) 
        self.msg_process_time_a_b  = ScalarStamped()

        self.pub_process_time_p_t  = rospy.Publisher(self.process_time_pendulum_topic, ScalarStamped, queue_size=10)
        self.msg_process_time_p_t  = ScalarStamped()

        self.condition_number_pub = rospy.Publisher(self.condition_number_topic,ScalarStamped,queue_size = 10)
        self.msg_condition_number = ScalarStamped()

        self.magnet_pos_pub = rospy.Publisher("/magnet_position",VectorStamped, queue_size = 10)
        self.msg_magnet_pos = VectorStamped()

            
    def init_angular_velocity_filters(self):

        # b, a = iirfilter(2, Wn=30.0, fs=1.0/self.Ts, btype="low", ftype="butter") 
        b = [1.0]
        a = [1.0] 

        self.livefilt_alpha_hat = LiveLFilter(b, a)
        self.livefilt_phi_hat = LiveLFilter(b, a)
        
        self.livefilt_beta_hat = LiveLFilter(b, a)
        self.livefilt_theta_hat = LiveLFilter(b, a)
    


    def init_offset_calib(self): 
        # learn ss-offset due to minor vicon calibration error by low pass filter with very low cutoff frequency to get steady state value
        b, a = iirfilter(2, Wn=0.05, fs=1.0/self.Ts, btype="low", ftype="butter") 
        self.livefilt_phi_ss = LiveLFilter(b, a)
        self.phi_ss   = 0.0     # dynamic steady state value 

        self.livefilt_theta_ss = LiveLFilter(b, a)
        self.theta_ss = 0.0     # dynamic steady state value
    
    
    def callback_alpha_beta(self, msg):
        self.callback_time_a_b = timing.time()
        # start measureing time: when main_callback triggered first time
        if not hasattr(self, 't_start'):
            self.t_start = timing.time()

        # read: 
        quat_rot =  msg.transform.rotation
        self.normal_vector_actuator = quaternion_to_normal_vector([quat_rot.x, quat_rot.y, quat_rot.z, quat_rot.w])

        self.alpha, self.beta = angles_from_normal_vector(self.normal_vector_actuator)

        self.vicon_position = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.magnet_position = self.vicon_position - self.distance_mag_vicon * self.normal_vector_actuator
        
        # use filtered angles to compute angular velocities:
        self.prev_alpha_hat = self.alpha_hat
        self.alpha_hat = self.livefilt_alpha_hat(self.alpha)
        self.alphaD = (self.alpha_hat - self.prev_alpha_hat) / self.Ts

        self.prev_beta_hat = self.beta_hat
        self.beta_hat = self.livefilt_beta_hat(self.beta)
        self.betaD = (self.beta_hat - self.prev_beta_hat) / self.Ts
            
        self.publish_actuator_states()


    def publish_actuator_states(self):
        # publish alpha: 
        self.msg_alpha.header.stamp = rospy.Time.now()
        self.msg_alpha.scalar = self.alpha 
        self.pub_a.publish(self.msg_alpha)

        # publish alphaD:
        self.msg_alphaD.header.stamp = rospy.Time.now()
        self.msg_alphaD.scalar = self.alphaD
        self.pub_aD.publish(self.msg_alphaD)

        # publish beta:
        self.msg_beta.header.stamp = rospy.Time.now()
        self.msg_beta.scalar = self.beta
        self.pub_b.publish(self.msg_beta)

        # publish betaD:
        self.msg_betaD.header.stamp = rospy.Time.now()
        self.msg_betaD.scalar = self.betaD
        self.pub_bD.publish(self.msg_betaD)

        # publish filtered velocities: # keep 0.0 as currently unused
        self.msg_alpha_hat.header.stamp = rospy.Time.now()
        self.msg_alpha_hat.scalar = self.alpha_hat
        self.pub_a_hat.publish(self.msg_alpha_hat)

        self.msg_beta_hat.header.stamp = rospy.Time.now()
        self.msg_beta_hat.scalar = self.beta_hat
        self.pub_b_hat.publish(self.msg_beta_hat)

        # publish process times:
        self.msg_process_time_a_b.header.stamp = rospy.Time.now()
        self.msg_process_time_a_b.scalar = timing.time() - self.callback_time_a_b
        self.pub_process_time_a_b.publish(self.msg_process_time_a_b)

        # publish magnet position: 
        self.msg_magnet_pos.header.stamp = rospy.Time.now()
        self.msg_magnet_pos.vector = self.magnet_position
        self.magnet_pos_pub.publish(self.msg_magnet_pos)



    def callback_phi_theta(self, msg):
        self.callback_time_p_t = timing.time()
        # start measureing time: when main_callback triggered first time
        if not hasattr(self, 't_start'):
            self.t_start = timing.time()

        # read: ScalarStampedPend
        quat_rot =  msg.transform.rotation
        n = quaternion_to_normal_vector([quat_rot.x, quat_rot.y, quat_rot.z, quat_rot.w])

        self.phi, self.theta = angles_from_normal_vector(n)
        
        self.prev_phi_hat = self.phi_hat
        self.phi_hat = self.livefilt_phi_hat(self.phi)
        self.phiD= (self.phi_hat - self.prev_phi_hat) / self.Ts

        self.prev_theta_hat = self.theta_hat
        self.theta_hat = self.livefilt_theta_hat(self.theta) 
        self.thetaD = (self.theta_hat - self.prev_theta_hat) / self.Ts
        
        if (timing.time() - self.t_start > self.T_offset_from_start) and (timing.time() - self.t_start < self.T_offset_from_start + self.T_convergence):
            # detect vicon calibration error:
            self.phi_ss = self.livefilt_phi_ss(self.phi) 
            self.theta_ss = self.livefilt_theta_ss(self.theta)


        self.state_feedback_step()
        self.publish_pendulum_states()


    def publish_pendulum_states(self):
        # publish phi: 
        self.msg_phi.header.stamp = rospy.Time.now()
        self.msg_phi.scalar = self.phi 
        self.pub_p.publish(self.msg_phi)

        # publish phiD:
        self.msg_phiD.header.stamp = rospy.Time.now()
        self.msg_phiD.scalar = self.phiD
        self.pub_pD.publish(self.msg_phiD)

        # publish phi_ss:
        self.msg_phi_ss.header.stamp = rospy.Time.now()
        self.msg_phi_ss.scalar = self.phi_ss
        self.pub_p_ss.publish(self.msg_phi_ss)

        # publish theta:
        self.msg_theta.header.stamp = rospy.Time.now()
        self.msg_theta.scalar = self.theta
        self.pub_t.publish(self.msg_theta)

        # publish thetaD:
        self.msg_thetaD.header.stamp = rospy.Time.now()
        self.msg_thetaD.scalar = self.thetaD
        self.pub_tD.publish(self.msg_thetaD)

        # publish theta_ss:
        self.msg_theta_ss.header.stamp = rospy.Time.now()
        self.msg_theta_ss.scalar = self.theta_ss
        self.pub_t_ss.publish(self.msg_theta_ss)

        # publish filtered velocities: keep 0.0 as currently unused
        self.msg_phi_hat.header.stamp = rospy.Time.now()
        self.msg_phi_hat.scalar = self.phi_hat
        self.pub_p_hat.publish(self.msg_phi_hat)

        self.msg_theta_hat.header.stamp = rospy.Time.now()
        self.msg_theta_hat.scalar = self.theta_hat
        self.pub_t_hat.publish(self.msg_theta_hat)

        # publish process times:
        self.msg_process_time_p_t.header.stamp = rospy.Time.now()
        self.msg_process_time_p_t.scalar = timing.time() - self.callback_time_p_t
        self.pub_process_time_p_t.publish(self.msg_process_time_p_t)

        
    # angle correction flag: 
    def get_corrected_state(self):
        x_a = np.array([[self.alpha - self.phi_ss], [self.phi - self.phi_ss], [self.alphaD], [self.phiD]])
        x_b = np.array([[self.beta - self.theta_ss], [self.theta - self.theta_ss], [self.betaD], [self.thetaD]])
        return x_a, x_b
    
    def get_raw_state(self):
        x_a = np.array([[self.alpha], [self.phi], [self.alphaD], [self.phiD]])
        x_b = np.array([[self.beta], [self.theta], [self.betaD], [self.thetaD]])
        return x_a, x_b
    
    def jacobian_torqueforce_to_torque(self, beta, alpha):
        # jacobian mapping torques and forces to control-torques
        force_torque_relation = np.array([[0,-self.l_mag*np.cos(beta)*np.cos(alpha) ,-np.sin(beta)*self.l_mag],[np.cos(beta)*np.cos(alpha)*self.l_mag, 0,-np.cos(beta)*np.sin(alpha)*self.l_mag],[np.sin(beta)*self.l_mag,self.l_mag*np.cos(beta)*np.sin(alpha) ,0]])
        
        return np.hstack((np.eye(3), force_torque_relation))
        

    def state_feedback_step(self):                         
        x_a, x_b = self.get_state()

        # state feedback control:
        torque_alpha = self.ctrl_a.run(x_SP = np.zeros_like(x_a), x = x_a) 
        torque_beta  = self.ctrl_b.run(x_SP = np.zeros_like(x_b), x = x_b) 

        magnetic_dipole_moment = self.m_tilde * self.normal_vector_actuator
        M = matrix_gradfield_to_torqueforce(magnetic_dipole_moment)

        

        
        if self.b_use_integral_control: 
            if (timing.time() - self.t_start > self.T_offset_from_start) and (timing.time() - self.t_start < self.T_offset_from_start + self.T_convergence):
                self.torque_int_alpha = self.ctrl_int_a.run(y_SP = 0.0, y = self.alpha - self.phi_ss)
                self.torque_int_beta  = self.ctrl_int_b.run(y_SP = 0.0, y = self.beta - self.theta_ss)
        
        self.torque_vec[0] = torque_beta + self.torque_int_beta   # rot x 
        self.torque_vec[1] = torque_alpha + self.torque_int_alpha  # rot y
        self.torque_vec[2] = 0.0 #  body fixed frame torque always zero

        if self.b_southpole_up:
            self.torque_vec = -1.0 * self.torque_vec

        if self.b_update_actuation_matrix:
            self.actuation_matrix = get_actuation_matrix_from_model(self.model, self.magnet_position) # init actuation matrix
        if self.b_flip_actuation_matrix:
            self.actuation_matrix = -self.actuation_matrix

        # minimum energy allocation: (1 step approach):
        J = self.jacobian_torqueforce_to_torque(beta = self.beta, alpha = self.alpha)
        J = J[:2, :]
        allocation_matrix = J @ M @ self.actuation_matrix
        computed_current = np.linalg.pinv(allocation_matrix) @ self.torque_vec[:2]

        # # 2-step allocation:  
        # J = self.jacobian_torqueforce_to_torque(beta = self.beta, alpha = self.alpha)
        # allocation_matrix = J @ M
        # inertial_torque = RT(self.alpha, self.beta) @ self.torque_vec
        # field_grad5 = np.linalg.pinv(allocation_matrix) @ inertial_torque
        # computed_current = np.linalg.pinv(self.actuation_matrix) @ field_grad5

       
        self.current_SP[self.active_drivers] = computed_current

        for ii in range(self.N_coils):
            self.current_SP[ii] = np.minimum(  np.maximum(self.current_SP[ii], self.CURRENT_MIN), self.CURRENT_MAX) # limit currents   
    

        # compute condition number:
        condition_number = np.linalg.cond(allocation_matrix)

        # special case because wiring was changed in the navion: 
        if self.calibration_file_name == "Navion_1_2_Calibration_old.yaml":
            self.current_SP[1] = - self.current_SP[1]  # Flip coil 2 current 

        self.publish_currents(self.current_SP, self.current_msg, self.current_SP_pub)
 
        self.publish_control_variables(torque_alpha, torque_beta, condition_number)



    def publish_control_variables(self, torque_alpha, torque_beta, condition_number):
        # publish control inputs: 
        self.msg_torque_a.header.stamp = rospy.Time.now()
        self.msg_torque_a.scalar = torque_alpha
        self.pub_torque_a.publish(self.msg_torque_a)

        self.msg_u_beta.header.stamp = rospy.Time.now()
        self.msg_u_beta.scalar = torque_beta
        self.pub_u_b.publish(self.msg_u_beta)

        self.msg_condition_number.header.stamp = rospy.Time.now()
        self.msg_condition_number.scalar = condition_number
        self.condition_number_pub.publish(self.msg_condition_number)

    

    def run(self):
        rospy.Subscriber("/vicon/actuator_20cm/catheter_tip", TransformStamped, self.callback_alpha_beta, queue_size=1)
        rospy.Subscriber("/vicon/inv_pend/pendulum_base", TransformStamped, self.callback_phi_theta, queue_size=1)
                    
        rospy.spin()


if __name__ == '__main__':

    node = InvertedPendulum3DControlNode()
    node.run()