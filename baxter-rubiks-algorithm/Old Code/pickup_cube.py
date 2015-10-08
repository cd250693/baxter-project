# coding: utf-8
import rospy
import baxter_interface
rospy.init_node('pickup_cube')
limb = baxter_interface.Limb('right')
gripper = baxter_interface.Gripper('right')
gripper.close()
angles = limb.joint_angles()
angles_pickup = {'right_e0': -0.6216457135803223,
 'right_e1': 0.9157865293212891,
 'right_s0': 1.136296267327881,
 'right_s1': -0.5503156070251465,
 'right_w0': 0.4621117118225098,
 'right_w1': 1.3011992018371583,
 'right_w2': -0.19174759826660157}
rospy.sleep(0.5)
angles_above_pickup = {'right_e0': -0.2308641083129883,
 'right_e1': 0.3386262585388184,
 'right_s0': 0.9073496349975586,
 'right_s1': -0.7481991284362793,
 'right_w0': 0.15263108822021484,
 'right_w1': 1.842310924145508,
 'right_w2': -0.045635928387451175}
rospy.sleep(0.5)
centered = {'right_e0': -0.11696603494262696,
 'right_e1': 2.2660731163146974,
 'right_s0': 0.6082233817016602,
 'right_s1': -1.143966171258545,
 'right_w0': 1.3640924140686037,
 'right_w1': 1.7445196490295412,
 'right_w2': -1.1727283109985351}
