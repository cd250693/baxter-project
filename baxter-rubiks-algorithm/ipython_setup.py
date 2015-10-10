import rospy
import baxter_interface
rospy.init_node('ipython')
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')
gripper_right = baxter_interface.Gripper('right')
# gripper_right.calibrate()
gripper_left = baxter_interface.Gripper('left')
# gripper_left.calibrate()

# angles for the position to pick up the cube
cube_pickup = {
    'right_e0': -0.7669903930664063,
    'right_e1': 0.8007379703613282,
    'right_s0': 1.2172137537963867,
    'right_s1': -0.4421699616027832,
    'right_w0': 0.6803204786499024,
    'right_w1': 1.432738054248047,
    'right_w2': -0.120800986907959}

# joint angles for position above the cube
above_cube_pickup = {
    'right_e0': -0.33402431618041994,
    'right_e1': 0.832568071673584,
    'right_s0': 1.0538448000732423,
    'right_s1': -0.6876068873840332,
    'right_w0': 0.26461168560791015,
    'right_w1': 1.4638011651672365,
    'right_w2': -0.016490293450927736}

# joint angles for right limb central and cube positions
right_centred = {
    'right_e0': -0.6369855214416504,
    'right_e1': 2.139136206262207,
    'right_s0': 0.8716845817199708,
    'right_s1': -0.9875001310729982,
    'right_w0': 1.3077186201782227,
    'right_w1': 2.089665325909424,
    'right_w2': -1.1757962725708009}

right_cube = {
    'right_e0': -0.11926700612182618,
    'right_e1': 2.259937193170166,
    'right_s0': 0.607839886505127,
    'right_s1': -1.145883647241211,
    'right_w0': 1.36485940446167,
    'right_w1': 1.746053629815674,
    'right_w2': -1.1727283109985351}

# joint angles for left limb central and cube positions
left_centred = {
    'left_e0': -0.33785926814575196,
    'left_e1': 2.086213869140625,
    'left_s0': -0.10584467424316407,
    'left_s1': -1.0837574254028322,
    'left_w0': -1.125174906628418,
    'left_w1': 1.6751070184570314,
    'left_w2': -0.645422415765381}

left_cube = {
    'left_e0': -0.3857961677124024,
    'left_e1': 2.163679898840332,
    'left_s0': -0.21667478604125978,
    'left_s1': -1.136679762524414,
    'left_w0': -1.2494273503051758,
    'left_w1': 1.5753982673583986,
    'left_w2': -0.5955680402160645}

# joint angles for left limb face manipulation position
left_face_rotation = {
    'left_e0': -0.33172334500122075,
    'left_e1': 2.148340090979004,
    'left_s0': -0.2546408104980469,
    'left_s1': -1.1201894690734864,
    'left_w0': -1.2275681241027834,
    'left_w1': 1.601092445526123,
    'left_w2': -0.645422415765381}

# angles for left arm rotating faces
joint_left_face_rotation = {}
# angles for left arm quater cube turn flat
joint_left_cube_flat_rotation = {}
# angles for left arm quater turn up
joint_left_cube_up_rotation = {}
# angles for left are quater turn down
joint_left_cube_down_rotation = {}
# left arm position for rotating the cube up/down
joint_left_arm_down = {
    'left_e0': -1.0760875214721681,
    'left_e1': 1.4580487372192383,
    'left_s0': -0.3873301484985352,
    'left_s1': 0.5556845397766114,
    'left_w0': -1.0177962515991212,
    'left_w1': 2.0359759983947754,
    'left_w2': -0.28570392141723633}
