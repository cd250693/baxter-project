import rospy
import baxter_interface

# intialize the rospy node and create objects for the limbs and arms
rospy.init_node('ipython')
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')
gripper_right = baxter_interface.Gripper('right')
gripper_left = baxter_interface.Gripper('left')

# check if baxter needs to be enabled
robotstate = baxter_interface.RobotEnable()
if robotstate.state().enabled is False:
    robotstate.enable()
# calibrate the grippers
gripper_right.calibrate()
gripper_left.calibrate()

######
# Joint angles for both arms
# right: right limb joint angles
# left: left limb joint angles
# central: position near the cube
# cube: position where the cube can be grabbed/manipulated
######

# joint angles for right limb central and cube positions
right_central = {
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

# joint angles for right limb pickup and place of cube
right_pickup_central = {
    'right_e0': -0.8310340908874513,
    'right_e1': 0.7685243738525391,
    'right_s0': 1.2340875424438478,
    'right_s1': -0.5276893904296875,
    'right_w0': 0.6887573729736328,
    'right_w1': 1.5393497188842775,
    'right_w2': -0.09165535197143555}

right_pickup_cube = {
    'right_e0': -0.7669903930664063,
    'right_e1': 0.8007379703613282,
    'right_s0': 1.2172137537963867,
    'right_s1': -0.4421699616027832,
    'right_w0': 0.6803204786499024,
    'right_w1': 1.432738054248047,
    'right_w2': -0.120800986907959}

right_flat_central = {
    'right_e0': 0.7110000943725586,
    'right_e1': 1.0618981992004395,
    'right_s0': 0.32213596508789066,
    'right_s1': -0.4640291878051758,
    'right_w0': 0.6450389205688477,
    'right_w1': 2.093883773071289,
    'right_w2': -0.5273058952331543}

right_flat_cube = {
    'right_e0': 0.4885728803833008,
    'right_e1': 1.344150663848877,
    'right_s0': 0.4506068559265137,
    'right_s1': -0.6580777572509766,
    'right_w0': 0.7646894218872071,
    'right_w1': 2.0639711477416993,
    'right_w2': -0.7006457240661621}

# joint angles for left limb central and cube positions
left_central = {
    'left_e0': -0.33785926814575196,
    'left_e1': 2.086213869140625,
    'left_s0': -0.10584467424316407,
    'left_s1': -1.0837574254028322,
    'left_w0': -1.125174906628418,
    'left_w1': 1.6751070184570314,
    'left_w2': -0.645422415765381}

left_cube = {
    'left_e0': -0.33172334500122075,
    'left_e1': 2.148340090979004,
    'left_s0': -0.2546408104980469,
    'left_s1': -1.1201894690734864,
    'left_w0': -1.2275681241027834,
    'left_w1': 1.601092445526123,
    'left_w2': -0.645422415765381}

# joint angles for left limb central and cube flat rotation
left_flat_central = {
    'left_e0': -0.6005534777709961,
    'left_e1': 1.2371555040161133,
    'left_s0': -0.3144660611572266,
    'left_s1': -0.5832961939270019,
    'left_w0': -0.7528010707946777,
    'left_w1': 2.095417753857422,
    'left_w2': -1.0404224681945802}

left_flat_cube = {
    'left_e0': -0.2523398393188477,
    'left_e1': 1.585752637664795,
    'left_s0': -0.5974855161987305,
    'left_s1': -0.8295001101013184,
    'left_w0': -0.9422476978820802,
    'left_w1': 2.014116772192383,
    'left_w2': -0.7911505904479981}

# joint angles for left limb central and cube vertical rotation
left_vertical_central = {
    'left_e0': -0.6902913537597657,
    'left_e1': 1.7330147931335451,
    'left_s0': -0.009587379913330078,
    'left_s1': 0.10584467424316407,
    'left_w0': -1.0760875214721681,
    'left_w1': 2.0946507634643554,
    'left_w2': -0.40573791793212893}

left_vertical_cube = {
    'left_e0': -1.1320778201660158,
    'left_e1': 1.4258351407104493,
    'left_s0': -0.3773592733886719,
    'left_s1': 0.5948010498229981,
    'left_w0': -0.9625729432983399,
    'left_w1': 2.0455633783081058,
    'left_w2': -0.3497476192382813}
print('ready')
