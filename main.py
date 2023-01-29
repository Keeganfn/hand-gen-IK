import pybullet as p
import pybullet_data
import numpy as np
import pathlib
import forward_kinematics
import jacobian_IK
from mojograsp.simcore.sim_manager import SimManagerDefault
from mojograsp.simcore.state import StateDefault
from mojograsp.simcore.reward import RewardDefault
from mojograsp.simcore.record_data import RecordDataJSON
from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase


# resource paths
current_path = str(pathlib.Path().resolve())
hand_path = current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf"
cube_path = current_path + \
    "/resources/2v2_Demo/object/2v2_Demo_cuboid_small.urdf"
data_path = current_path+"/data/"
points_path = current_path+"/resources/points.csv"

# start pybullet
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                             cameraTargetPosition=[0, 0.1, 0.5])

# load objects into pybullet
plane_id = p.loadURDF("plane.urdf")
hand_id = p.loadURDF(hand_path, useFixedBase=True,
                     basePosition=[0.0, 0.0, 0.05])
# cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05])

# Create TwoFingerGripper Object and set the initial joint positions
hand = TwoFingerGripper(hand_id, path=hand_path)

# Create ObjectBase for the cube object
# cube = ObjectBase(cube_id, path=cube_path)

# change visual of gripper
p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])

d = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]},
     "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}


#p.resetJointState(hand_id, 0, -0.2)
#p.resetJointState(hand_id, 1, -.4)
ik = jacobian_IK.JacobianIK(hand_id, d["finger1"], d["finger2"])
# ik.calculate_jacobian_f1()
target = np.array([0.01, .1])
target2 = np.array([0.02, .1])
target3 = np.array([0.02, .11])
target_deb = np.array([0.01, .1, .05])
debug_id_new = p.addUserDebugPoints(
    [target_deb],
    [[255, 0, 0]],
    pointSize=10)
j, angles, k = ik.calculate_ik(target)
print(angles)
p.resetJointState(hand_id, 0, angles[0])
p.resetJointState(hand_id, 1, angles[1])
j, angles, k = ik.calculate_ik(target2)
print(angles)
p.resetJointState(hand_id, 0, angles[0])
p.resetJointState(hand_id, 1, angles[1])
j, angles, k = ik.calculate_ik(target3)
print(angles)
p.resetJointState(hand_id, 0, angles[0])
p.resetJointState(hand_id, 1, angles[1])


#FK = forward_kinematics.ForwardKinematicsSIM(hand_id, d["finger1"])
#FK.set_joint_angles([-.2, -.4])
# FK.calculate_forward_kinematics()
# m1 = FK.calculate_FK(angles_f1=[.1, .1])
# m1 = FK.calculate_FK(angles_f1=[-.1, -.1])
# #FK.calculate_FK(angles_f1=[-.1, -.1])
#p.resetJointState(hand_id, 0, 0.1)
#p.resetJointState(hand_id, 1, 0.1)
m1_c = [p.getLinkState(hand_id, 0)[0], p.getLinkState(hand_id, 0)[1]]
m2_c = [p.getLinkState(hand_id, 1)[0], p.getLinkState(hand_id, 1)[1]]

print(f"L0 ACTUAL: {m1_c}")
print(f"L1 ACTUAL: {m2_c}")

# sim manager
manager = SimManagerDefault(num_episodes=2)

# Run the sim
manager.run()
manager.stall()
