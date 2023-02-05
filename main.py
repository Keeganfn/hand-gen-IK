import time
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
#hand_path = current_path+"/resources/2v2_nosensors/2v2_nosensors_limited.urdf"
#hand_path = current_path+"/resources/2v2_2.1_1.2_1.1_8.10/hand/2v2_2.1_1.2_1.1_8.10.urdf"
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
d2 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.05174999999999999, 0], [
    0, 0.09974999999999999, 0]]}, "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}
# {'name': 'lengths': [0.05174999999999999, 0.09974999999999999, 0.09974999999999999, 0.05174999999999999, 0.06]}
print("JOINT BEFORE", p.getJointState(hand_id, 0)[0])
print("JOINT BEFORE", p.getJointState(hand_id, 1)[0])
ik = jacobian_IK.JacobianIK(hand_id, d["finger1"])
p.resetJointState(hand_id, 0, -.5)
p.resetJointState(hand_id, 1, .1)

start_time = time.time()
ik.finger_fk.update_angles_from_sim()
start = ik.finger_fk.calculate_forward_kinematics()
target = np.array([0.01, .1])
x = np.linspace(start[0], target[0], 150)
y = np.linspace(start[1], target[1], 150)
for i in range(150):
    target = np.array([x[i], y[i]])
# HEHEHEHEHEEH
#    print("TARGET", target)
    target_deb = np.array([x[i], y[i], .05])
    debug_id_new = p.addUserDebugPoints(
    [target_deb],
    [[255, 0, 0]],
    pointSize=10)
    j, angles, k = ik.calculate_ik(target, ee_location=None)
    p.setJointMotorControlArray(hand_id, [0, 1], p.POSITION_CONTROL, targetPositions=angles)
    #p.resetJointState(hand_id, 0, angles[0])
    #p.resetJointState(hand_id, 1, angles[1])
    p.stepSimulation()

print("--- %s seconds ---" % (time.time() - start_time))
# # ik.finger.calculate_forward_kinematics()
# target = np.array([0.01, .1])
# target_deb = np.array([0.01, .1, .05])
# debug_id_new = p.addUserDebugPoints(
#     [target_deb],
#     [[255, 0, 0]],
#     pointSize=10)
# start_time = time.time()
# #j, angles, k = ik.calculate_ik(target, ee_location=[-.01, .052, 0])
# j, angles, k = ik.calculate_ik(target, ee_location=None)
# print("--- %s seconds ---" % (time.time() - start_time))
# print(angles)
# print(j)
# print(k)
# p.resetJointState(hand_id, 0, angles[0])
# #p.resetJointState(hand_id, 0, -.16)
# p.resetJointState(hand_id, 1, angles[1])
# #p.resetJointState(hand_id, 1, .5)
# print("JOINT AFTER", p.getJointState(hand_id, 0)[0])
# print("JOINT AFTER", p.getJointState(hand_id, 1)[0])

# m1_c = [p.getLinkState(hand_id, 0)[0], p.getLinkState(hand_id, 0)[1]]
# m2_c = [p.getLinkState(hand_id, 1)[0], p.getLinkState(hand_id, 1)[1]]

# print(f"L0 ACTUAL: {m1_c}")
# print(f"L1 ACTUAL: {m2_c}")

# sim manager
manager = SimManagerDefault(num_episodes=2)

# Run the sim
manager.run()
manager.stall()
