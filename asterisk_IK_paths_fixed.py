import time
import pybullet as p
import math
import pybullet_data
import numpy as np
import matrix_helper as mh
import pathlib
import forward_kinematics
import jacobian_IK
import asterisk_controller
from mojograsp.simcore.sim_manager import SimManagerDefault
from mojograsp.simcore.state import StateDefault
from mojograsp.simcore.reward import RewardDefault
from mojograsp.simcore.record_data import RecordDataJSON
from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase


# # resource paths
# current_path = str(pathlib.Path().resolve())
# hand_path = current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf"
# #hand_path = current_path+"/resources/2v2_nosensors/2v2_nosensors_limited.urdf"
# #hand_path = current_path+"/resources/2v2_2.1_1.2_1.1_8.10/hand/2v2_2.1_1.2_1.1_8.10.urdf"
# cube_path = current_path + \
#     "/resources/2v2_Demo/object/2v2_Demo_cuboid_small.urdf"
# data_path = current_path+"/data/"
# points_path = current_path+"/resources/points.csv"

# # start pybullet
# physics_client = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)
# p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
#                              cameraTargetPosition=[0, 0.1, 0.5])

# # load objects into pybullet
# plane_id = p.loadURDF("plane.urdf")
# hand_id = p.loadURDF(hand_path, useFixedBase=True,
#                      basePosition=[0.0, 0.0, 0.05])

# # Create TwoFingerGripper Object and set the initial joint positions
# hand = TwoFingerGripper(hand_id, path=hand_path)
# d = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]},
#      "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}
# # d2 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.05174999999999999, 0], [
# #    0, 0.09974999999999999, 0]]}, "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, 0.09974999999999999, 0], [
# #        0, 0.05174999999999999, 0]]}}
# # {'name': 'lengths': [0.05174999999999999, 0.09974999999999999, 0.09974999999999999, 0.05174999999999999, 0.06]}

# # Get ik and fk setup and reset joints to proper angles
# ik_f1 = jacobian_IK.JacobianIK(hand_id, d["finger1"])
# ik_f2 = jacobian_IK.JacobianIK(hand_id, d["finger2"])
# p.resetJointState(hand_id, 0, -.5)
# p.resetJointState(hand_id, 2, .5)
# ik_f1.finger_fk.update_angles_from_sim()
# ik_f2.finger_fk.update_angles_from_sim()

# # change visual of gripper
# p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
# p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
# p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
# p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
# p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])

# # Create ObjectBase for the cube object
# cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05])
# cube = ObjectBase(cube_id, path=cube_path)


# start_f1 = ik_f1.finger_fk.calculate_forward_kinematics()
# target_f1 = np.array([0.01, .1])
# x_f1 = np.linspace(start_f1[0], target_f1[0], 150)
# y_f1 = np.linspace(start_f1[1], target_f1[1], 150)
# start_f2 = ik_f2.finger_fk.calculate_forward_kinematics()
# target_f2 = np.array([-0.01, .1])
# x_f2 = np.linspace(start_f2[0], target_f2[0], 150)
# y_f2 = np.linspace(start_f2[1], target_f2[1], 150)
# for i in range(150):
#     target_f1 = np.array([x_f1[i], y_f1[i]])
#     target_f2 = np.array([x_f2[i], y_f2[i]])
# #     target_deb = np.array([x_f1[i], y_f1[i], .05])
# # #    debug_id_new = p.addUserDebugPoints(
# #         [target_deb],
# #         [[255, 0, 0]],
# #         pointSize=10)
# #     target_deb = np.array([x_f2[i], y_f2[i], .05])
# #     debug_id_new = p.addUserDebugPoints(
# #         [target_deb],
# #         [[255, 0, 0]],
# #         pointSize=10)

#     j, angles_f1, k = ik_f1.calculate_ik(target_f1, ee_location=None)
#     j, angles_f2, k = ik_f2.calculate_ik(target_f2, ee_location=None)
#     p.setJointMotorControlArray(hand_id, [0, 1], p.POSITION_CONTROL, targetPositions=angles_f1)
#     p.setJointMotorControlArray(hand_id, [2, 3], p.POSITION_CONTROL, targetPositions=angles_f2)
#     p.stepSimulation()
# #    time.sleep(.01)


# def step_towards_goal(start_vec, end_vec, distance):
#     temp_x = end_vec[0] - start_vec[0]
#     temp_y = end_vec[1] - start_vec[1]
#     magnitude = math.sqrt((temp_x**2 + temp_y**2))
#     temp_x /= magnitude
#     temp_y /= magnitude
#     temp_x = start_vec[0] + distance*temp_x
#     temp_y = start_vec[1] + distance*temp_y
#     return [temp_x, temp_y]


# start_f1 = ik_f1.finger_fk.calculate_forward_kinematics()
# target_f1 = np.array([0.01, .03])
# x_f1 = np.linspace(start_f1[0], target_f1[0], 150)
# y_f1 = np.linspace(start_f1[1], target_f1[1], 150)
# start_f2 = ik_f2.finger_fk.calculate_forward_kinematics()
# target_f2 = np.array([-0.01, .03])
# x_f2 = np.linspace(start_f2[0], target_f2[0], 150)
# y_f2 = np.linspace(start_f2[1], target_f2[1], 150)

# for i in range(150):
#     start_f1 = ik_f1.finger_fk.calculate_forward_kinematics()
#     start_f2 = ik_f2.finger_fk.calculate_forward_kinematics()
#     tt1 = np.array(step_towards_goal(start_f1, target_f1, .005))
#     tt2 = np.array(step_towards_goal(start_f2, target_f2, .005))
#     #target_f1 = np.array([x_f1[i], y_f1[i]])
#     #target_f2 = np.array([x_f2[i], y_f2[i]])
#     # target_deb = np.array([x_f1[i], y_f1[i], .05])
#     # debug_id_new = p.addUserDebugPoints(
#     #     [target_deb],
#     #     [[255, 0, 0]],
#     #     pointSize=10)
#     # target_deb = np.array([x_f2[i], y_f2[i], .05])
#     # debug_id_new = p.addUserDebugPoints(
#     #     [target_deb],
#     #     [[255, 0, 0]],
#     #     pointSize=10)

#     contact_point_info1 = p.getContactPoints(cube_id, hand_id, linkIndexB=1)
#     contact_point_info2 = p.getContactPoints(cube_id, hand_id, linkIndexB=3)
#     t1 = mh.create_translation_matrix(contact_point_info1[-1][6])
#     t2 = mh.create_translation_matrix(contact_point_info2[-1][6])
#     f1 = mh.create_transformation_matrix(p.getLinkState(hand_id, 1)[0], p.getLinkState(hand_id, 1)[1])
#     f2 = mh.create_transformation_matrix(p.getLinkState(hand_id, 3)[0], p.getLinkState(hand_id, 3)[1])
#     #f1 = mh.create_translation_matrix(p.getLinkState(hand_id, 1)[0])
#     #f2 = mh.create_translation_matrix(p.getLinkState(hand_id, 3)[0])
#     cp1 = np.linalg.inv(f1) @ t1 @ [0, 0, 1]
#     cp2 = np.linalg.inv(f2) @ t2 @ [0, 0, 1]

#     j, angles_f1, k = ik_f1.calculate_ik(tt1, ee_location=cp1)
#     j, angles_f2, k = ik_f2.calculate_ik(tt2, ee_location=cp2)
#     p.setJointMotorControlArray(hand_id, [0, 1], p.POSITION_CONTROL, targetPositions=angles_f1)
#     p.setJointMotorControlArray(hand_id, [2, 3], p.POSITION_CONTROL, targetPositions=angles_f2)
#     p.stepSimulation()


# # sim manager
# manager = SimManagerDefault(num_episodes=2)

# # Run the sim
# manager.run()
# manager.stall()


def setup_hand(hand_path, hand_info, start_angle=.5, x=0):
    # load urdf
    hand_id = p.loadURDF(
        hand_path, useFixedBase=True, basePosition=[x, 0.0, 0.05],
        flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_SELF_COLLISION)
    # setup ik
    ik_f1 = jacobian_IK.JacobianIK(hand_id, hand_info["finger1"])
    ik_f2 = jacobian_IK.JacobianIK(hand_id, hand_info["finger2"])
    distal_f1_index = ik_f1.finger_fk.link_ids[-1]
    distal_f2_index = ik_f2.finger_fk.link_ids[-1]
    proximal_f1_index = ik_f1.finger_fk.link_ids[0]
    proximal_f2_index = ik_f2.finger_fk.link_ids[0]
    # get joints in starting positions
    p.resetJointState(hand_id, proximal_f1_index, -start_angle)
    p.resetJointState(hand_id, proximal_f2_index, start_angle)
    p.resetJointState(hand_id, distal_f1_index, .2)
    p.resetJointState(hand_id, distal_f2_index, -.2)
    ik_f1.finger_fk.update_angles_from_sim()
    ik_f2.finger_fk.update_angles_from_sim()
    # change color
    p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeDynamics(hand_id, 0, jointLowerLimit=-1.57, jointUpperLimit=1.74)
    p.changeDynamics(hand_id, 1, jointLowerLimit=-1.57, jointUpperLimit=1.74)
    p.changeDynamics(hand_id, 2, jointLowerLimit=1.57, jointUpperLimit=-1.74)
    p.changeDynamics(hand_id, 3, jointLowerLimit=1.57, jointUpperLimit=-1.74)
    return hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index


def get_paths():
    # resource paths
    current_path = str(pathlib.Path().resolve())
    #hand_path = current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf"
    hand_path = current_path+"/resources/2v2_2.1_1.2_1.1_8.10/hand/2v2_2.1_1.2_1.1_8.10.urdf"
    cube_path = current_path + \
        "/resources/2v2_Demo/object/2v2_Demo_cuboid_small.urdf"
    data_path = current_path+"/data/"
    return current_path, hand_path, cube_path, data_path


def setup_sim():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                 cameraTargetPosition=[0, 0.1, 0.5])
    # load objects into pybullet
    plane_id = p.loadURDF("plane.urdf", flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
    p.changeDynamics(plane_id, -1, lateralFriction=.5)


def test():
    # start pybullet
    setup_sim()

    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    # get paths for data and sim objects
    current_path, hand_path, cube_path, data_path = get_paths()
    # load hand
    test_hand = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]},
                 "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}
    test_hand2 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.05174999999999999, 0], [0, 0.09974999999999999, 0]]}, "finger2": {
        "name": "finger1", "num_links": 2, "link_lengths": [[0, 0.09974999999999999, 0], [0, 0.05174999999999999, 0]]}}

    hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand2, .4, x=0)
    # load cube
    cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05])
    controller = asterisk_controller.AsteriskController(
        hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)

    hand_id2, ik_f12, ik_f22, distal_f1_index2, distal_f2_index2 = setup_hand(hand_path, test_hand2, .4, x=1)
    # load cube
    cube_id2 = p.loadURDF(cube_path, basePosition=[1.0, 0.1067, .05])
    controller2 = asterisk_controller.AsteriskController(
        hand_id2, cube_id2, ik_f12, ik_f22, distal_f1_index2, distal_f2_index2)

    controller.close_hand()
    controller.move_hand_multi(directions[0])
    controller2.close_hand()
    controller2.move_hand_multi(directions[1], main=True, offset=1)


if __name__ == "__main__":
    # start pybullet
    # test()
    setup_sim()

    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    start = time.time()
    for i in range(len(directions)):
        # get paths for data and sim objects
        current_path, hand_path, cube_path, data_path = get_paths()
        # load hand
        test_hand = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]},
                     "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}
        test_hand2 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.05174999999999999, 0], [0, 0.09974999999999999, 0]]}, "finger2": {
            "name": "finger1", "num_links": 2, "link_lengths": [[0, 0.09974999999999999, 0], [0, 0.05174999999999999, 0]]}}

        hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand2, .5)
        # load cube
        cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        controller = asterisk_controller.AsteriskController(
            hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)
        controller.close_hand()
        controller.move_hand(directions[i])
        p.resetSimulation()
    print("END TIME", time.time() - start)


# reset_sim

# load(hand)
# load(cube)
# close_hand(hand)
# move_hand
# check for completion
