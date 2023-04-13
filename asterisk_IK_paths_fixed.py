from copy import deepcopy
import sys
import os
import time
import pybullet as p
import json
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


def setup_hand(hand_path, hand_info, start_angle=.5, x=0):
    # load urdf
    hand_id = p.loadURDF(
        hand_path, useFixedBase=True, basePosition=[x, 0.0, 0.05],
        flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    # setup ik
    print(hand_info["finger1"])
    print(hand_info["finger2"])
    from copy import deepcopy
    ik_f1 = jacobian_IK.JacobianIK(hand_id, deepcopy(hand_info["finger1"]))
    ik_f2 = jacobian_IK.JacobianIK(hand_id, deepcopy(hand_info["finger2"]))
    distal_f1_index = ik_f1.finger_fk.link_ids[-1]
    distal_f2_index = ik_f2.finger_fk.link_ids[-1]
    proximal_f1_index = ik_f1.finger_fk.link_ids[0]
    proximal_f2_index = ik_f2.finger_fk.link_ids[0]
    # get joints in starting positions
    p.resetJointState(hand_id, proximal_f1_index, -start_angle)
    p.resetJointState(hand_id, proximal_f2_index, start_angle)
    p.resetJointState(hand_id, distal_f1_index, .4)
    p.resetJointState(hand_id, distal_f2_index, -.4)
    ik_f1.finger_fk.update_angles_from_sim()
    ik_f2.finger_fk.update_angles_from_sim()
    # change color

    p.changeDynamics(hand_id, distal_f1_index, lateralFriction=1, rollingFriction=.04,
                     mass=5)
    p.changeDynamics(hand_id, distal_f2_index, lateralFriction=1, rollingFriction=0.04,
                     mass=5)
    p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
    if len(ik_f1.finger_fk.link_ids) == 2:
        p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeDynamics(hand_id, 0, jointLowerLimit=-1.57, jointUpperLimit=1.57)
        p.changeDynamics(hand_id, 1, jointLowerLimit=-1.57, jointUpperLimit=2.09)
    if len(ik_f1.finger_fk.link_ids) == 3:
        p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
        p.changeDynamics(hand_id, 0, jointLowerLimit=-1.57, jointUpperLimit=1.57)
        p.changeDynamics(hand_id, 1, jointLowerLimit=-1.57, jointUpperLimit=2.09)
        p.changeDynamics(hand_id, 2, jointLowerLimit=-1.57, jointUpperLimit=2.09)
    if len(ik_f2.finger_fk.link_ids) == 2:
        p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeDynamics(hand_id, 2, jointLowerLimit=-1.57, jointUpperLimit=1.57)
        p.changeDynamics(hand_id, 3, jointLowerLimit=-2.09, jointUpperLimit=1.57)
    if len(ik_f2.finger_fk.link_ids) == 3:
        p.changeVisualShape(hand_id, 3, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 4, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeVisualShape(hand_id, 5, rgbaColor=[1, 0.5, 0, 1])
        p.changeDynamics(hand_id, 3, jointLowerLimit=-1.57, jointUpperLimit=1.57)
        p.changeDynamics(hand_id, 4, jointLowerLimit=-2.09, jointUpperLimit=1.57)
        p.changeDynamics(hand_id, 5, jointLowerLimit=-2.09, jointUpperLimit=1.57)
    if len(ik_f2.finger_fk.link_ids) == 3 and len(ik_f1.finger_fk.link_ids) == 2:
        p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeVisualShape(hand_id, 4, rgbaColor=[1, 0.5, 0, 1])
        p.changeDynamics(hand_id, 2, jointLowerLimit=-1.57, jointUpperLimit=1.57, jointLimitForce=.05)
        p.changeDynamics(hand_id, 3, jointLowerLimit=-2.09, jointUpperLimit=1.57, jointLimitForce=.05)
        p.changeDynamics(hand_id, 4, jointLowerLimit=-2.09, jointUpperLimit=1.57, jointLimitForce=.05)

    return hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index


def get_paths():
    # resource paths
    current_path = str(pathlib.Path().resolve())
    hand_path = current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf"
    hand_path2 = current_path+"/resources/2v2_2.1_1.2_1.1_8.10/hand/2v2_2.1_1.2_1.1_8.10.urdf"
    hand_path3 = current_path+"/resources/3v3_Demo/hand/3v3_Demo.urdf"
    hand_path4 = current_path+"/resources/2v3_Demo/hand/2v3_Demo.urdf"
    hand_path5 = current_path+"/resources/2v2_Demo_short/hand/2v2_Demo.urdf"
    hand_path6 = current_path+"/resources/2v2_Demo_long/hand/2v2_Demo.urdf"
    hand_path7 = current_path+"/resources/2v2_Demo_elong/hand/2v2_Demo.urdf"
    # hand_path8 = current_path+"/generated_hands/2v2_1.1_1.1_1.1_1.1/hand/2v2_1.1_1.1_1.1_1.1.urdf"
    hand_path8 = current_path+"/generated_2v2_1.1/2v2_50.50_50.50_1.1_63/hand/2v2_50.50_50.50_1.1_63.urdf"
    # hand_path8 = current_path+"/generated_chosen/3v3_50.0.25.0.25.0_40.0.35.0.25.0_1.1_540/hand/3v3_50.0.25.0.25.0_40.0.35.0.25.0_1.1_540.urdf"
    hand_path9 = current_path+"/generated_2v2_1.1/2v2_75.25_70.30_1.1_53/hand/2v2_75.25_70.30_1.1_53.urdf"
    hand_path10 = current_path + "/generated_hands/2v3_60.40_40.30.30_1.0.9_63/hand/2v3_60.40_40.30.30_1.0.9_63.urdf"
    hand_path11 = current_path + "/generated_hands/3v3_50.25.25_25.45.30_1.1_63/hand/3v3_50.25.25_25.45.30_1.1_63.urdf"
    cube_path = current_path + \
        "/resources/2v2_Demo/object/2v2_Demo_cuboid_small.urdf"
    data_path = current_path+"/data/"
    return current_path, hand_path11, cube_path, data_path


def setup_sim():
    physics_client = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    # p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
    #                             cameraTargetPosition=[0, 0.1, 0.5])
    # load objects into pybullet
    plane_id = p.loadURDF("plane.urdf", flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
    p.changeDynamics(plane_id, -1, lateralFriction=.3)


def get_hand_paths():
    current_path = str(pathlib.Path().resolve())
    print(sys.argv[1], sys.argv[2])

    paths = []
    names = []
    for file in os.listdir(current_path + "/generated_hands"):
        # print(str(file))
        # print(len(names))
        temp_str = current_path + "/generated_hands/" + str(file) + "/hand/" + str(file) + ".urdf"
        paths.append(temp_str)
        names.append(str(file))

    print(paths.pop(-1))
    print(len(names))
    print(names.pop(-1))
    print(len(names))

    with open(current_path + "/generated_hands/hand_descriptions_rerun_all.json", "r+") as fp:
        hand_descs = json.load(fp)

    print(hand_descs[0])
    print(len(hand_descs))
    total_num = len(hand_descs)
    node_num = int(sys.argv[1]) - 1
    total_nodes = int(sys.argv[2])
    num_hands_run = math.ceil(total_num / total_nodes)
    start = node_num * num_hands_run
    end = start + num_hands_run
    hand_descs = hand_descs[start:min(end+1, len(hand_descs)+1)]
    print(hand_descs)
    print(len(hand_descs))
    print(start, end)
    time.sleep(10)

    while len(hand_descs) <= len(names):
        hand_descs.append({"name": "dub"})

    return paths, hand_descs, names


def run_batch():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                 cameraTargetPosition=[0, 0.1, 0.5])

    hand_paths, hand_descs, names = get_hand_paths()
    print(len(names), len(hand_descs))
    old_hand = ""
    for i in range(len(hand_paths)):
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        start_positions = ["MM", "TT", "BB", "MT", "TM", "MB", "BM", "TB", "BT"]
        current_path, _, cube_path, data_path = get_paths()
        t = False
        for k in range(len(hand_descs)):
            if t == True:
                break
            for b in range(len(names)):
                if hand_descs[k]["name"] == names[b]:
                    test_hand = deepcopy(hand_descs[k]["sim"])
                    print(hand_descs.pop(k))
                    trial_name = names.pop(b)
                    hand_path = deepcopy(hand_paths[b])
                    print(hand_paths.pop(b))
                    t = True
                    break
        if old_hand == trial_name:
            return
        else:
            old_hand = trial_name
        for j in range(len(directions)):
            for start in start_positions:
                # get paths for data and sim objects
                p.setGravity(0, 0, -10)
                p.setPhysicsEngineParameter(contactBreakingThreshold=.001)
                # load objects into pybullet
                plane_id = p.loadURDF("plane.urdf", flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
                p.changeDynamics(plane_id, -1, lateralFriction=.01, rollingFriction=0)
                # load hand
                hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand, 1)
                # load cube
                cube_id = p.loadURDF(
                    cube_path, basePosition=[0.0, 0.1067, .05],
                    flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
                p.changeDynamics(cube_id, -1, mass=3, restitution=.95, lateralFriction=1)
                controller = asterisk_controller.AsteriskController(
                    hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)
                controller.close_hand2(start=start)
                controller.move_hand2(directions[j])
                data_p = current_path + "/data_test/" + trial_name
                if not os.path.exists(data_p):
                    os.makedirs(data_p)
                controller.save(trial_name, directions[j], start, data_p)
                p.resetSimulation()


if __name__ == "__main__":
    # start pybullet
    # test()
    run_batch()
    # get_hand_paths()
    # setup_sim()
    # physics_client = p.connect(p.GUI)
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
    #                              cameraTargetPosition=[0, 0.1, 0.5])

    # directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    # start = time.time()
    # for i in range(len(directions)):
    #     p.setGravity(0, 0, -10)
    #     p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
    #                                  cameraTargetPosition=[0, 0.1, 0.5])
    #     p.setPhysicsEngineParameter(contactBreakingThreshold=.001)
    #     # load objects into pybullet
    #     plane_id = p.loadURDF("plane.urdf", flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
    #     p.changeDynamics(plane_id, -1, lateralFriction=.01, rollingFriction=0)
    #     # get paths for data and sim objects
    #     current_path, hand_path, cube_path, data_path = get_paths()
    #     # load hand
    #     test_hand = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]},
    #                  "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}}
    #     test_hand2 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.05174999999999999, 0], [0, 0.09974999999999999, 0]]}, "finger2": {
    #         "name": "finger1", "num_links": 2, "link_lengths": [[0, 0.09974999999999999, 0], [0, 0.05174999999999999, 0]]}}
    #     test_hand3 = {"finger1": {"name": "finger0", "num_links": 3, "link_lengths": [[0, .048, 0], [0, .048, 0], [0, .048, 0]]},
    #                   "finger2": {"name": "finger1", "num_links": 3, "link_lengths": [[0, .048, 0], [0, .048, 0], [0, .048, 0]]}}
    #     test_hand4 = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}, "finger2": {
    #         "name": "finger1", "num_links": 3, "link_lengths": [[0, .048, 0], [0, .048, 0], [0, .048, 0]]}}
    #     test_hand5 = {"name": "3v3_50.0.25.0.25.0_40.0.35.0.25.0_1.1_540", "sim": {"finger1": {"name": "finger0", "num_links": 3, "link_lengths": [[0, 0.072, 0], [
    #         0, 0.036, 0], [0, 0.036, 0]]}, "finger2": {"name": "finger1", "num_links": 3, "link_lengths": [[0, 0.0576, 0], [0, 0.0504, 0], [0, 0.036, 0]]}}}
    #     test_hand6 = {"name": "2v2_75.25_70.30_1.1_53", "sim": {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.10799999999999998, 0], [
    #         0, 0.036, 0]]}, "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, 0.1008, 0], [0, 0.043199999999999995, 0]]}}}
    #     test_hand7 = {"name": "2v3_60.40_40.30.30_1.0.9_63", "sim": {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, 0.08185263157894736, 0], [
    #         0, 0.05456842105263158, 0]]}, "finger2": {"name": "finger1", "num_links": 3, "link_lengths": [[0, 0.06063157894736842, 0], [0, 0.04547368421052631, 0], [0, 0.04547368421052631, 0]]}}}
    #     test_hand8 = {"name": "3v3_50.25.25_25.45.30_1.1_63", "sim": {"finger1": {"name": "finger0", "num_links": 3, "link_lengths": [[0, 0.072, 0], [0, 0.036, 0], [
    #         0, 0.036, 0]]}, "finger2": {"name": "finger1", "num_links": 3, "link_lengths": [[0, 0.036, 0], [0, 0.0648, 0], [0, 0.043199999999999995, 0]]}}}

    #     hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand8["sim"], 1)
    #     # hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand, 1)
    #     # load cube
    #     cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

    #     # time.sleep()
    #     p.changeDynamics(cube_id, -1, restitution=.95, mass=3, lateralFriction=1)
    #     p.changeVisualShape(cube_id, -1, rgbaColor=[0.5, 0.5, 0.5, 1])

    #     controller = asterisk_controller.AsteriskController(
    #         hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)
    #     controller.close_hand()
    #     # controller.close_hand2()
    #     controller.move_hand2(directions[i])
    #     data_p = current_path + "/data_angles/" + "test"
    #     if not os.path.exists(data_p):
    #         os.makedirs(data_p)

    #     controller.save("test", directions[i], data_p)
    #     p.resetSimulation()

    # print("END TIME", time.time() - start)


# reset_sim

# load(hand)
# load(cube)
# close_hand(hand)
# move_hand
# check for completion
