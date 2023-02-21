from copy import deepcopy
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
        flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_SELF_COLLISION)
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

    p.changeDynamics(hand_id, 1, lateralFriction=1, rollingFriction=.03, mass=3)
    p.changeDynamics(hand_id, 3, lateralFriction=1, rollingFriction=.03, mass=3)
    p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeDynamics(hand_id, 0, jointLowerLimit=-2.09, jointUpperLimit=1.39)
    p.changeDynamics(hand_id, 1, jointLowerLimit=-1.57, jointUpperLimit=2.09)
    p.changeDynamics(hand_id, 2, jointLowerLimit=-1.39, jointUpperLimit=2.09)
    p.changeDynamics(hand_id, 3, jointLowerLimit=-2.09, jointUpperLimit=1.57)
    return hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index


def get_paths():
    # resource paths
    current_path = str(pathlib.Path().resolve())
    hand_path = current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf"
    hand_path2 = current_path+"/resources/2v2_2.1_1.2_1.1_8.10/hand/2v2_2.1_1.2_1.1_8.10.urdf"
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
    p.changeDynamics(plane_id, -1, lateralFriction=.4)


def get_hand_paths():
    current_path = str(pathlib.Path().resolve())

    paths = []
    names = []
    for file in os.listdir(current_path + "/generated_hands"):
        print(str(file))
        print(len(names))
        temp_str = current_path + "/generated_hands/" + str(file) + "/hand/" + str(file) + ".urdf"
        paths.append(temp_str)
        names.append(str(file))

    paths.pop(-1)
    print(len(names))
    names.pop(-1)
    print(names[60])
    print(len(names))

    with open(current_path + "/generated_hands/hand_descriptions.json", "r+") as fp:
        hand_descs = json.load(fp)

    while len(hand_descs) <= len(names):
        hand_descs.append({"name": "dub"})

    return paths, hand_descs, names


def run_batch():
    setup_sim()
    hand_paths, hand_descs, names = get_hand_paths()
    print(len(names), len(hand_descs))
    for i in range(len(hand_paths)):
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
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

        for j in range(len(directions)):
            # get paths for data and sim objects
            # load hand
            hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand, 1)
            # load cube
            cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
            p.changeDynamics(cube_id, -1, mass=5, restitution=.95)
            controller = asterisk_controller.AsteriskController(
                hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)
            controller.close_hand()
            controller.move_hand2(directions[j])
            data_p = current_path + "/data/" + trial_name
            if not os.path.exists(data_p):
                os.makedirs(data_p)
            controller.save(trial_name, directions[j], data_p)
            p.resetSimulation()


if __name__ == "__main__":
    # start pybullet
    # test()
    # run_batch()
    get_hand_paths()
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

        hand_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index = setup_hand(hand_path, test_hand, 1)
        # load cube
        cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.1067, .05], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        p.changeDynamics(cube_id, -1, restitution=.95, mass=5)

        controller = asterisk_controller.AsteriskController(
            hand_id, cube_id, ik_f1, ik_f2, distal_f1_index, distal_f2_index)
        controller.close_hand()
        # controller.close_hand2()
        controller.move_hand2(directions[i])
        data_p = current_path + "/data_angles"
        controller.save("2v2_1.1_1.1_1.1_1.1", directions[i], data_p)
        p.resetSimulation()

    # print("END TIME", time.time() - start)


# reset_sim

# load(hand)
# load(cube)
# close_hand(hand)
# move_hand
# check for completion
