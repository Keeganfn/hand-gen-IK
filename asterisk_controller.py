import numpy as np
import pybullet as p
import time
import math
import matrix_helper as mh
import pickle as pkl


class AsteriskController():

    def __init__(self, hand_id, cube_id, ik_f1, ik_f2, distal_f1, distal_f2) -> None:
        self.hand_id = hand_id
        self.cube_id = cube_id
        self.ik_f1 = ik_f1
        self.ik_f2 = ik_f2
        self.distal_f1 = distal_f1
        self.distal_f2 = distal_f2
        # self.f1_direction_dict = {
        #     "N": np.array([0.015, .158]),
        #     "NE": np.array([0.065, .158]),
        #     "E": np.array([0.165, .108]),
        #     "SE": np.array([.165, -.0446]),
        #     "S": np.array([0.013, -.0446]),
        #     "SW": np.array([-0.135, -.0446]),
        #     "W": np.array([-0.135, .108]),
        #     "NW": np.array([-0.035, .1567])}
        # self.f2_direction_dict = {
        #     "N": np.array([-0.015, .158]),
        #     "NE": np.array([.035, .158]),
        #     "E": np.array([0.135, .108]),
        #     "SE": np.array([0.135, -.0446]),
        #     "S": np.array([-0.013, -.0446]),
        #     "SW": np.array([-0.165, -.0446]),
        #     "W": np.array([-0.165, .108]),
        #     "NW": np.array([-0.065, .158])}

        self.f1_direction_dict = {
            "N": np.array([0.015, .1567]),
            "NE": np.array([0.065, .1567]),
            "E": np.array([0.165, .1067]),
            "SE": np.array([.165, -.0433]),
            "S": np.array([0.013, -.0433]),
            "SW": np.array([-0.135, -.0433]),
            "W": np.array([-0.135, .1067]),
            "NW": np.array([-0.035, .1567])}
        self.f2_direction_dict = {
            "N": np.array([-0.015, .1567]),
            "NE": np.array([.035, .1567]),
            "E": np.array([0.135, .1067]),
            "SE": np.array([0.135, -.0433]),
            "S": np.array([-0.013, -.0433]),
            "SW": np.array([-0.165, -.0433]),
            "W": np.array([-0.165, .1067]),
            "NW": np.array([-0.065, .1567])}
        # self.f1_direction_dict = {
        #     "N": np.array([0.015, .15]),
        #     "NE": np.array([0.215, .2]),
        #     "E": np.array([0.2, .1067]),
        #     "SE": np.array([.2, 0]),
        #     "S": np.array([0.015, .05]),
        #     "SW": np.array([-0.17, 0]),
        #     "W": np.array([-0.17, .1067]),
        #     "NW": np.array([-0.17, .2])}
        # self.f2_direction_dict = {
        #     "N": np.array([-0.015, .15]),
        #     "NE": np.array([0.185, .2]),
        #     "E": np.array([0.17, .1067]),
        #     "SE": np.array([0.17, 0]),
        #     "S": np.array([-0.015, .05]),
        #     "SW": np.array([-0.2, 0]),
        #     "W": np.array([-0.2, .1067]),
        #     "NW": np.array([-0.2, .2])}
        self.trial_data = []
        # self.f1_direction_dict = {
        #     "N": np.array([0.01, .15]),
        #     "NE": np.array([0.2, .2]),
        #     "E": np.array([0.2, .1067]),
        #     "SE": np.array([.2, 0]),
        #     "S": np.array([0.01, .05]),
        #     "SW": np.array([-0.18, 0]),
        #     "W": np.array([-0.18, .1067]),
        #     "NW": np.array([-0.18, .2])}
        # self.f2_direction_dict = {
        #     "N": np.array([-0.01, .15]),
        #     "NE": np.array([0.18, .2]),
        #     "E": np.array([0.18, .1067]),
        #     "SE": np.array([0.18, 0]),
        #     "S": np.array([-0.01, .05]),
        #     "SW": np.array([-0.2, 0]),
        #     "W": np.array([-0.2, .1067]),
        #     "NW": np.array([-0.2, .2])}

    def get_cube_position(self):
        # p.changeDynamics(self.hand_id, 1, mass=5)
        # p.changeDynamics(self.hand_id, 3, mass=5)
        # print(p.getBasePositionAndOrientation(self.cube_id)[0])
        return p.getBasePositionAndOrientation(self.cube_id)[0]

    def step_towards_goal(self, start_vec, end_vec, distance):
        temp_x = end_vec[0] - start_vec[0]
        temp_y = end_vec[1] - start_vec[1]
        magnitude = math.sqrt((temp_x**2 + temp_y**2))
        if magnitude <= distance:
            return [end_vec[0], end_vec[1]]
        temp_x /= magnitude
        temp_y /= magnitude
        temp_x = start_vec[0] + distance*temp_x
        temp_y = start_vec[1] + distance*temp_y
        return [temp_x, temp_y]

    def show_points_debug(self, debug_points):
        debug_points = [debug_points[0], debug_points[1], .05]
        debug_id_new = p.addUserDebugPoints(
            [debug_points],
            [[0, 255, 0]],
            pointSize=10)

    def close_hand(self, debug=False):
        target_f1 = np.array([0.0195, self.get_cube_position()[1] + 0])
        target_f2 = np.array([-0.0195, self.get_cube_position()[1] + 0])
        contact_point_info1 = None
        contact_point_info2 = None
        tsteps = 0
        while tsteps < 100000:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .005))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .005))

            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f1)
            contact_point_info2 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f2)
            if contact_point_info1 and contact_point_info2:
                break

            if not contact_point_info1:
                found, angles_f1, it = self.ik_f1.calculate_ik(
                    sub_target_f1, ee_location=[-.011, self.ik_f1.finger_fk.original_ee_end[1]-.005, 1])
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            if not contact_point_info2:
                found, angles_f2, it = self.ik_f2.calculate_ik(
                    sub_target_f2, ee_location=[.011, self.ik_f2.finger_fk.original_ee_end[1]-.005, 1])
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)
            tsteps += 1
            p.stepSimulation()

    def record(self, ik_angles, d1, d2):
        temp = self.ik_f1.finger_fk.current_angles + self.ik_f2.finger_fk.current_angles
        obj_pos = p.getBasePositionAndOrientation(self.cube_id)
        obj_pos1 = list(obj_pos[0])
        obj_pos1[1] -= .1067
        if not d1:
            d1 = 1000
        if not d2:
            d2 = 1000
        save_dict = {
            "obj_pos": obj_pos1,
            "obj_or": obj_pos[1],
            "ik_angles": ik_angles,
            "d1": d1,
            "d2": d2,
            "joint_1": temp[0],
            "joint_2": temp[1],
            "joint_3": temp[2],
            "joint_4": temp[3],  
            }
        self.trial_data.append(save_dict)

    def save(self, name, direction, path):
        # label = "data/" + direction + "_" + name + ".pkl"
        label = path + "/" + direction + "_" + name + ".pkl"

        with open(label, "wb+") as file:
            pkl.dump(self.trial_data, file)
        self.trial_data = []

    def move_hand2(self, direction, debug=False):
        target_f1 = self.f1_direction_dict[direction]
        target_f2 = self.f2_direction_dict[direction]

#        start = time.time()
        tsteps = 0
        cp1_count = 0
        cp2_count = 0
        cp3_count = 0
        thresh = .005
        while tsteps < 600:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .003))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .003))
            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getClosestPoints(self.hand_id, self.cube_id, .003, linkIndexA=self.distal_f1)
            contact_point_info2 = p.getClosestPoints(self.hand_id, self.cube_id, .003, linkIndexA=self.distal_f2)
            rot = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.cube_id)[1])
            if abs(rot[0]) > .3 or abs(rot[0]) > .3:
                print("ROTATION IS BAD")
                break
            vel = p.getBaseVelocity(self.cube_id)[0]
            if abs(vel[0]) > .55 or abs(vel[1]) > .55:
                print("velocity IS BAD")
                break

            if contact_point_info1:
                d1 = abs(contact_point_info1[-1][6][0]) - abs(contact_point_info1[-1][5][0]
                                                              ) + abs(contact_point_info1[-1][6][1]) - abs(contact_point_info1[-1][5][1])
            else:
                d1 = None
            if contact_point_info2:
                d2 = abs(contact_point_info2[-1][6][0]) - abs(contact_point_info2[-1][5][0]
                                                              ) + abs(contact_point_info2[-1][6][1]) - abs(contact_point_info2[-1][5][1])
            else:
                d2 = None

            if np.allclose(start_f1[:2], target_f1, atol=5e-3) or np.allclose(start_f2[:2], target_f2, atol=5e-3):
                print("Close enough here")
                break

            if not d2 and not d1:
                print("LOST CONTACT")
                break

            if not d1:
                print("BROKEN CONTACT 1")
                break

            if not d2:
                print("BROKEN CONTACT 2")
                break

            if contact_point_info2:
                # print("GOt HERE")

                cp1_count = 0
                t2 = mh.create_translation_matrix(contact_point_info2[-1][6])
                f2 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, self.distal_f2)[0],
                    p.getLinkState(self.hand_id, self.distal_f2)[1])
                cp2 = np.linalg.inv(f2) @ t2 @ [0, 0, 1]
                found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2, ee_location=cp2)
                if not angles_f2:
                    break
                # print(self.ik_f2.finger_fk.link_ids)
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)

            if contact_point_info1:
                cp2_count = 0
                t1 = mh.create_translation_matrix(contact_point_info1[-1][6])
                f1 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, self.distal_f1)[0],
                    p.getLinkState(self.hand_id, self.distal_f1)[1])
                cp1 = np.linalg.inv(f1) @ t1 @ [0, 0, 1]
                found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1, ee_location=cp1)
                if not angles_f1:
                    break
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            tsteps += 1

            self.record(angles_f1+angles_f2, d1, d2)
            p.stepSimulation()
            # time.sleep(.5)
#        print("CLOSE TIME", time.time() - start, tsteps)

    def move_hand(self, direction, debug=False):
        target_f1 = self.f1_direction_dict[direction]
        target_f2 = self.f2_direction_dict[direction]

        start = time.time()
        tsteps = 0
        cp1_count = 0
        cp2_count = 0
        cp3_count = 0
        while tsteps < 1000:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .002))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .002))
            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getContactPoints(
                bodyA=self.cube_id, bodyB=self.hand_id, linkIndexB=self.distal_f1)
            contact_point_info2 = p.getContactPoints(
                bodyA=self.cube_id, bodyB=self.hand_id, linkIndexB=self.distal_f2)
            closest1 = p.getClosestPoints(self.hand_id, self.cube_id, .1, linkIndexA=1)
            closest2 = p.getClosestPoints(self.hand_id, self.cube_id, .1, linkIndexA=3)
            print(abs(closest[-1][6][0]) - abs(closest[-1][5][0]) + abs(closest[-1][6][1]) - abs(closest[-1][5][1]))

            if np.allclose(start_f1[:2], target_f1, atol=5e-3) or np.allclose(start_f2[:2], target_f2, atol=5e-3):
                print("Close enough here")
                break

            if not contact_point_info1 and not contact_point_info2:
                print("LOST CONTACT")
                break

            if not contact_point_info1:
                cp1_count += 1
                if cp1_count > 30:
                    print("BROKEN CONTACT 1")
                    break

            if not contact_point_info2:
                cp2_count += 1
                if cp2_count > 30:
                    print("BROKEN CONTACT 2")
                    break

            if contact_point_info2:
                # print("GOt HERE")

                cp1_count = 0
                t2 = mh.create_translation_matrix(contact_point_info2[-1][6])
                f2 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, 3)[0],
                    p.getLinkState(self.hand_id, 3)[1])
                cp2 = np.linalg.inv(f2) @ t2 @ [0, 0, 1]
                found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2, ee_location=cp2)
                # print(self.ik_f2.finger_fk.link_ids)
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)

            if contact_point_info1:
                p.getClosestPoints(self.hand_id, self.cube_id, 2, linkIndexA=3)
                cp2_count = 0
                t1 = mh.create_translation_matrix(contact_point_info1[-1][6])
                f1 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, 1)[0],
                    p.getLinkState(self.hand_id, 1)[1])
                cp1 = np.linalg.inv(f1) @ t1 @ [0, 0, 1]
                found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1, ee_location=cp1)
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            tsteps += 1
            p.stepSimulation()
#        print("CLOSE TIME", time.time() - start)
