import numpy as np
import pybullet as p
import time
import math
import matrix_helper as mh


class AsteriskController():

    def __init__(self, hand_id, cube_id, ik_f1, ik_f2, distal_f1, distal_f2) -> None:
        self.hand_id = hand_id
        self.cube_id = cube_id
        self.ik_f1 = ik_f1
        self.ik_f2 = ik_f2
        self.distal_f1 = distal_f1
        self.distal_f2 = distal_f2
        self.f1_direction_dict = {
            "E": np.array([0.15, .1]),
            "W": np.array([-0.13, .1]),
            "N": np.array([0.01, .15]),
            "S": np.array([0.01, .05]),
            "NE": np.array([0.15, .15]),
            "NW": np.array([-0.13, .15]),
            "SE": np.array([0.15, .03]),
            "SW": np.array([-0.13, .03])}
        self.f2_direction_dict = {
            "E": np.array([0.13, .1]),
            "W": np.array([-0.15, .1]),
            "N": np.array([-0.01, .15]),
            "S": np.array([-0.01, .05]),
            "NE": np.array([0.13, .15]),
            "NW": np.array([-0.15, .15]),
            "SE": np.array([0.13, .03]),
            "SW": np.array([-0.15, .03])}

    def get_cube_position(self):
        #p.changeDynamics(self.cube_id, -1, lateralFriction=.95, rollingFriction=.95, spinningFriction=.95)
        #p.changeDynamics(self.hand_id, 1, lateralFriction=10, rollingFriction=10, spinningFriction=10)
        #p.changeDynamics(self.hand_id, 3, lateralFriction=10, rollingFriction=10, spinningFriction=10)
        print(p.getBasePositionAndOrientation(self.cube_id)[0])
        return p.getBasePositionAndOrientation(self.cube_id)[0]

    def step_towards_goal(self, start_vec, end_vec, distance):
        temp_x = end_vec[0] - start_vec[0]
        temp_y = end_vec[1] - start_vec[1]
        magnitude = math.sqrt((temp_x**2 + temp_y**2))
        temp_x /= magnitude
        temp_y /= magnitude
        temp_x = start_vec[0] + distance*temp_x
        temp_y = start_vec[1] + distance*temp_y
        return [temp_x, temp_y]

    def show_points_debug(self, debug_points):
        debug_points = [debug_points[0], debug_points[1], .05]
        debug_id_new = p.addUserDebugPoints(
            [debug_points],
            [[255, 0, 0]],
            pointSize=10)

    def close_hand(self, debug=False):
        target_f1 = np.array([0.01, self.get_cube_position()[1]])
        target_f2 = np.array([-0.01, self.get_cube_position()[1]])
        contact_point_info1 = None
        contact_point_info2 = None
        start = time.time()
        tsteps = 0
        while tsteps < 1000:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .01))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .01))

            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f1)
            contact_point_info2 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f2)
            if contact_point_info1 and contact_point_info2:
                break
            if not contact_point_info1:
                found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1)
                # print(found)
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            if not contact_point_info2:
                found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2)
                print(found)
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)
            tsteps += 1
            p.stepSimulation()

        print("CLOSE TIME", time.time() - start)

    def move_hand(self, direction, debug=False):
        target_f1 = self.f1_direction_dict[direction]
        target_f2 = self.f2_direction_dict[direction]

        start = time.time()
        tsteps = 0
        while tsteps < 1000:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .01))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .01))
            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f1)
            contact_point_info2 = p.getContactPoints(self.cube_id, self.hand_id, linkIndexB=self.distal_f2)
            # print(contact_point_info1[-1])
            if not contact_point_info1 and not contact_point_info2:
                # print("here")
                # sub_target_f1 = np.array(self.step_towards_goal(start_f1, self.get_cube_position(), .01))
                # sub_target_f2 = np.array(self.step_towards_goal(start_f2, self.get_cube_position(), .01))
                # found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1, ee_location=None)
                # found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2, ee_location=None)
                # p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                #                             p.POSITION_CONTROL, targetPositions=angles_f1)
                # p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                #                             p.POSITION_CONTROL, targetPositions=angles_f2)

                pass
                # break
            if np.allclose(start_f1[:2], target_f1, atol=5e-3) or np.allclose(start_f2[:2], target_f2, atol=5e-3):
                print("Close enough here")
                break
            if contact_point_info1:
                t1 = mh.create_translation_matrix(contact_point_info1[-1][6])
                f1 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, 1)[0],
                    p.getLinkState(self.hand_id, 1)[1])
                cp1 = np.linalg.inv(f1) @ t1 @ [0, 0, 1]
                found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1, ee_location=cp1)
                print(found)
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            if contact_point_info2:
                print("GOt HERE")
                t2 = mh.create_translation_matrix(contact_point_info2[-1][6])
                f2 = mh.create_transformation_matrix(
                    p.getLinkState(self.hand_id, 3)[0],
                    p.getLinkState(self.hand_id, 3)[1])
                cp2 = np.linalg.inv(f2) @ t2 @ [0, 0, 1]
                found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2, ee_location=cp2)
                print(found)
                print(self.ik_f2.finger_fk.link_ids)
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)
            tsteps += 1
            p.stepSimulation()
        print("CLOSE TIME", time.time() - start)
