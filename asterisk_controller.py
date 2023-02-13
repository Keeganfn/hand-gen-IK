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
            "N": np.array([0.01, .15]),
            "NE": np.array([0.2, .2]),
            "E": np.array([0.2, .1067]),
            "SE": np.array([.2, 0]),
            "S": np.array([0.01, .05]),
            "SW": np.array([-0.18, 0]),
            "W": np.array([-0.18, .1067]),
            "NW": np.array([-0.18, .2])}
        self.f2_direction_dict = {
            "N": np.array([-0.01, .15]),
            "NE": np.array([0.18, .2]),
            "E": np.array([0.18, .1067]),
            "SE": np.array([0.18, 0]),
            "S": np.array([-0.01, .05]),
            "SW": np.array([-0.2, 0]),
            "W": np.array([-0.2, .1067]),
            "NW": np.array([-0.2, .2])}

    def get_cube_position(self):
        #p.changeDynamics(self.hand_id, 1, mass=5)
        #p.changeDynamics(self.hand_id, 3, mass=5)
        print(p.getBasePositionAndOrientation(self.cube_id)[0])
        return p.getBasePositionAndOrientation(self.cube_id)[0]

    def step_towards_goal(self, start_vec, end_vec, distance):
        temp_x = end_vec[0] - start_vec[0]
        temp_y = end_vec[1] - start_vec[1]
        magnitude = math.sqrt((temp_x**2 + temp_y**2))
        if magnitude <= distance:
            return [start_vec[0], start_vec[1]]
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
        target_f1 = np.array([0.019, self.get_cube_position()[1]])
        target_f2 = np.array([-0.019, self.get_cube_position()[1]])
        contact_point_info1 = None
        contact_point_info2 = None
        tsteps = 0
        while tsteps < 1000:
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
                found, angles_f1, it = self.ik_f1.calculate_ik(sub_target_f1)
                p.setJointMotorControlArray(self.hand_id, self.ik_f1.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f1)
            if not contact_point_info2:
                found, angles_f2, it = self.ik_f2.calculate_ik(sub_target_f2)
                p.setJointMotorControlArray(self.hand_id, self.ik_f2.finger_fk.link_ids,
                                            p.POSITION_CONTROL, targetPositions=angles_f2)
            tsteps += 1
            p.stepSimulation()

    def move_hand(self, direction, debug=False):
        target_f1 = self.f1_direction_dict[direction]
        target_f2 = self.f2_direction_dict[direction]

        start = time.time()
        tsteps = 0
        cp1_count = 0
        cp2_count = 0
        cp3_count = 0
        while tsteps < 200:
            start_f1 = self.ik_f1.finger_fk.calculate_forward_kinematics()
            start_f2 = self.ik_f2.finger_fk.calculate_forward_kinematics()
            sub_target_f1 = np.array(self.step_towards_goal(start_f1, target_f1, .01))
            sub_target_f2 = np.array(self.step_towards_goal(start_f2, target_f2, .01))
            if debug:
                self.show_points_debug(sub_target_f1)
                self.show_points_debug(sub_target_f2)

            contact_point_info1 = p.getContactPoints(bodyA=self.cube_id, bodyB=self.hand_id, linkIndexB=self.distal_f1)
            contact_point_info2 = p.getContactPoints(bodyA=self.cube_id, bodyB=self.hand_id, linkIndexB=self.distal_f2)

            if np.allclose(start_f1[:2], target_f1, atol=5e-3) or np.allclose(start_f2[:2], target_f2, atol=5e-3):
                print("Close enough here")
                break

            if not contact_point_info1 and not contact_point_info2:
                break

            if not contact_point_info1:
                cp1_count += 1
                if cp1_count > 5:
                    print("BROKEN CONTACT 1")
                    break

            if not contact_point_info2:
                cp2_count += 1
                if cp2_count > 5:
                    print("BROKEN CONTACT 2")
                    break

            if contact_point_info2:
                #print("GOt HERE")
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
        print("CLOSE TIME", time.time() - start)
