from .matrix_helper import MatrixHelp
from .forward_kinematics_live import ForwardKinematicsLIVE
import numpy as np


class JacobianIKLIVE():

    def __init__(self, hand_id, finger_info) -> None:
        # Get forward IK info for each finger
        self.finger_fk = ForwardKinematicsLIVE(hand_id, finger_info)
        self.mh = MatrixHelp()

        # TODO: May 
        self.MAX_ITERATIONS = 10000
        self.MAX_STEP = .005
        self.STARTING_STEP = 1
        self.ERROR = .1

        pass

    def update_angles(self, angles):
        self.finger_fk.update_angles_from_motors(angles)

    def calculate_jacobian(self):
        mat_jacob = np.zeros([2, self.finger_fk.num_links])
        angles = self.finger_fk.current_angles.copy()
        link_end_locations = self.finger_fk.link_lengths.copy()
        angles.reverse()
        link_end_locations.reverse()

        # matrix of accumulated values
        mat_accum = np.identity(3)
        # The z vector we spin around
        omega_hat = [0, 0, 1]

        total_angles = sum(angles)
        for i, (ang, length) in enumerate(zip(angles, link_end_locations)):
            total_angles -= ang
            mat_accum = self.mh.create_rotation_matrix(ang) @ self.mh.create_translation_matrix(length) @ mat_accum
            mat_r = self.mh.create_rotation_matrix(total_angles) @ mat_accum
            r = [mat_r[0, 2], mat_r[1, 2], 0]
            omega_cross_r = np.cross(omega_hat, r)
            mat_jacob[0:2, self.finger_fk.num_links - i - 1] = np.transpose(omega_cross_r[0:2])

        return mat_jacob

    def solve_jacobian(self, jacobian, vx_vy):
        """ Do the pseudo inverse of the jacobian
        @param - jacobian - the 2xn jacobian you calculated from the current joint angles/lengths
        @param - vx_vy - a 2x1 numpy array with the distance to the target point (vector_to_goal)
        @return - changes to the n joint angles, as a 1xn numpy array"""
        res = np.linalg.lstsq(jacobian, vx_vy, rcond=None)
        delta_angles = res[0]

        return delta_angles

    def vector_to_goal(self, target):
        end_pt = self.finger_fk.calculate_forward_kinematics()
        return np.array(target) - np.array(end_pt[:2])

    def distance_to_goal(self, target):
        vec = self.vector_to_goal(target)
        return np.sqrt(vec[0] * vec[0] + vec[1] * vec[1])
    # TODO: EE is [x,y,z] but z doesn't matter (make 1)
    # If you pass None it goes to default position
    def calculate_ik(self, target, b_one_step=False, ee_location=None):
        """
        Use jacobian to calculate angles that move the grasp point towards target. Instead of taking 'big' steps we're
        going to take small steps along the vector (because the Jacobian is only valid around the joint angles)
        @param arm - The arm geometry, as constructed in arm_forward_kinematics
        @param angles - A list of angles for each link, followed by a triplet for the wrist and fingers
        @param target - a 2x1 numpy array (x,y) that is the desired target point
        @param b_one_step - if True, return angles after one successful movement towards goal
        @ return if we got better and angles that put the grasp point as close as possible to the target, and number
                of iterations it took
        """
        b_keep_going = True
        b_found_better = False
        self.finger_fk.update_ee_end_point(ee_location)
        # TODO: Update angles from motors
        #self.finger_fk.update_angles_from_sim()
        angles = self.finger_fk.current_angles.copy()
        best_distance = self.distance_to_goal(target)
        count_iterations = 0
        d_step = self.MAX_STEP

        while b_keep_going and count_iterations < self.MAX_ITERATIONS:

            # This is the vector to the target. Take maximum 0.05 of a step towards the target
            vec_to_target = self.vector_to_goal(target)
            vec_length = np.linalg.norm(vec_to_target)
            if vec_length > d_step:
                # shorten step
                vec_to_target *= d_step / vec_length
            elif np.isclose(vec_length, 0.0):
                b_keep_going = False

            delta_angles = np.zeros(len(angles))
            self.finger_fk.set_joint_angles(angles)
            jacobian = self.calculate_jacobian()
            delta_angles = self.solve_jacobian(jacobian, vec_to_target)
            if delta_angles is None:
                return b_found_better, angles, count_iterations
            #print("JACOBIAN ", jacobian)
            #print("DELTA_ANGLES ", delta_angles)

            # This rarely happens - but if the matrix is degenerate (the arm is in a straight line) then the angles
            #  returned from solve_jacobian will be really, really big. The while loop below will "fix" this, but this
            #  just shortcuts the whole problem. There are far, far better ways to deal with this
            avg_ang_change = np.linalg.norm(delta_angles)
            if avg_ang_change > 100:
                print("JACOBIAN TOO LARGE")
                # print(delta_angles)
                # print(target)
                delta_angles *= 0.1 / avg_ang_change
                # print(delta_angles)
            elif avg_ang_change < 0.000001:
                print("JACOBIAN TOO SMALL")
                # print(delta_angles)
                delta_angles *= 0.1 / avg_ang_change

            b_took_one_step = False
            # Start with a step size of 1 - take one step along the gradient
            step_size = self.STARTING_STEP
            # Two stopping criteria - either never got better OR one of the steps worked
            while step_size > self.MAX_STEP and not b_took_one_step:
                # print(self.finger_fk.current_angles)
                new_angles = []
                for i, a in enumerate(angles):
                    new_angles.append(a + step_size * delta_angles[i])
                # Get the new distance with the new angles
                self.finger_fk.set_joint_angles(new_angles)
                new_dist = self.distance_to_goal(target)

                if new_dist > best_distance:
                    step_size *= 0.5
                else:
                    b_took_one_step = True
                    angles = new_angles
                    best_distance = new_dist
                    b_found_better = True
                count_iterations += 1

            # We can stop if we're close to the goal
            if np.isclose(best_distance, 0, atol=1e-3):
                b_keep_going = False

            # End conditions - b_one_step is true  - don't do another round
            #   OR we didn't take a step (b_took_one_step)
            if b_one_step or not b_took_one_step:
                b_keep_going = False

        #self.finger_fk.update_angles_from_sim()
        # TODO: Update angles by motors
        # Return the new angles, and whether or not we ever found a better set of angles
        return b_found_better, angles, count_iterations