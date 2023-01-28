import pybullet as p
import numpy as np


class ForwardKinematicsSIM():
    def __init__(self, hand_id, hand_info: dict) -> None:
        # hand information and pybullet body id

        self.hand_info = hand_info
        self.hand_id = hand_id
        # finger information
        self.finger1 = hand_info["finger1"]
        self.finger2 = hand_info["finger2"]
        # number of links in each finger
        self.num_links_f1 = hand_info["finger1"]["num_links"]
        self.num_links_f2 = hand_info["finger2"]["num_links"]
        # link lengths of each finger
        self.link_lengths_f1 = hand_info["finger1"]["link_lengths"]
        self.link_lengths_f2 = hand_info["finger2"]["link_lengths"]
        # transforms
        self.current_link_transforms_f1 = []
        self.link_translations = []
        self.link_rotations = []
        self.current_link_transforms_f2 = []
        # angles
        self.current_angles_f1 = []
        self.current_angles_f2 = []
        # Current pose of each link
        self.current_poses_f1 = []
        self.current_poses_f2 = []
        # Holds the pybullet link ids for each finger
        self.link_ids_f1 = []
        self.link_ids_f2 = []
        # Holds the link names for each finger for debugging purposes
        self.link_names_f1 = []
        self.link_names_f2 = []
        # Create our link id and name lists above
        self.initialize_transforms()
        # debug
        self.debug_id_new = None
        self.debug_id_old = None

    def initialize_transforms(self):
        # get initial poses from sim and link ids (MUST BE DONE IN THIS ORDER)
        self.get_link_ids()
        self.update_poses_from_sim()

        # get the base link out first
        base_link_t = self.create_translation_matrix(self.current_poses_f1[0][0])
        base_link_r = self.create_rotation_matrix(p.getEulerFromQuaternion(self.current_poses_f1[0][1])[2])
        self.link_translations.append(base_link_t)
        self.link_rotations.append(base_link_r)
        self.current_angles_f1.append(p.getJointState(self.hand_id, self.link_ids_f1[0]))
        # Get the transformation from previous link to next link
        for i in range(1, len(self.current_poses_f1)):
            mat_t = self.create_translation_matrix(self.current_poses_f1[i][0])
            mat_r = self.create_rotation_matrix(p.getEulerFromQuaternion(self.current_poses_f1[i][1])[2])
            # Since poses are in the global frame we need to find the matrix that takes us from previous to next using A@B.I
            mat_t_link = mat_t @ np.linalg.inv(self.link_translations[-1])
            mat_r_link = mat_r @ np.linalg.inv(self.link_rotations[-1])
            self.link_translations.append(mat_t_link)
            self.link_rotations.append(mat_r_link)
            self.current_angles_f1.append(p.getJointState(self.hand_id, self.link_ids_f1[i]))
        print("DEBUG F1 STARTING Transforms: ", self.link_translations, self.link_rotations)

        # Do the same for Finger 2
        base_link = self.create_transformation_matrix(self.current_poses_f2[0][0], self.current_poses_f2[0][1])
        self.current_link_transforms_f2.append(base_link)
        self.current_angles_f2.append(p.getJointState(self.hand_id, self.link_ids_f2[0]))
        # Get the transformation from previous link to next link
        for i in range(1, len(self.current_poses_f2)):
            mat = self.create_transformation_matrix(self.current_poses_f2[i][0], self.current_poses_f2[i][1])
            # Since poses are in the global frame we need to find the matrix that takes us from previous to next using A@B.I
            mat_t_link = mat @ np.linalg.inv(self.current_link_transforms_f2[-1])
            self.current_link_transforms_f2.append(mat_t_link)
            self.current_angles_f2.append(p.getJointState(self.hand_id, self.link_ids_f2[i]))
        print("DEBUG F2 STARTING Transforms: ", self.current_link_transforms_f2)

    def create_translation_matrix(self, translation):
        mat_t = np.identity(3)
        mat_t[0][2] = translation[0]
        mat_t[1][2] = translation[1]
        mat_t[2][2] = 1
        return mat_t

    def create_transformation_matrix(self, translation, orientation):
        mat_t = np.identity(3)
        mat_t[0][2] = translation[0]
        mat_t[1][2] = translation[1]
        mat_t[2][2] = 1

        theta = p.getEulerFromQuaternion(orientation)[2]
        mat_t[0][0] = np.cos(theta)
        mat_t[0][1] = -np.sin(theta)
        mat_t[1][0] = np.sin(theta)
        mat_t[1][1] = np.cos(theta)
        return mat_t

    def create_rotation_matrix(self, theta):
        mat_r = np.identity(3)
        mat_r[0][0] = np.cos(theta)
        mat_r[0][1] = -np.sin(theta)
        mat_r[1][0] = np.sin(theta)
        mat_r[1][1] = np.cos(theta)
        return mat_r

    def set_joint_angles(self, angles):
        for i in range(len(angles)):
            mat_r = self.create_rotation_matrix(angles[i])
            self.link_rotations[i] = mat_r
            self.current_angles_f1 = angles[i]

    def calculate_forward_kinematics(self):
        debug = []
        link_location = np.identity(3)
        for i in range(len(self.link_lengths_f1)):
            link_location = link_location @ self.link_translations[i] @ self.link_rotations[i]
            debug.append(link_location @ [0, 0, 1])
            debug[-1][2] = .05
            print(f"LINK {i}: {link_location}")
        link_end = self.create_translation_matrix(self.link_lengths_f1[i])
        link_location = link_location @ link_end
        debug.append(link_location @ [0, 0, 1])
        debug[-1][2] = .05

        p.addUserDebugPoints(
            debug,
            [[255, 0, 0]] * 3,
            pointSize=10)

        return link_location

    def test(self):
        self.update_poses_from_sim()
        l0 = self.current_poses_f1[0]
        l1 = self.current_poses_f1[1]

        print(l0)
        m0 = self.create_transformation_matrix(l0[0], l0[1])

        print(m0)

        print(l1)
        m1 = self.create_transformation_matrix(l1[0], l1[1])
        print(p.getMatrixFromQuaternion(l1[1]))
        print(m1)

        f = m1@np.linalg.inv(m0)
        v = f @ m0
        print(f)
        print(v)
        mat_t = np.identity(3)
        theta = .1
        mat_t[0][0] = np.cos(theta)
        mat_t[0][1] = -np.sin(theta)
        mat_t[1][0] = np.sin(theta)
        mat_t[1][1] = np.cos(theta)
        m0 = m0 @ mat_t
        j = self.create_transformation_matrix([0, .072, 0], [0, 0, 0, 1])
        print(m0 @ j)
        print(m0 @ f @ j @ [0, 0, 1])
        b = m0 @ f @ j @ [0, 0, 1]
        b[2] = .05
        p.addUserDebugPoints(
            [b],
            [[0, 0, 255]],
            pointSize=20)

        t = np.arccos((np.trace(m0)-1)/2)
        print(t)

    def get_link_ids(self):
        for i in range(p.getNumJoints(self.hand_id)):
            j_info = p.getJointInfo(self.hand_id, i)
            j_name = j_info[12].decode('UTF-8')
            j_index = j_info[0]
            if "finger0" in j_name and "static" not in j_name:
                self.link_ids_f1.append(j_index)
                self.link_names_f1.append(j_name)
            if "finger1" in j_name and "static" not in j_name:
                self.link_ids_f2.append(j_index)
                self.link_names_f2.append(j_name)

    def debug_show_link_positions(self, poses_current=None, poses_new=None):
        print("POSES OLD", poses_current)
        print("POSES NEW", poses_new)
        if self.debug_id_new:
            p.removeUserDebugItem(self.debug_id_old)
            p.removeUserDebugItem(self.debug_id_new)

        if poses_current:
            self.debug_id_old = p.addUserDebugPoints(
                [item[0] for item in poses_current],
                [[0, 255, 0]] * len(poses_current),
                pointSize=10)
        if poses_new:
            self.debug_id_new = p.addUserDebugPoints(
                [item[0] for item in poses_new],
                [[255, 0, 0]] * len(poses_new),
                pointSize=10)

    def update_poses_from_sim(self, finger=1):
        # get each current link pose in global coordinates [(x,y,z), (x,y,z,w)]
        for id in self.link_ids_f1:
            l_state = p.getLinkState(self.hand_id, id)
            self.current_poses_f1.append([list(l_state[0]), list(l_state[1])])

        # get each current link pose in global coordinates [(x,y,z), (x,y,z,w)]
        for id in self.link_ids_f2:
            l_state = p.getLinkState(self.hand_id, id)
            self.current_poses_f2.append([list(l_state[0]), list(l_state[1])])

    def calculate_FK(self, angles_f1=None, angles_f2=None, ee_location=None):
        ee_link_length = self.link_lengths_f1[-1]
        if ee_location:
            # Calculate link length at new location
            # ee_link_length = 0
            pass

        if len(self.current_poses_f1) == 0:
            self.update_poses_from_sim()

        new_link_poses = []
        # get the bottom link without the link length added
        bottom_link = p.multiplyTransforms(
            self.current_poses_f1[0][0],
            self.current_poses_f1[0][1],
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, angles_f1[0]]))
        bottom_link = [list(bottom_link[0]), list(bottom_link[1])]
        new_link_poses.append(bottom_link)
        # for every link, we use the previous link pose and transform it using the current link length and angle to give us the link end pose
        for i in range(1, len(angles_f1)):
            print("ROTATION PRIOR: ", self.current_poses_f1[i])
            rotation = p.multiplyTransforms(
                self.current_poses_f1[i][0],
                self.current_poses_f1[i][1],
                [0, 0, 0],
                p.getQuaternionFromEuler([0, 0, angles_f1[i]]))

            next_link = p.multiplyTransforms(
                new_link_poses[-1][0],
                new_link_poses[-1][1],
                self.link_lengths_f1[i],
                [0, 0, 0, 1])
            next_link = p.multiplyTransforms(
                next_link[0],
                next_link[1],
                [0, 0, 0],
                rotation[1])
            print("ROTATION: ", rotation)
            next_link = [list(next_link[0]), list(next_link[1])]
            new_link_poses.append(next_link)
        # get final end effector pose
        ee_link_pose = p.multiplyTransforms(
            new_link_poses[-1][0],
            new_link_poses[-1][1],
            ee_link_length,
            [0, 0, 0, 1])
        ee_link_pose = [list(ee_link_pose[0]), list(ee_link_pose[1])]

        # DEBUG
        self.debug_show_link_positions(self.current_poses_f1, new_link_poses)
        self.current_poses_f1 = new_link_poses
        return new_link_poses, ee_link_pose
