import numpy as np
from .matrix_helper import MatrixHelp


class ForwardKinematicsLIVE():
    def __init__(self, hand_id, finger_info: dict) -> None:
        self.mh = MatrixHelp()
        # finger information and pybullet body id
        self.hand_id = hand_id
        self.finger_name = finger_info["name"]
        self.num_links = finger_info["num_links"]
        self.base_link_offset = finger_info["offset"]
        self.link_lengths = finger_info["link_lengths"]
        # transforms
        self.link_translations = []
        self.link_rotations = []
        self.current_angles = []
        # ee_ending location
        self.original_ee_end = None
       # Create our link id and name lists above
        self.initialize_transforms()
        # Holds the link names for each finger for debugging purposes
        self.debug_id_new = None

    def update_ee_end_point(self, ee_ending_local=None):
        if ee_ending_local is not None:
            self.link_lengths[-1] = ee_ending_local
        else:
            self.link_lengths[-1] = self.original_ee_end

    # TODO: Radians from motor [b_r, t_r, b_l, t_l]
    def update_angles_from_motors(self, angles):
        # get each current link pose in global coordinates [(x,y,z), (x,y,z,w)]
        for i in range(len(self.current_angles)):
            self.current_angles[i] = angles[i]
        self.set_joint_angles(self.current_angles)

    def initialize_transforms(self):
        # get the base link out first
        base_link_t = self.mh.create_translation_matrix(self.base_link_offset)
        base_link_r = self.mh.create_rotation_matrix(0)
        self.link_translations.append(base_link_t)
        self.link_rotations.append(base_link_r)
        self.current_angles.append(0)
        # Get the transformation from previous link to next link
        for i in range(1, len(self.link_lengths)):
            mat_t = self.mh.create_translation_matrix(self.link_lengths[i])
            mat_r = self.mh.create_rotation_matrix(0)
            # Since poses are in the global frame we need to find the matrix that takes us from previous to next using A@B.I
            self.link_translations.append(mat_t)
            self.link_rotations.append(mat_r)
            self.current_angles.append(0)
        self.original_ee_end = self.link_lengths[-1]
        #print(f"DEBUG\nLINK_TRANSLATIONS:\n{self.link_translations}\nLINK_ROTATIONS:\n{self.link_rotations}\nCURRENT_ANGLES:\n{self.current_angles}")

    def set_joint_angles(self, angles):
        # sets the joint angles and updates rotation matrices
        for i in range(len(angles)):
            mat_r = self.mh.create_rotation_matrix(angles[i])
            self.link_rotations[i] = mat_r
            self.current_angles[i] = angles[i]

    def calculate_forward_kinematics(self):
        link_location = np.identity(3)
        # iterate over every link and multiply the transforms together
        for i in range(len(self.link_lengths)):
            link_location = link_location @ self.link_translations[i] @ self.link_rotations[i]
        # finally get the end effector end location
        link_end = self.mh.create_translation_matrix(self.link_lengths[-1])
        link_location = link_location @ link_end
        return link_location @ [0, 0, 1]
