import numpy as np

class MatrixHelp: 
    def __init__(self) -> None:
        pass
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
