import forward_kinematics_live
import jacobian_IK_live

# X offset from middle of palm to link
testhand = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]], "offset": [.029, 0, 0]},
            "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]], "offset": [-.029, 0, 0]}}


# FK is just a check
#fk1 = forward_kinematics_live.ForwardKinematicsLIVE(1, testhand["finger1"])

# TODO: Create an IK for each finger
# 0,0 is center of the palm
# y is out 
# x is negative to left, pos right
ik1 = jacobian_IK_live.JacobianIKLIVE(1, testhand["finger1"])
#fk1.set_joint_angles([-1.5708, 1.5708])
#print(fk1.calculate_forward_kinematics())
# TODO: pass in goal location (but as small step between contact point and ultimate target position), keegan sent over func
_, angles, _= ik1.calculate_ik([.101, .07199974], ee_location=None) # add end effector location as vector
print(angles)
# TODO: send angles to motors
# IN calculate IK optional para


#fk1.set_joint_angles(angles)

#print(fk1.calculate_forward_kinematics())
