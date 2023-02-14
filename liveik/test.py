import forward_kinematics_live
import jacobian_IK_live


testhand = {"finger1": {"name": "finger0", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]], "offset": [.029, 0, 0]},
            "finger2": {"name": "finger1", "num_links": 2, "link_lengths": [[0, .072, 0], [0, .072, 0]]}, "offset": [-.029, 0, 0]}


fk1 = forward_kinematics_live.ForwardKinematicsLIVE(1, testhand["finger1"])
ik1 = jacobian_IK_live.JacobianIKLIVE(1, testhand["finger1"])
#fk1.set_joint_angles([-1.5708, 1.5708])
print(fk1.calculate_forward_kinematics())
_, angles, _= ik1.calculate_ik([.101, .07199974])
print(angles)
fk1.set_joint_angles(angles)

print(fk1.calculate_forward_kinematics())
