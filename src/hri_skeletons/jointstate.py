
HUMAN_JOINT_NAMES = ["waist",
        "head_r", 
        "head_y",
        "head_p",
        "l_y_shoulder",
        "l_p_shoulder",
        "l_r_shoulder",
        "l_elbow",
        "r_y_shoulder",
        "r_p_shoulder",
        "r_r_shoulder",
        "r_elbow",
        "l_r_hip",
        "l_p_hip",
        "l_knee",
        "r_r_hip",
        "r_p_hip",
        "r_knee"]

def compute_jointstate(ik_chains, pose_3d):
    r_arm, l_arm, r_leg, l_leg = ik_chains

    target = [] #TODO from pose_3d -> 3d pose wrist - 3d pose shoulder
    r_arm.inverse_kinematics(target)
    return [0.] * len(HUMAN_JOINT_NAMES)



# To plot with ikpy:
#
# import matplotlib.pyplot as plt
# import ikpy.utils.plot as plot_utils
# 
# target=[0.2,-0.2,0.1];fig, ax = plot_utils.init_3d_figure()
# print(my_chain.inverse_kinematics(target))
# my_chain.plot(my_chain.inverse_kinematics(target), ax, target=target)
# plt.xlim(-0.4, 0.4)
# plt.ylim(-0.4, 0.4)
# ax.set_zlim(-0.4,0.4)
# 
# plt.show()

