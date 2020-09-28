
HUMAN_JOINT_NAMES = ["waist",
        "head_r", 
        "head_y",
        "head_p",
        "l_y_shoulder",
        "l_p_shoulder",
        "l_elbow",
        "r_y_shoulder",
        "r_p_shoulder",
        "r_elbow",
        "l_r_hip",
        "l_p_hip",
        "l_knee",
        "r_r_hip",
        "r_p_hip",
        "r_knee"]

def compute_jointstate(pose_3d):
    return [0.] * len(HUMAN_JOINT_NAMES)
