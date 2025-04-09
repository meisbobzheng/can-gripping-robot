# Legacy code to take a look at

def main():

    # These will be an input from the camera telling us where the can is
    x_pos = 0.5
    y_pos = 0.1
    z_pos = 0.2

    first_x_pos, first_y_pos = calculate_first_pos(x_pos, y_pos)

    bot = InterbotixManipulatorXS(
        robot_model="wx250",
        group_name="arm",
        gripper_name="gripper",
    )

    robot_startup()

    # wrist angle to look up at - maybe calculate, maybe can just cheat and hard code
    wrist_rotate_angle = math.pi / 4

    wrist_angle = math.pi / 5

    bot.arm.go_to_home_pose()

    # bot.arm.set_ee_cartesian_trajectory(x_pos, y_pos, z_pos)
    bot.arm.set_ee_pose_components(x=first_x_pos, y=first_y_pos, z=z_pos)

    final_pos = bot.arm.set_ee_pose_components(x=x_pos, y=y_pos, z=z_pos, execute=False)
    # bot.arm.set_single_joint_position("wrist_rotate", wrist_rotate_angle)
    # bot.arm.set_single_joint_position("wrist_angle", wrist_angle)

    # curr_joints = list(bot.arm.get_joint_commands())
    curr_joints = final_pos[0]

    # adjust shoulder and elbow to move end effector straight down
    shoulder_index = bot.arm.info_index_map["shoulder"]
    elbow_index = bot.arm.info_index_map["elbow"]
    wrist_rotate_index = bot.arm.info_index_map["wrist_rotate"]
    wrist_angle_index = bot.arm.info_index_map["wrist_angle"]

    curr_joints[shoulder_index] = math.pi / 4.4
    # curr_joints[elbow_index] = math.pi / 4
    curr_joints[wrist_rotate_index] = wrist_rotate_angle
    curr_joints[wrist_angle_index] = wrist_angle

    bot.arm.set_joint_positions(curr_joints)

    # joint_angles = bot.arm.set_ee_pose_components(x=x_pos, y=y_pos, z=z_pos)
    # print(joint_angles[0])wrist_rotate
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()

    robot_shutdown()