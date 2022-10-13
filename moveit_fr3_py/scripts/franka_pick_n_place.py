import copy
import sys

import actionlib
import moveit_commander
import moveit_msgs.msg
import rospy
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from franka_gripper.msg import GraspAction, GraspGoal


class FrankaPickNPlace(object):
    def __init__(self):
        super(FrankaPickNPlace, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("franka_pick_n_place", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # robot needs to start from home pose
        self.home_pose = self.move_group.get_current_pose().pose

        # set waypoint poses
        _left_pose = self.move_group.get_current_pose().pose
        _left_pose.position.x = 0.407464
        _left_pose.position.y = -0.276469
        _left_pose.position.z = 0.11

        _left_hover_pose = self.move_group.get_current_pose().pose
        _left_hover_pose.position.x = 0.407464
        _left_hover_pose.position.y = -0.276469

        _right_pose = self.move_group.get_current_pose().pose
        _right_pose.position.x = 0.526153
        _right_pose.position.y = 0.556913
        _right_pose.position.z = 0.11

        _right_hover_pose = self.move_group.get_current_pose().pose
        _right_hover_pose.position.x = 0.526153
        _right_hover_pose.position.y = 0.556913

        self.left_pose = copy.deepcopy(_left_pose)
        self.left_hover_pose = copy.deepcopy(_left_hover_pose)
        self.right_pose = copy.deepcopy(_right_pose)
        self.right_hover_pose = copy.deepcopy(_right_hover_pose)

        self.gripper_action_client = actionlib.SimpleActionClient(
            "/franka_gripper/gripper_action",
            GripperCommandAction,
        )
        self.gripper_action_client.wait_for_server()

        self.gripper_grasp_client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp",
            GraspAction,
        )
        self.gripper_grasp_client.wait_for_server()

    def go_down_left(self):
        waypoints = [self.left_hover_pose, self.left_pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)

    def go_down_right(self):
        waypoints = [self.right_hover_pose, self.right_pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)

    def home_from_left(self):
        waypoints = [self.left_hover_pose, self.home_pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)

    def home_from_right(self):
        waypoints = [self.right_hover_pose, self.home_pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)

    def gripper_close(self):
        goal = GraspGoal()
        goal.width = 0.0001
        goal.speed = 0.7
        goal.force = 10
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.07

        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result(rospy.Duration.from_sec(10.0))

    def gripper_open(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.035
        goal.command.max_effort = 0.0

        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result(rospy.Duration.from_sec(10.0))


def main():
    try:
        task = FrankaPickNPlace()

        # left-side pick-n-place routine
        task.gripper_open()
        task.go_down_left()
        task.gripper_close()
        task.home_from_left()
        task.go_down_left()
        task.gripper_open()
        task.home_from_left()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
