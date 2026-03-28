#!/usr/bin/env python3
"""
Navigation goal test for p3dx robots.

Usage:
    test_nav.py [namespace] [spawn_x] [spawn_y] [goal_x] [goal_y]

Defaults (single robot, warehouse world spawn → nearby goal):
    namespace = ''
    spawn     = (12.86, -9.48)
    goal      = (8.0,   -9.0)
"""
import sys
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

TIMEOUT_SEC = 90


def make_pose(navigator, x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def main():
    args = sys.argv[1:]
    namespace = args[0] if len(args) > 0 else ''
    spawn_x   = float(args[1]) if len(args) > 1 else 12.86
    spawn_y   = float(args[2]) if len(args) > 2 else -9.48
    goal_x    = float(args[3]) if len(args) > 3 else 8.0
    goal_y    = float(args[4]) if len(args) > 4 else -9.0

    label = namespace if namespace else 'robot'

    rclpy.init()
    nav = BasicNavigator(namespace=namespace)

    print(f'[{label}] Setting initial pose ({spawn_x}, {spawn_y})')
    nav.setInitialPose(make_pose(nav, spawn_x, spawn_y))

    print(f'[{label}] Waiting for Nav2...')
    nav.waitUntilNav2Active()

    print(f'[{label}] Sending goal ({goal_x}, {goal_y})')
    nav.goToPose(make_pose(nav, goal_x, goal_y))

    timeout = Duration(seconds=TIMEOUT_SEC)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and Duration.from_msg(feedback.navigation_time) > timeout:
            nav.cancelTask()
            print(f'[{label}] FAILED: timed out after {TIMEOUT_SEC}s', file=sys.stderr)
            sys.exit(1)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'[{label}] SUCCESS: reached goal ({goal_x}, {goal_y})')
        sys.exit(0)
    else:
        print(f'[{label}] FAILED: result={result}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
