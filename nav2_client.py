##실행하실 때 python3 /home/pinky/nav2_client.py 0.5 0.5 0 이거 치시면 이동합니다
##python3 /home/pinky/nav2_client.py x좌표 y좌표 핑키가바라보는방향
##아직 문제가 많아 해결해야 할 부분이 많습니다


#!/usr/bin/env python3

import argparse
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2Client(Node):

    def __init__(self):
        super().__init__("nav2_client")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

    def go_to_pose(self, x, y, yaw_degree):
        print("Nav2 서버 연결 확인 중...")

        if not self.client.wait_for_server(timeout_sec=10.0):
            print("오류: /navigate_to_pose 서버를 찾지 못했습니다.")
            return False

        goal = NavigateToPose.Goal()

        # 목표 좌표는 map 좌표계 기준
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        # degree를 quaternion으로 변환
        yaw_radian = math.radians(float(yaw_degree))

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw_radian / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_radian / 2.0)

        print(
            f"목표 전송: "
            f"x={x:.2f} m, "
            f"y={y:.2f} m, "
            f"방향={yaw_degree:.1f}°"
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            print("오류: 목표 전송 결과를 받지 못했습니다.")
            return False

        if not goal_handle.accepted:
            print("목표가 거절되었습니다.")
            return False

        print("목표 승인. 이동을 시작합니다.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            print("오류: 이동 결과를 받지 못했습니다.")
            return False

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            print("목표 지점에 도착했습니다.")
            return True

        print(f"이동 실패 또는 중단: 상태 코드 {wrapped_result.status}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Pinky Nav2 좌표 이동 프로그램"
    )

    parser.add_argument("x", type=float, help="목표 x 좌표 [m]")
    parser.add_argument("y", type=float, help="목표 y 좌표 [m]")
    parser.add_argument("yaw", type=float, help="도착 방향 [degree]")

    args = parser.parse_args()

    rclpy.init()

    node = Nav2Client()

    try:
        success = node.go_to_pose(
            args.x,
            args.y,
            args.yaw
        )

    except KeyboardInterrupt:
        print("\n사용자가 프로그램을 중단했습니다.")
        success = False

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
