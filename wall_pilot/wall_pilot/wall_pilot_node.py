import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from wall_bot_msgs.msg import WallInfo
import numpy as np


class wallPilot(Node):

    def __init__(self):
        super().__init__('wall_pilot')
        self.declare_parameter('cmd_vel', '/wall_bot/cmd_vel')

        self.cmd_vel = self.get_parameter('cmd_vel').value

        # wall_detector에서 받아올 값들
        self.vaild_point_count = None     # 오른쪽 벽에 닿은 포인터 수
        self.right_distance = None        # 오른쪽 벽까지의 거리 [m]
        self.right_degrees = None         # 오른쪽 벽의 방향각 [rad]
        self.front_dist = None            # 전면 최소 거리 [m]
        self.corner = None                # 코너 진입 여부 (True/False)

        # 제어 타이머 (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # wall_detector → wall.info subscribe
        self.sub_wall_info = self.create_subscription(
            WallInfo, '/wall/info', self.wall_cb, 20
        )

        # cmd_vel publisher
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel, 10)

        # ---------------- 제어 gains ----------------
        self.DESIRED_DIST = 0.7   # 유지할 벽 거리 [m]
        self.K_D = -0.8           # 거리 오차 제어 gain
        self.K_TH = 0.5           # 벽 각도 제어 gain
        self.K_FD = 0.5           # 전방 거리 제어 gain
        self.W_LIMIT = 1.2        # 회전 속도 제한 [rad/s]
        self.FWD_SPEED = 0.5      # 기본 전진 속도
        self.front_slow = 1.5     # 전방 감속 시작 거리
        self.front_safety = 0.7   # 안전 정지 거리

    # ----------- 콜백 -----------
    def wall_cb(self, msg: WallInfo):
        self.valid_point_count = msg.valid_point_count
        self.right_distance = msg.right_distance
        self.right_degrees = msg.right_degrees
        self.front_dist = msg.front_dist
        self.corner = msg.corner

    # ----------- 각도 기반 속도 비율 계산 함수 -----------
    def angle_normalize(self, angle):
        wall_angle = self.right_degrees * (180.0 / math.pi)
        degree_max = 68.0

        # -68 ~ 68도 제한
        degree = max(-degree_max, min(wall_angle, degree_max))

        # 각도 절댓값이 클수록 비율 감소 (1 → 0)
        degree_speed_ratio = 1 - abs(degree) / degree_max

        return degree_speed_ratio

    # ----------- 벽 신뢰도 -----------
    def wall_confidence(self):
        if self.valid_point_count is None:
            return 0.0

        conf = self.valid_point_count / 30.0  # 30개 이상이면 1.0
        return max(0.0, min(conf, 1.0))

    # ----------- 전방 거리 기반 속도 비율 -----------
    def front_factor_ratio(self):
        d = self.front_dist
        d_stop = self.front_safety
        d_slow = self.front_slow

        if d is None or d < 0:
            return 1.0

        if d <= d_stop:
            return 0.0

        expression = (d - d_stop) / (d_slow - d_stop)
        clamped_value = np.clip(expression, 0.0, 1.0)
        return clamped_value#max(0.0, min(expression, 1.0))

    # ----------- 메인 제어 루프 -----------
    def control_loop(self):
        cmd = Twist()

        # 데이터 없으면 정지
        if self.right_distance is None or self.right_degrees is None:
            self.get_logger().warn("[WallPilot] waiting for wall data...")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        front_d = self.front_dist if self.front_dist is not None else -1.0

        # 거리 오차
        distance_gap = self.right_distance - self.DESIRED_DIST

        # 회전 제어
        dist_factor = self.K_D * distance_gap * abs(distance_gap)
        angle_factor = self.K_TH * self.right_degrees
        angle_control = dist_factor + angle_factor

        # 전방 거리 제어
        front_factor = self.front_factor_ratio()

        # 각도 기반 속도 감소
        angle_speed_ratio = self.angle_normalize(self.right_degrees)
        angle_speed_ratio = 0.3 + angle_speed_ratio * 0.7

        # 벽 길이 기반 속도 감소
        wall_len_ratio = self.wall_confidence()

        # ------------- 코너 모드 -------------

        R_target = 0.35
        corner_mode = bool(self.corner) if self.corner is not None else False

        if corner_mode:
            corner_speed_ratio = 0.5 * front_factor + 0.5 * wall_len_ratio

            cmd.linear.x = self.FWD_SPEED * corner_speed_ratio
            cmd.angular.z = -cmd.linear.x / R_target

            self.get_logger().info("[WallPilot] corner mode: searching for wall...")
            self.get_logger().info(
                f"각속도 = {math.degrees(cmd.angular.z):.2f}, "
                f"선속도 = {cmd.linear.x:.2f}, "
                f"전면 거리 = {self.front_dist:.2f}, "
                f"벽 길이 비율 = {wall_len_ratio:.2f}"
            )
            self.pub_cmd.publish(cmd)
            return

        # ----------- 전방 위험 상황: 제자리 회전 -----------
        if front_factor <= 0.1:
            angle_control *= 1.4
            cmd.angular.z = angle_control
            cmd.linear.x = 0.0
            self.pub_cmd.publish(cmd)
            return

        # 로그 출력
        self.get_logger().info(
            f"거리 차이={distance_gap:.2f}, 거리제어={dist_factor:.2f}, "
            f"각도 제어={angle_factor:.2f}, 전방 거리={front_d:.2f}, "
            f"총 회전 제어={angle_control:.2f}, 전방 비율={front_factor:.2f}, "
            f"각도 속도 비율={angle_speed_ratio:.2f}"
        )

        # 최종 속도 결정
        speed_ratio = 0.6 * front_factor + 0.4 * angle_speed_ratio
        fwd = self.FWD_SPEED * speed_ratio

        # 회전 제한
        angle_control = max(-self.W_LIMIT, min(angle_control, self.W_LIMIT))

        cmd.linear.x = fwd
        cmd.angular.z = angle_control

        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"각속도 = {math.degrees(angle_control):.2f}, 선속도 = {fwd:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    WallPilot = wallPilot()
    rclpy.spin(WallPilot)
    WallPilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
