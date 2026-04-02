import rclpy, math
from rclpy.node import Node
import numpy as np
from sklearn.linear_model import LinearRegression
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from wall_bot_msgs.msg import WallInfo


class wall_detector(Node):

    def __init__(self):
        super().__init__('wall_detector')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('right_range_min', 0.3)
        self.declare_parameter('right_range_max', 1.0)
        self.declare_parameter('front_range_max', 1.5)
        self.declare_parameter('front_range_min', 0.0)
        self.declare_parameter('angle_increment', 1)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.right_range_min = self.get_parameter('right_range_min').value
        self.right_range_max = self.get_parameter('right_range_max').value
        self.front_range_max = self.get_parameter('front_range_max').value
        self.front_range_min = self.get_parameter('front_range_min').value
        self.angle_increment = self.get_parameter('angle_increment').value

        # WallInfo 퍼블리셔
        self.pub_wall_info = self.create_publisher(WallInfo, '/wall/info', 20)

        # /scan 구독
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.right_detecter, 10
        )

        # RViz 용 marker
        self.marker_pub = self.create_publisher(MarkerArray, '/scan_marker', 10)

    

    def angle_to_index(self, angle, msg):
        return int((angle - msg.angle_min) / msg.angle_increment)

    def marker(self, msg):
        points_marker = Marker()
        points_marker.header.frame_id = msg.header.frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "raw_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD

        points_marker.scale.x = 0.05
        points_marker.scale.y = 0.05

        points_marker.color.r = 0.0
        points_marker.color.g = 0.0
        points_marker.color.b = 1.0
        points_marker.color.a = 1.0
        return points_marker

    def front_detect(self, msg: LaserScan):
        """
        전면 -15° ~ +15° 구간의 최소 거리 계산
        """
        front_left_angle  = math.radians(15)   # +15 deg
        front_right_angle = math.radians(-15)  # -15 deg

        front_idx_left  = self.angle_to_index(front_left_angle, msg)
        front_idx_right = self.angle_to_index(front_right_angle, msg)

        front_start_idx = max(0, min(front_idx_left, front_idx_right))
        front_end_idx   = min(len(msg.ranges) - 1, max(front_idx_left, front_idx_right))

        front_rmin = max(self.front_range_min, msg.range_min)
        front_rmax = min(self.front_range_max, msg.range_max)

        front_range = [] #전면 유효 거리 값 판단 갯수 3 ~ 5개로 판단하게 만들기 수정 필요.
        for i in range(front_start_idx, front_end_idx + 1):
            r = msg.ranges[i]
            if not math.isfinite(r):
                continue
            if not (front_rmin <= r <= front_rmax):
                continue
            front_range.append(r)

        if len(front_range) == 0:
            return None
        return np.min(front_range)

    def right_detecter(self, msg: LaserScan):
        """
        오른쪽 벽 검출 + 회귀 + front 거리까지 계산
        """
        points_marker = self.marker(msg)
        marker_array = MarkerArray()

        # ---- WallInfo 메시지 준비 ----
        wall_info = WallInfo()

        # -------- 1) 오른쪽 벽 포인트 추출 --------
        right_far_angle  = math.radians(-15)   # right-front
        right_near_angle = math.radians(-90)  # right 뒤쪽

        right_idx_far  = self.angle_to_index(right_far_angle, msg)
        right_idx_near = self.angle_to_index(right_near_angle, msg)

        right_start_idx = max(0, min(right_idx_far, right_idx_near))
        right_end_idx   = min(len(msg.ranges) - 1, max(right_idx_far, right_idx_near))

        right_rmin = max(self.right_range_min, msg.range_min)
        right_rmax = min(self.right_range_max, msg.range_max)

        x_array = []
        y_array = []
        for i in range(right_start_idx, right_end_idx + 1):
            if i >= len(msg.ranges):
                break

            r = msg.ranges[i]
            if not math.isfinite(r):
                continue
            if not (right_rmin <= r <= right_rmax):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            x_array.append(x)
            y_array.append(y)

            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            points_marker.points.append(p)
        

        vaild_point_count = len(x_array)
        wall_info.valid_point_count = vaild_point_count

        # -------- 2) 코너 / 벽 상실 판단 --------
        if len(x_array) < 5:
            self.get_logger().warn(
                f"[WallDetector] Not enough points for regression (N={len(x_array)})"
            )
            # 포인트 부족 → 코너/벽 없음으로 간주
            corner = True
            wall_info.corner = True 
            wall_info.right_distance = -1.0
            wall_info.right_degrees = 0.0

            # 전면 거리도 같이 세팅해 주자
            front_d = self.front_detect(msg)
            if front_d is None:
                wall_info.front_dist = -1.0
            else:
                wall_info.front_dist = float(front_d)

            # WallInfo publish
            self.pub_wall_info.publish(wall_info)

            # marker publish
            marker_array.markers.append(points_marker)
            self.marker_pub.publish(marker_array)
            return
        
        front_d = self.front_detect(msg)
        if front_d is None:
            wall_info.front_dist = -1.0
        else:
            wall_info.front_dist = float(front_d)

        # 여기까지 왔으면: 포인트 충분, 코너 아님
        wall_info.corner = False

        # -------- 3) Linear Regression으로 벽 기울기 계산 --------
        x = np.asarray(x_array)
        y = np.asarray(y_array)
        degrees_input = x.reshape(-1, 1)

        right_regressor = LinearRegression().fit(degrees_input, y)
        slope = float(right_regressor.coef_[0])       # 기울기 m
        intercept = float(right_regressor.intercept_) # 절편 b

        slope_degrees = math.atan(slope) #벽의 기울기의 각도 구하기(rad)
        
        distance_to_wall = abs(intercept) / math.sqrt(1.0 + slope * slope) #점과 직선 사이의 거리 공식 이용(단위:m)

        
        wall_info.right_distance = float(distance_to_wall)
        wall_info.right_degrees    = float(slope_degrees)

        self.get_logger().info(
            f"[WallDetector] 인식한 포인터 개수={len(x_array)}, 거리={distance_to_wall:.3f}m, 벽의 각도={slope_degrees:.1f}rad, 전면 거리={front_d}m")
        
        # -------- 4) 회귀 직선 마커 --------
        x1 = float(min(x_array) - 0.2)
        x2 = float(max(x_array) + 0.2)
        y1 = slope * x1 + intercept
        y2 = slope * x2 + intercept

        line_marker = Marker()
        line_marker.header.frame_id = msg.header.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "wall_fit_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.03
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        p1 = Point(); p1.x = x1; p1.y = y1; p1.z = 0.0
        p2 = Point(); p2.x = x2; p2.y = y2; p2.z = 0.0
        line_marker.points.append(p1)
        line_marker.points.append(p2)

        # -------- 5) 전면 거리 먼저 계산해서 넣어두기 --------
        front_d = self.front_detect(msg)
        if front_d is None:
            wall_info.front_dist = -1.0
        else:
            wall_info.front_dist = float(front_d)



        # -------- 6) WallInfo + Marker 퍼블리시 --------
        self.pub_wall_info.publish(wall_info)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = wall_detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
