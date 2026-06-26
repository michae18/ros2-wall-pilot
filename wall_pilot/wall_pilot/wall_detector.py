import rclpy, math
from rclpy.node import Node
import numpy as np
from sklearn.linear_model import LinearRegression
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32, Int8
from geometry_msgs.msg import Twist,Point
from visualization_msgs.msg import Marker, MarkerArray


class wallPilot(Node):


    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter('scan_topic','/scan')
        self.declare_parameter('cmd_vel','/wall_bot/cmd_vel')

        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 1.0)
        self.declare_parameter('angle_increment',1)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel = self.get_parameter('cmd_vel').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_increment = self.get_parameter('angle_increment').value


        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback,10)
        self.pub_cmd = self.create_publisher(Twist,self.cmd_vel, 10)
        
        self.marker_pub = self.create_publisher(MarkerArray,'/scan_marker', 10)

        # ---------------- 제어 gains ----------------
        # 오른쪽 벽 추종용: 거리 오차 + 방향각 오차 → angular.z
        self.DESIRED_DIST = 0.7      # 벽과 유지하고 싶은 거리 [m]
        self.K_D = -1              # 거리 오차 제어 gain (오른쪽 벽 기준, 부호 주의)
        self.K_TH = 1             # 벽 방향각 제어 gain
        self.W_LIMIT = 1.2           # 회전 속도 제한 [rad/s]
        self.FWD_SPEED = 0.2         # 기본 전진 속도 [m/s]

    def angle_to_index(self, angle, msg):
        return int((angle - msg.angle_min) / msg.angle_increment)


    def scan_callback(self, msg: LaserScan):
        # right_x, right_y = self.right_point_num(msg)
        # # left_x, left_y= self.left_point_num(msg)
        
        points_marker = Marker()
        points_marker.header.frame_id = msg.header.frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "raw_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD

        points_marker.scale.x = 0.05
        points_marker.scale.y = 0.05

        # 파란색 점
        points_marker.color.r = 0.0
        points_marker.color.g = 0.0
        points_marker.color.b = 1.0
        points_marker.color.a = 1.0


        right_far_angle  = math.radians(0)  # 전방 쪽
        right_near_angle = math.radians(-90)  # 완전 오른쪽

        right_idx_far  = self.angle_to_index(right_far_angle, msg)
        right_idx_near = self.angle_to_index(right_near_angle, msg)

        right_start_idx = min(right_idx_far, right_idx_near)
        right_end_idx   = max(right_idx_far, right_idx_near)

        right_rmin = max(self.range_min, msg.range_min)
        right_rmax = min(self.range_max, msg.range_max)
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

        marker_array = MarkerArray()

        if len(x_array) < 2:
            self.get_logger().warn(
                f"Not enough valid points for regression (N={len(x_array)})"
            )
            # cmd = Twist()
            # cmd.linear.x = 0.1
            # cmd.angular.z = -1.0
            marker_array.markers.append(points_marker)
            # self.pub_cmd.publish(cmd)
            self.marker_pub.publish(marker_array)
            return
        
        
         # -----------------------------
        # 3) Linear Regression으로 벽 기울기 계산  오른쪽 보다 왼쪽의 포인수 수가 많을 경우 왼쪽 기울기 계산.
        # -----------------------------
        x = np.asarray(x_array)
        y = np.asarray(y_array)
        degrees_input = x.reshape(-1, 1)
        right_regressor = LinearRegression().fit(degrees_input,y)
        right_wall_degree = float(right_regressor.coef_[0]) # 벽에 닿은 포인터들의 기울기
        intercept = float(right_regressor.intercept_) # 절편
        
        right_wall_direction = math.atan(right_wall_degree) # 벽의 방향각:rad

        distance_to_wall = abs(intercept) / math.sqrt(1.0 + right_wall_degree * right_wall_degree) # 오른쪽 벽과 로봇의 거리

        distance_error= distance_to_wall - self.DESIRED_DIST  # 벽과의 거리 제어
        #양수: 너무 많이 떨어져 있음, 음수: 너무 가까움.

        self.get_logger().info(
            f"포인트 수={len(x_array)}, 기울기={right_wall_degree:.2f}, "
            f"벽과의 거리={distance_to_wall:.2f}"
        )
        # if right_wall_degree< 0 :
        #     right_wall_degree += math.pi
        # right_wall_normal =(right_wall_direction + math.pi/2) #벽의 법선 각(벽과 수직인 각도:rad)
        # right_wall_with_bot = abs(math.degrees(right_wall_normal) - 90)

        #회귀 직선 -  marker

        x1 = float(min(x_array) - 0.2)
        x2 = float(max(x_array) + 0.2)
        y1 = right_wall_degree * x1 + intercept
        y2 = right_wall_degree * x2 + intercept

        line_marker = Marker()
        line_marker.header.frame_id = msg.header.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "wall_fit_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.03  # 선 두께

        # 초록색 선
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        # 직선을 위해 x축 양 끝 두 점 선택
        
        p1 = Point()
        p1.x = x1
        p1.y = y1
        p1.z = 0.0

        p2 = Point()
        p2.x = x2
        p2.y = y2
        p2.z = 0.0

        line_marker.points.append(p1)
        line_marker.points.append(p2)

        # ---------------- MarkerArray 퍼블리시 ----------------
        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)
        self.marker_pub.publish(marker_array)

        w_dist = self.K_D * distance_error
        # distance_error = min(distance_error, 0.3)
        # distance_error = max(distance_error, -0.3)

        w_theta = self.K_TH * right_wall_direction

        # right_wall_degree = min(right_wall_degree, 1.0)
        # right_wall_degree = max(right_wall_degree, -1.0)
        w = w_dist + w_theta
        self.get_logger().info(
            f"거리 오차={w_dist:.2f}, 벽 각도 제어={w_theta:.2f}, "
            f"회전 제어={w:.2f}"
        )
        if len(x_array) <= 10:
            self.FWD_SPEED = 0.1
            w = -1.0
        else:
            if w > self.W_LIMIT:
                w = self.W_LIMIT

            elif w < -self.W_LIMIT:
                w = -self.W_LIMIT    
        

        cmd = Twist()
        cmd.linear.x = self.FWD_SPEED
        cmd.angular.z = w

        self.pub_cmd.publish(cmd)
        self.get_logger().info(f"각/속도 = {w:.2f} / {self.FWD_SPEED}")

        self.marker_pub.publish(marker_array)
      

def main(args=None):
    rclpy.init(args=args)

    WallPilot = wallPilot()

    rclpy.spin(WallPilot)

    WallPilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()