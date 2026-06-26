# Wall_bot msgs

#wall_info
int32    valid_point_count # 오른쪽 벽에 닿은 포인터 수
float32 right_distance   # 오른쪽 벽까지 거리 [m]
float32 right_degrees      # 오른쪽 벽 기울기의 각도 [rad]  (atan(slope) 결과)
float32 front_dist       # 전면 최소 거리 [m], 없으면 -1.0
bool    corner           # 코너/벽 상실 여부

