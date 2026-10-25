# CostInfo.msg
# 表示从单个前沿点到单个机器人的代价信息

int32 robot_id                # 机器人ID
float32 euclidean_distance    # 欧式距离（米）
float32 astar_distance        # A*路径距离（米，-1表示无法到达）
float32 astar_cost            # A*路径代价（考虑障碍物等，-1表示无法到达）
