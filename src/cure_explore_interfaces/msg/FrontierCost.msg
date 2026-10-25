# FrontierCost.msg
# 表示单个前沿簇到所有机器人的代价信息

int32 frontier_id             # 前沿簇ID
geometry_msgs/Point centroid  # 前沿簇质心位置
CostInfo[] costs              # 到各个机器人的代价数组
