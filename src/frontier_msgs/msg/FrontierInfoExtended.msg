# FrontierInfoExtended.msg
int32 id
float64[2] position  # [x, y] 前沿中心坐标
float32 semantic_score  # 语义得分
float64[] distances_to_robots  # 到每个机器人的距离
string[] robot_names           # 机器人名称列表
float32 risk_level
float32 exploration_gain