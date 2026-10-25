# FrontierCluster.msg
# 表示单个前沿簇的信息（带语义系数）

int32 id                          # 簇ID
geometry_msgs/Point centroid      # 簇质心位置
int32 size                        # 簇包含的前沿点数量

# 归一化系数 [0.0, 1.0]
float32 info_gain                 # 信息增益 (归一化到0-1, 1.0=最大信息增益)
float32 structure_coefficient     # 结构系数 (0.5=中性, 1.0=高价值结构如出口, 0.0=低价值)
float32 hazard_coefficient        # 危险系数 (0.0=安全, 1.0=极度危险)
float32 objective_coefficient     # 目标系数 (1.0=最高优先级目标如受困人员, 0.0=非目标)

# 关联的语义对象信息（如果有）
int32 associated_semantic_id      # 关联的语义对象ID (-1表示无关联)
string associated_semantic_name   # 关联的语义对象名称
string associated_semantic_category  # 关联的语义对象类别

# 原始前沿点列表（用于恢复等功能）
geometry_msgs/Point[] frontier_points
