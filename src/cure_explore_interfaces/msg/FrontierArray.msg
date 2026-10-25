# FrontierArray.msg
# 表示所有前沿簇的数组

std_msgs/Header header
FrontierCluster[] clusters        # 所有前沿簇
int32 total_clusters              # 簇总数
int32 total_frontier_points       # 前沿点总数
