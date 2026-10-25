# SemanticMap.msg
# 表示当前已发现的所有语义对象

std_msgs/Header header
SemanticObject[] objects
int32 total_objects  # 总共有多少语义对象（包括未发现的）
int32 discovered_count  # 已发现多少
