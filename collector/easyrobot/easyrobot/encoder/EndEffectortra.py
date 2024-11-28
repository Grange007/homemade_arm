import numpy as np

# 定义 norm_stats.npy 文件的路径
norm_stats_path = './stats/gather_balls/norm_stats.npy'

# 读取 norm_stats.npy 文件
norm_stats = np.load(norm_stats_path, allow_pickle=True).item()

# 访问均值和标准差数据
mean = norm_stats['mean']
std = norm_stats['std']

print(f"Mean: {mean}")
print(f"Standard Deviation: {std}")