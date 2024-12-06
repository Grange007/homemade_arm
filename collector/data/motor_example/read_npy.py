
import numpy as np

# 定义 .npy 文件的路径
file_path = './1732784613753.npy'  # 替换为你的 .npy 文件路径

# 读取 .npy 文件
data = np.load(file_path, allow_pickle=True).item()

# 打印读取的数据
print("Loaded data:")
for key, value in data.items():
    print(f"{key}: {value}")