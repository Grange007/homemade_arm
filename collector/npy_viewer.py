import numpy as np

# 加载 .npy 文件
file_path = './data/task0/scene1/1732768433407.npy'
data = np.load(file_path, allow_pickle=True)

# 打印数据
print(data)

# 如果数据是字典，可以打印键和值
if isinstance(data, dict):
    for key, value in data.items():
        print(f"{key}: {value}")