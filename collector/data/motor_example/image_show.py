import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# 定义包含 .npy 文件的目录
directory = '../motor_example'  # 替换为 .npy 文件所在的目录

# 获取目录中的所有 .npy 文件
npy_files = sorted([f for f in os.listdir(directory) if f.endswith('.npy')])

# 读取所有 .npy 文件中的图像数据和编码器值
images = []
unitree_values = []
cyber_values = []
timestamps = []

for npy_file in npy_files:
    file_path = os.path.join(directory, npy_file)
    data = np.load(file_path, allow_pickle=True).item()
    
    image = data.get('image')
    unitree_encoder = data.get('unitree_encoder')
    cyber_encoder = data.get('cybergear_encoder')
    
    if image is not None:
        images.append(image)
    else:
        print(f"No 'image' key found in {npy_file}")
    
    if unitree_encoder is not None and cyber_encoder is not None:
        unitree_values.append(unitree_encoder)
        cyber_values.append(cyber_encoder)
        timestamps.append(int(npy_file.split('.')[0]))  # 假设文件名是时间戳

# 计算时间差（以第一个文件为时间0）
time_diffs = [(t - timestamps[0]) / 1000.0 for t in timestamps]  # 转换为秒

# 创建图像动画窗口
fig1, ax1 = plt.subplots(figsize=(10, 8))

def update_image(frame):
    ax1.clear()
    ax1.imshow(images[frame])
    ax1.set_title(npy_files[frame])
    ax1.axis('off')  # 关闭坐标轴

# 创建编码器值动画窗口
num_unitree_encoders = len(unitree_values[0])
num_cyber_encoders = len(cyber_values[0])

fig2, axs = plt.subplots(num_unitree_encoders + num_cyber_encoders, 1, figsize=(10, 8))

def update_encoders(frame):
    for i in range(num_unitree_encoders):
        axs[i].clear()
        axs[i].plot(time_diffs[:frame+1], [uv[i] for uv in unitree_values[:frame+1]], label=f'Unitree Encoder {i+1}')
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel(f'Unitree Encoder {i+1} Value')
        axs[i].set_ylim([np.min([uv[i] for uv in unitree_values]) , np.max([uv[i] for uv in unitree_values]) ])  # 调整 y 轴范围
        axs[i].legend()
        #axs[i].set_title(f'Unitree Encoder {i+1} Values Over Time')
    
    for j in range(num_cyber_encoders):
        axs[num_unitree_encoders + j].clear()
        axs[num_unitree_encoders + j].plot(time_diffs[:frame+1], [cv[j] for cv in cyber_values[:frame+1]], label=f'Cyber Encoder {j+1}')
        axs[num_unitree_encoders + j].set_xlabel('Time (s)')
        axs[num_unitree_encoders + j].set_ylabel(f'Cyber Encoder {j+1} Value')
        axs[num_unitree_encoders + j].set_ylim([np.min([cv[j] for cv in cyber_values]) , np.max([cv[j] for cv in cyber_values]) ])  # 调整 y 轴范围
        axs[num_unitree_encoders + j].legend()
       # axs[num_unitree_encoders + j].set_title(f'Cyber Encoder {j+1} Values Over Time')

# 创建动画
ani1 = animation.FuncAnimation(fig1, update_image, frames=len(images), interval=500)  # interval 以毫秒为单位
ani2 = animation.FuncAnimation(fig2, update_encoders, frames=len(images), interval=500)  # interval 以毫秒为单位

# 显示动画
plt.tight_layout()
plt.show()