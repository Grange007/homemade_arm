import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

# 定义文件夹路径
folder_path = '/home/grange/Program/act_arm/collector/data/task1260145/scene1'

# 获取所有 numpy 文件
numpy_files = [f for f in os.listdir(folder_path) if f.endswith('.npy')]

# 创建输出文件夹
output_folder = 'output_videos_and_plots'
os.makedirs(output_folder, exist_ok=True)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter(os.path.join(output_folder, 'combined_video.avi'), fourcc, 20.0, (1280 * 2, 720 * 2))


unitree_encoder = []
cybergear_encoder = []
servo_encoder = []


numpy_files.sort()
for numpy_file in numpy_files:
    # 加载 numpy 文件
    data = np.load(os.path.join(folder_path, numpy_file), allow_pickle=True).item()

    # 提取图像和深度数据
    image = data['image']
    depth = data['depth']
    image_orbbec = data['image_orbbec']
    depth_orbbec = data['depth_orbbec']

    # 提取编码器数据
    unitree_encoder.append(data['Unitree_encoder'])
    cybergear_encoder.append(data['Cybergear_encoder'])
    servo_encoder.append(data['servo_encoder'])

    depth_frame = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_frame = cv2.cvtColor(depth_frame.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    # 处理 Orbbec 深度数据
    depth_orbbec_frame = cv2.normalize(depth_orbbec, None, 0, 255, cv2.NORM_MINMAX)
    depth_orbbec_frame = cv2.cvtColor(depth_orbbec_frame.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    depth_frame
    
    # 拼接图像和深度数据
    top_row = np.hstack((image, depth_frame))
    bottom_row = np.hstack((image_orbbec, depth_orbbec_frame))
    bottom_row = cv2.resize(bottom_row, (top_row.shape[1], top_row.shape[0]))
    combined_frame = np.vstack((top_row, bottom_row))

    # 写入视频
    video_writer.write(combined_frame)
    cv2.imshow('Combined Video', combined_frame)
    if cv2.waitKey(2) & 0xFF == ord('q'):
        break

video_writer.release()
cv2.destroyAllWindows()

    # 绘制编码器数据曲线
plt.figure()
plt.plot(unitree_encoder, label='Unitree Encoder')
plt.legend()
plt.title(f'Unitree Encoder Data') 
plt.figure()
plt.plot(cybergear_encoder, label='Cybergear Encoder')
plt.legend()
plt.title(f'Cybergear Encoder Data') 
plt.figure()
plt.plot(servo_encoder, label='Servo Encoder')
plt.legend()
plt.title(f'Servo Encoder Data')
plt.show()