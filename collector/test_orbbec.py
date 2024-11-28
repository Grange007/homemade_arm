import numpy as np
import os
from easyrobot.camera.orbbec import OrbbecRGBDCamera
import pyorbbecsdk as orbsdk
import time

# config = rs.config()
# pipeline = rs.pipeline()
# config.enable_device("147122072860")
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
# pipeline_profile = pipeline.start(config)

camera = OrbbecRGBDCamera(
    frame_rate=15,
    resolution=(1280, 720),
    logger_name="Orbbec RGBD Camera",
    shm_name_rgb=None,
    shm_name_depth=None,
    streaming_freq=15
)

output_dir = "depth_data"
os.makedirs(output_dir, exist_ok=True)

frame_count = 0

while True:
    [color, depth] = camera.get_info()
    print(color.shape)
    print(depth.shape)

