import numpy as np

from easyrobot.camera.realsense import RealSenseRGBDCamera
import pyrealsense2 as rs

context = rs.context()
connected_devices = [d.get_info(rs.camera_info.serial_number) for d in context.devices]
print("Connected devices:", connected_devices)

# config = rs.config()
# pipeline = rs.pipeline()
# config.enable_device("147122072860")
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
# pipeline_profile = pipeline.start(config)

camera = RealSenseRGBDCamera(
    serial="147122072860",
    frame_rate=15,
    resolution=(640, 480),
    enable_emitter=False,
    align=False,
    streaming_freq=10
)

while True:
    [rgb, depth] = camera.get_info()
    print(rgb.shape, depth.shape)

