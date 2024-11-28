'''
Orbbec Camera.

Author: Langzhe Gu
'''

import numpy as np
import pyorbbecsdk as orbsdk
import cv2
from easyrobot.camera.base import RGBCameraBase, RGBDCameraBase

from typing import Union, Any, Optional
MIN_DEPTH = 20  # 20mm
MAX_DEPTH = 10000  # 10000mm

def yuyv_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    yuyv = frame.reshape((height, width, 2))
    bgr_image = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
    return bgr_image


def uyvy_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    uyvy = frame.reshape((height, width, 2))
    bgr_image = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
    return bgr_image


def i420_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    y = frame[0:height, :]
    u = frame[height:height + height // 4].reshape(height // 2, width // 2)
    v = frame[height + height // 4:].reshape(height // 2, width // 2)
    yuv_image = cv2.merge([y, u, v])
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)
    return bgr_image


def nv21_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    y = frame[0:height, :]
    uv = frame[height:height + height // 2].reshape(height // 2, width)
    yuv_image = cv2.merge([y, uv])
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV21)
    return bgr_image


def nv12_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    y = frame[0:height, :]
    uv = frame[height:height + height // 2].reshape(height // 2, width)
    yuv_image = cv2.merge([y, uv])
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
    return bgr_image


def determine_convert_format(frame: orbsdk.VideoFrame):
    if frame.get_format() == orbsdk.OBFormat.I420:
        return orbsdk.OBConvertFormat.I420_TO_RGB888
    elif frame.get_format() == orbsdk.OBFormat.MJPG:
        return orbsdk.OBConvertFormat.MJPG_TO_RGB888
    elif frame.get_format() == orbsdk.OBFormat.YUYV:
        return orbsdk.OBConvertFormat.YUYV_TO_RGB888
    elif frame.get_format() == orbsdk.OBFormat.NV21:
        return orbsdk.OBConvertFormat.NV21_TO_RGB888
    elif frame.get_format() == orbsdk.OBFormat.NV12:
        return orbsdk.OBConvertFormat.NV12_TO_RGB888
    elif frame.get_format() == orbsdk.OBFormat.UYVY:
        return orbsdk.OBConvertFormat.UYVY_TO_RGB888
    else:
        return None

def frame_to_rgb_frame(frame: orbsdk.VideoFrame) -> Union[Optional[orbsdk.VideoFrame], Any]:
    if frame.get_format() == orbsdk.OBFormat.RGB:
        return frame
    convert_format = determine_convert_format(frame)
    if convert_format is None:
        print("Unsupported format")
        return None
    print("covert format: {}".format(convert_format))
    convert_filter = orbsdk.FormatConvertFilter()
    convert_filter.set_format_convert_format(convert_format)
    rgb_frame = convert_filter.process(frame)
    if rgb_frame is None:
        print("Convert {} to RGB failed".format(frame.get_format()))
    return rgb_frame


def frame_to_bgr_image(frame: orbsdk.VideoFrame) -> Union[Optional[np.array], Any]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())
    image = np.zeros((height, width, 3), dtype=np.uint8)
    if color_format == orbsdk.OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == orbsdk.OBFormat.BGR:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif color_format == orbsdk.OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    elif color_format ==orbsdk. OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif color_format ==orbsdk. OBFormat.I420:
        image = i420_to_bgr(data, width, height)
        return image
    elif color_format == orbsdk.OBFormat.NV12:
        image = nv12_to_bgr(data, width, height)
        return image
    elif color_format == orbsdk.OBFormat.NV21:
        image = nv21_to_bgr(data, width, height)
        return image
    elif color_format == orbsdk.OBFormat.UYVY:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    else:
        print("Unsupported color format: {}".format(color_format))
        return None
    return image

class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result

class OrbbecRGBDCamera(RGBDCameraBase):
    '''
    RealSense RGB-D Camera.
    '''
    def __init__(
        self, 
        frame_rate = 15, 
        resolution = (1280, 720),
        align_mode = "HW",
        logger_name: str = "RealSense RGBD Camera",
        shm_name_rgb: str = None, 
        shm_name_depth: str = None,
        streaming_freq: int = 30, 
        **kwargs
    ):
        '''
        Initialization.

        Parameters:
        - serial: str, required, the serial number of the realsense device;
        - frame_rate: int, optional, default: 15, the framerate of the realsense camera;
        - resolution: (int, int), optional, default: (1280, 720), the resolution of the realsense camera;
        - enable_emitter: bool, optional, default: True, whether to enable the emitter;
        - align: bool, optional, default: True, whether align the frameset with the RGB image;
        - logger_name: str, optional, default: "Camera", the name of the logger;
        - shm_name: str, optional, default: None, the shared memory name of the camera data, None means no shared memory object;
        - streaming_freq: int, optional, default: 30, the streaming frequency.
        '''
        super(OrbbecRGBDCamera, self).__init__()
        self.pipeline = orbsdk.Pipeline()
        self.config = orbsdk.Config()
        self.device = self.pipeline.get_device()
        self.device_info = self.device.get_device_info()
        self.device_pid = self.device_info.get_pid()
        self.temporal_filter = TemporalFilter(alpha = 0.5)
        try:
            profile_list = self.pipeline.get_stream_profile_list(orbsdk.OBSensorType.COLOR_SENSOR)
            color_profile = profile_list.get_video_stream_profile(resolution[0], resolution[1], orbsdk.OBFormat.RGB, frame_rate)
            self.config.enable_stream(color_profile)
            profile_list = self.pipeline.get_stream_profile_list(orbsdk.OBSensorType.DEPTH_SENSOR)
            assert profile_list is not None
            depth_profile = profile_list.get_default_video_stream_profile()
            assert depth_profile is not None
            print("color profile : {}x{}@{}_{}".format(color_profile.get_width(),
                                                        color_profile.get_height(),
                                                        color_profile.get_fps(),
                                                        color_profile.get_format()))
            print("depth profile : {}x{}@{}_{}".format(depth_profile.get_width(),
                                                        depth_profile.get_height(),
                                                        depth_profile.get_fps(),
                                                        depth_profile.get_format()))
            self.config.enable_stream(depth_profile)
        except Exception as e:
            print(e)
            return
        
        if align_mode == 'HW':
            if self.device_pid == 0x066B:
                # Femto Mega does not support hardware D2C, and it is changed to software D2C
                self.config.set_align_mode(orbsdk.OBAlignMode.SW_MODE)
            else:
                self.config.set_align_mode(orbsdk.OBAlignMode.HW_MODE)
        elif align_mode == 'SW':
            self.config.set_align_mode(orbsdk.OBAlignMode.SW_MODE)
        else:
            self.config.set_align_mode(orbsdk.OBAlignMode.DISABLE)
        try: 
            self.pipeline.enable_frame_sync()
        except Exception as e:
            print(e)
        try:
            self.pipeline.start(self.config)
            print("pipeline started")
        except Exception as e:
            print(e)
            return    
        

        # Get intrinsic
        self.camera_param_list = self.device.get_calibration_camera_param_list()
        # print(self.camera_param_list)
        
        # self.intrinsic = self.camera_param_list.color_intrinsic
        super(OrbbecRGBDCamera, self).__init__(
            logger_name = logger_name,
            shm_name_rgb = shm_name_rgb,
            shm_name_depth = shm_name_depth,
            streaming_freq = streaming_freq,
            **kwargs
        )

    def get_rgb_image(self):
        '''
        Get the RGB image from the camera.
        ''' 
        frames: orbsdk.FrameSet = None
        while (frames is None):
            frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(frame_to_bgr_image(color_frame)).astype(np.uint8)
        return color_image

    def get_depth_image(self):
        '''
        Get the depth image from the camera.
        '''
        frames: orbsdk.FrameSet = None
        depth_frame = None
        while (frames is None):
            frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
        depth_frame = frames.get_depth_frame()
        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)

        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()
        
        depth_data = depth_data.reshape((height, width))

        depth_data = depth_data.astype(np.float32) * scale
        depth_data = np.where((depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0)
        depth_data = depth_data.astype(np.float64)
        # Apply temporal filtering
        depth_data = self.temporal_filter.process(depth_data)
        depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        if depth_image is None:
            print("failed to convert frame to image")
        return depth_image

    def get_info(self):
        '''
        Get the RGB image along with the depth image from the camera.
        '''

        while True:
            try:
                frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue
                # covert to RGB format
                color_image = frame_to_bgr_image(color_frame)
                if color_image is None:
                    print("failed to convert frame to image")
                    continue
                depth_frame = frames.get_depth_frame()
                if depth_frame is None:
                    continue

                width = depth_frame.get_width()
                height = depth_frame.get_height()
                scale = depth_frame.get_depth_scale()

                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_data = depth_data.reshape((height, width))
                depth_image = depth_data.astype(np.float32) * scale
                depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_image = depth_image.astype(np.float32) / 255.0
                # depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
                # overlay color image on depth image
                # depth_image = cv2.addWeighted(color_image, 0.5, depth_image, 0.5, 0)
                if (color_image is not None) or (depth_image is not None):
                    return color_image, depth_image
                    continue
                # cv2.imshow("SyncAlignViewer ", depth_image)
                # key = cv2.waitKey(1)
                # if key == ord('q'):
                    # break
            except KeyboardInterrupt:
                break

        color_image = self.get_rgb_image()
        depth_image = self.get_depth_image()
        return color_image, depth_image
        frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
        color_frame = None
        while (color_frame is None):
            print("color")
            print(color_frame)
            while (frames is None):
                frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
            color_frame = frames.get_color_frame()
        color_image = np.asanyarray(frame_to_bgr_image(color_frame)).astype(np.uint8)
        depth_frame = None
        while (depth_frame is None):
            while (frames is None):
                frames: orbsdk.FrameSet = self.pipeline.wait_for_frames(100)
            print("depth", depth_frame is None)
            print("frame", frames is None)
            
            depth_frame = frames.get_depth_frame()
        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)

        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()
        
        depth_data = depth_data.reshape((height, width))

        depth_data = depth_data.astype(np.float32) * scale
        depth_data = np.where((depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0)
        depth_data = depth_data.astype(np.float64)
        # Apply temporal filtering
        depth_data = self.temporal_filter.process(depth_data)
        depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        return color_image, depth_image

    def get_intrinsic(self, return_mat = True):
        if return_mat:
            return np.array([
                [self.intrinsic.fx, 0., self.intrinsic.ppx],
                [0., self.intrinsic.fy, self.intrinsic.ppy],
                [0., 0., 1.]
            ], dtype = np.float32)
        else:
            return self.intrinsic
    