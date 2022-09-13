## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

#  ********************************************TEST01**********************************************
# # Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()
#
# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))
#
# found_rgb = False
# for s in device.sensors:
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True
#         break
# if not found_rgb:
#     print("The demo requires Depth camera with Color sensor")
#     exit(0)
#
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#
# if device_product_line == 'L500':
#     print("111")
#     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
# else:
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#
# # Start streaming
# pipeline.start(config)
#
# try:
#     while True:
#
#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()
#         if not depth_frame or not color_frame:
#             continue
#
#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         print(depth_frame)
#         color_image = np.asanyarray(color_frame.get_data())
#         print(color_image)
#
#         # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#
#         depth_colormap_dim = depth_colormap.shape
#         color_colormap_dim = color_image.shape
#
#         # If depth and color resolutions are different, resize color image to match depth image for display
#         if depth_colormap_dim != color_colormap_dim:
#             resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
#             images = np.hstack((resized_color_image, depth_colormap))
#         else:
#             images = np.hstack((color_image, depth_colormap))
#
#         # Show images
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RealSense', iges)
#         cv2.waitKey(1)
#
# finally:
#
#     # Stop streaming
#     pipeline.stop()



#  ********************************************TEST02**********************************************


# 获取摄像头的内参是为了将像素坐标转化成实际坐标

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()
# 获取内参
depth_profile = depth.get_profile()
print(depth_profile)
# <pyrealsense2.video_stream_profile: 1(0) 640x480 @ 30fps 1>
print(type(depth_profile))
# <class 'pyrealsense2.pyrealsense2.stream_profile'>
print(depth_profile.fps())
# 30
print(depth_profile.stream_index())
# 0
print(depth_profile.stream_name())
# Depth
print(depth_profile.stream_type())
# stream.depth
print('', depth_profile.unique_id)
# <bound method PyCapsule.unique_id of <pyrealsense2.video_stream_profile: 1(0) 640x480 @ 30fps 1>>

color_profile = color.get_profile()
print(color_profile)
# <pyrealsense2.video_stream_profile: 2(0) 640x480 @ 30fps 6>
print(type(color_profile))
# <class 'pyrealsense2.pyrealsense2.stream_profile'>
print(depth_profile.fps())
# 30
print(depth_profile.stream_index())
# 0

cvsprofile = rs.video_stream_profile(color_profile)
dvsprofile = rs.video_stream_profile(depth_profile)

color_intrin = cvsprofile.get_intrinsics()
print("color intrin:" , color_intrin)
print("color intrin type:",type(color_intrin))
# width: 640, height: 480, ppx: 318.482, ppy: 241.167, fx: 616.591, fy: 616.765, model: 2, coeffs: [0, 0, 0, 0, 0]

# depth_intrin = dvsprofile.get_intrinsics()
# print(depth_intrin)
# # width: 640, height: 480, ppx: 317.78, ppy: 236.709, fx: 382.544, fy: 382.544, model: 4, coeffs: [0, 0, 0, 0, 0]
#
# extrin = depth_profile.get_extrinsics_to(color_profile)
# print(extrin)
# # rotation: [0.999984, -0.00420567, -0.00380472, 0.00420863, 0.999991, 0.00076919, 0.00380145, -0.00078519, 0.999992]
# translation: [0.0147755, 0.000203265, 0.00051274]

