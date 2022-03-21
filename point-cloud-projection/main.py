#!/usr/bin/env python3
import json
import os
import tempfile
import platform
from pathlib import Path
import ransac
import open3d as o3d
# import RPi.GPIO as GPIO


import cv2
import depthai as dai
import numpy

# import socket

# HOST = '155.41.122.253'
# PORT = 2000
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST,PORT))

#setup for new depthai version
# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = False

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("disparity")
xoutRight.setStreamName("right")


# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.initialConfig.setConfidenceThreshold(245)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.depth.link(xout.input)
monoRight.out.link(xoutRight.input)


try:
    from projector_3d import PointCloudVisualizer
except ImportError as e:
    raise ImportError(f"\033[1;5;31mError occured when importing PCL projector: {e} \033[0m ")


#setup for old depthai version
# device = depthai.Device("", False)
# pipeline = device.create_pipeline(config={
#     'streams': ['right', 'depth'],
#     'ai': {
#         "blob_file": str(Path('./mobilenet-ssd/mobilenet-ssd.blob').resolve().absolute()),
#     },
#     'camera': {'mono': {'resolution_h': 720, 'fps': 30}},
# })

# if pipeline is None:
#     raise RuntimeError("Error creating a pipeline!")

#setup PI
# GPIO.setmode(GPIO.BOARD)
# #motor1
# GPIO.setup(8,GPIO.OUT)
# pwm2 = GPIO.PWM(8, 100)
# pwm2.start(0)

right = None
pcl_converter = None
vis = o3d.visualization.Visualizer()
vis.create_window()
isstarted = False
with dai.Device(pipeline) as device:
    qDepth = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    while True:
        # data_packets = pipeline.get_available_data_packets()
        
        #to get prism
        # was at the border work 0.7 to 2.2 with 1/4 z rotation 
        corners = numpy.asarray([[-0.5,-1.0,0.7],[0.5,-1.0,0.7],[0.5,1.0,0.7],[-0.5,1.0,0.7],[-0.5,-1.0,1.7],[0.5,-1.0,1.7],[0.5,1.0,1.7],[-0.5,1.0,1.7]])

        
        rotation_matrix = numpy.asarray([
                            [
                                0.999945342540741,
                                -0.008072619326412678,
                                0.006644818000495434
                            ],
                            [
                                0.008046404458582401,
                                0.9999597668647766,
                                0.00396250793710351
                            ],
                            [
                                -0.00667653838172555,
                                -0.0039088246412575245,
                                0.9999700784683228
                            ]
                        ])
        rotation_matrix = rotation_matrix.astype("float64")
        
        # corners = numpy.asarray([[0,0,0.7],[1,0,0.7],[1,2.14,0.7],[0,2.14,0.7],[0,0,2],[1,0,2],[1,2.14,2],[0,2.14,2]])
        # moved to the left 
        # corners = numpy.asarray([[-1,0,0.7],[0,0,0.7],[0,2.14,0.7],[-1,2.14,0.7],[-1,0,2],[0,0,2],[0,2.14,2],[-1,2.14,2]])
        bounds = corners.astype("float64")
        bounds = o3d.utility.Vector3dVector(bounds)
        oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(bounds)
        # oriented_bounding_box.rotate(rotation_matrix, numpy.asarray([0,0,0]))
        
        
        # for packet in data_packets:
            # if packet.stream_name == "right":
                # right = packet.getData()
        inRight = qRight.get()
        right = inRight.getFrame()
        # print(type(right))
        cv2.imshow("right", right)
            # elif packet.stream_name == "depth":
                #frame = packet.getData()
        inDepth = qDepth.get()
        frame = inDepth.getFrame()
        median = cv2.medianBlur(frame, 5)
        median2 = cv2.medianBlur(median,5)
        # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        # print(type(median2))
        cv2.imshow("depth",median2)
        '''
        median3 = cv2.medianBlur(median,5)
        median4 = cv2.medianBlur(median,5)
        median5 = cv2.medianBlur(median,5)

        bilateral = cv2.bilateralFilter(frame,15,75,75)
        '''
        if right is not None:

            if pcl_converter is None:
                fd, path = tempfile.mkstemp(suffix='.json')
                with os.fdopen(fd, 'w') as tmp:
                    json.dump({
                        "width": 1280,
                        "height": 800,
                        "intrinsic_matrix": 
                        #[
                        #     [
                        #         1494.0189208984375,
                        #         0.0,
                        #         957.5805053710938
                        #     ],
                        #     [
                        #         0.0,
                        #         1492.128662109375,
                        #         543.8084106445313
                        #     ],
                        #     [
                        #         0.0,
                        #         0.0,
                        #         1.0
                        #     ]
                        # ]
                            [[995.72348093,   0.,         639.55650338],
                            [  0.,         994.46367699, 363.3655595 ],
                            [  0.,           0.,           1.        ]]
                    }, tmp)

                pcl_converter = PointCloudVisualizer(path)
            pcd = pcl_converter.rgbd_to_projection(median, right)
            # print(numpy.asarray(pcd.points))
            #to get points within bounding box
            num_pts = oriented_bounding_box.get_point_indices_within_bounding_box(pcd.points)
            # cropped_pcd = pcd.crop(oriented_bounding_box)

            if not isstarted:
                vis.add_geometry(pcd)
                # vis.add_geometry(pcd_x)
                # vis.add_geometry(pcd_y)
                # vis.add_geometry(pcd_z)
                vis.add_geometry(oriented_bounding_box)
                # vis.add_geometry(pcd_pback)
                # vis.add_geometry(pcd_pfront)
                # vis.add_geometry(pcd_pmid)
                isstarted = True       
                            
            else:
                vis.update_geometry(pcd)
                # vis.update_geometry(pcd_x)
                # vis.update_geometry(pcd_y)
                # vis.update_geometry(pcd_z)
                # vis.update_geometry(pcd_pback)
                # vis.update_geometry(pcd_pmid)
                # vis.update_geometry(pcd_pfront)
                vis.update_geometry(oriented_bounding_box)
                vis.poll_events()
                vis.update_renderer()
            if len(num_pts)>5000:
                # pwm2.ChangeDutyCycle(100)
                print("Obstacle")
                # s.send(bytes('1','utf-8'))
            else:
                # pwm2.ChangeDutyCycle(0)
                print("Nothing")
                # s.send(bytes('0','utf-8'))
            # print("X", numpy.shape(numpy.asarray(pcd.points)[:,0]))
            # print("Y", numpy.shape(numpy.asarray(pcd.points)[:,1]))
            # print("Z", numpy.shape(numpy.asarray(pcd.points)[:,2]))

            # print(numpy.asarray((pcd.points)))
            # print(numpy.shape(numpy.asarray((pcl_converter.pcl.points))))
            # pointsc = numpy.asarray((pcl_converter.pcl.points))
            # pointspcd = numpy.asarray((pcd.points))
            # print("X max: , X min: ", max(pointspcd[:,0]),min(pointspcd[:,0]))
            # print("Y max: , Y min: ", max(pointspcd[:,1]),min(pointspcd[:,1]))
            # print("Z max: , Z min: ", max(pointspcd[:,2]),min(pointspcd[:,2]))


                # x,y,z = ransac.find_plane(pcd)
                # ransac.show_graph(x,y,z)
            # cv2.imshow(packet.stream_name, frame)
            '''
            cv2.imshow("filter", median)
            '''
            # cv2.imshow("filter2", median2)
            '''
            cv2.imshow("filter3", median3)
            cv2.imshow("filter4", median4)
            '''
            #cv2.imshow("filter5", median5)


            #cv2.imshow("filter2", bilateral)

        if cv2.waitKey(1) == ord("q"):
            break

    if pcl_converter is not None:
        pcl_converter.close_window()
