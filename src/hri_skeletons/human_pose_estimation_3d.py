#!/usr/bin/env python3
"""
 Copyright (c) 2019 Intel Corporation
               2020 Bristol Robotics Lab, Séverin Lemaignan

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
      http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""

import json
import os
import sys
import uuid

import cv2
import numpy as np

from hri_skeletons.urdf_generator import make_urdf_human
from hri_skeletons.modules.inference_engine import InferenceEngine
from hri_skeletons.modules.input_reader import InputReader
from hri_skeletons.modules.draw import Plotter3d, draw_poses
from hri_skeletons.modules.parse_poses import parse_poses

import rospy
import rosparam
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType

def generate_id():
    return str(uuid.uuid4())[:5] # for a 5 char long ID

class HumanPoseEstimator3D:
    
    def __init__(self, model, device, height_size, debug=False):

        self.debug = debug

        self.bridge = CvBridge()

   
        self.stride = 8

        rospy.loginfo("Instanciating the OpenVino inference engine...")
        self.inference_engine = InferenceEngine(model, device, self.stride)
        rospy.loginfo("Done instanciating the OpenVino inference engine")
    
        if self.debug:
   
            self.canvas_3d = np.zeros((720, 1280, 3), dtype=np.uint8)
            self.plotter = Plotter3d(self.canvas_3d.shape[:2])
            self.canvas_3d_window_name = 'Human pose estimation 3D - DEBUG'
            cv2.namedWindow(self.canvas_3d_window_name)
            cv2.setMouseCallback(self.canvas_3d_window_name, Plotter3d.mouse_callback)
    
        extrinsics = {
            "R": [[0.1656794936,0.0336560618,-0.9856051821    ],
                [-0.09224101321,0.9955650135,0.01849052095    ],
                [0.9818563545,0.08784972047,0.1680491765    ]],
            "t": [[17.76193366    ],    [126.741365    ],    [286.3860507    ]]
            }
        self.R = np.array(extrinsics['R'], dtype=np.float32)
        self.t = np.array(extrinsics['t'], dtype=np.float32)
        self.R_inv = np.linalg.inv(self.R)
    
        self.base_height = height_size
        self.fx = -1 # TODO: get that from camera info
    
        self.mean_time = 0

        self.got_frames = False
        self.urdf_published = False

        self.subscriber = rospy.Subscriber("image", Image, self.on_image_cb,  queue_size = 1)
        rospy.loginfo("Waiting for frames")
    
    def rotate_poses(self, poses_3d):
        for pose_id in range(poses_3d.shape[0]):
            pose_3d = poses_3d[pose_id].reshape((-1, 4)).transpose()
            pose_3d[0:3] = np.dot(self.R_inv, pose_3d[0:3] - self.t)
            poses_3d[pose_id] = pose_3d.transpose().reshape(-1)
    
        return poses_3d
    
    
    def on_image_cb(self, data):

        if not self.got_frames:
            rospy.loginfo("Got first frame! Starting skeleton detection")
            self.got_frames = True

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
    
        current_time = cv2.getTickCount()
        input_scale = self.base_height / frame.shape[0]
        scaled_img = cv2.resize(frame, dsize=None, fx=input_scale, fy=input_scale)
    
        if self.fx < 0:  # Focal length is unknown
            self.fx = np.float32(0.8 * frame.shape[1])
    
        inference_result = self.inference_engine.infer(scaled_img)
    
        poses_3d, poses_2d = parse_poses(inference_result, input_scale, self.stride, self.fx, is_video = True)
    
        edges = []
        if len(poses_3d) > 0:

            if not self.urdf_published:
                id = generate_id()
                urdf = make_urdf_human(id)
                rospy.loginfo("Publishing URDF description for <%s>: human_description_%s" % (id, id))
                rosparam.set_param_raw("human_description_%s" % id, urdf)
                self.urdf_published = True

            poses_3d = self.rotate_poses(poses_3d)
            poses_3d_copy = poses_3d.copy()
            x = poses_3d_copy[:, 0::4]
            y = poses_3d_copy[:, 1::4]
            z = poses_3d_copy[:, 2::4]
            poses_3d[:, 0::4], poses_3d[:, 1::4], poses_3d[:, 2::4] = -z, x, -y
    
            poses_3d = poses_3d.reshape(poses_3d.shape[0], 19, -1)[:, :, 0:3]
            edges = (Plotter3d.SKELETON_EDGES + 19 * np.arange(poses_3d.shape[0]).reshape((-1, 1, 1))).reshape((-1, 2))
        
        if self.debug:
            self.plotter.plot(self.canvas_3d, poses_3d, edges)
        
            draw_poses(frame, poses_2d)
            current_time = (cv2.getTickCount() - current_time) / cv2.getTickFrequency()
            if self.mean_time == 0:
                self.mean_time = current_time
            else:
                self.mean_time = self.mean_time * 0.95 + current_time * 0.05
            cv2.putText(frame, 'FPS: {}'.format(int(1 / self.mean_time * 10) / 10),
                        (40, 80), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
            
            cv2.imshow(self.canvas_3d_window_name, self.canvas_3d)
            cv2.imshow('3D Human Pose Estimation - DEBUG', frame)
            cv2.waitKey(10)
    



if __name__ == '__main__':
    
    rospy.init_node("human_pose_estimation_3d")

    # link to the XML file containing the OpenVINO model for 3D human pose estimation
    # https://github.com/openvinotoolkit/open_model_zoo/blob/master/models/public/human-pose-estimation-3d-0001/description/human-pose-estimation-3d-0001.md
    model = rospy.get_param("~model")

    # one of: CPU, GPU, FPGA, HDDL or MYRIAD. '
    device = rospy.get_param("~device", "CPU")

    # Network input layer height size.
    height_size = rospy.get_param("~height_size", 256)

    # if true, display a 3D canvas with the skeletons
    debug = rospy.get_param("~debug", False)
    if debug:
        rospy.logwarn("Debugging mode ON")

    estimator = HumanPoseEstimator3D(model, device, height_size, debug)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
