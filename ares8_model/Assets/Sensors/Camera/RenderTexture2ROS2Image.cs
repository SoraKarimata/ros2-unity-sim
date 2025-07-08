// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// RenderTexture2ROS2Image.cs
// Author: Ar-Ray (2022)
// https://github.com/Ar-Ray-code

using System.Collections;
using UnityEngine;
using System.IO;
using System;
using ROS2;

namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class RenderTexture2ROS2Image : MonoBehaviour
    {
        // Start is called before the first frame update
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.Image> image_pub;
        private IPublisher<sensor_msgs.msg.CameraInfo> camera_info_pub;

        public RenderTexture renderTexture;

        // width and height fit automatically to the renderTexture
        private Int32 width;
        private Int32 height;

        [Header("Camera Calibration Parameters")]
        [Tooltip("Camera distortion parameters (D) - 5 values")]
        public double[] distortionCoefficients = new double[5] {0, 0, 0, 0, 0};
        
        [Tooltip("Camera intrinsic parameter matrix (K) - 3x3 matrix")]
        public double[] intrinsicMatrix = new double[9] {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        
        [Tooltip("Camera projection matrix (P) - 3x4 matrix")]
        public double[] projectionMatrix = new double[12] {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
        
        [Tooltip("Camera distortion model")]
        public string distortionModel = "plumb_bob";
        
        [Header("ROS2 Setting")]
        public string nodeName = "RGB_Camera";
        public string imageTopic = "image_raw";
        public string cameraInfoTopic = "camera_info";
        public string frameId = "camera_frame";

        // Initialize ROS2
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        // Main loop
        void Update()
        {   
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    string uniqueNodeName = nodeName + "_" + gameObject.name;
                    ros2Node = ros2Unity.CreateNode(uniqueNodeName);
                    image_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(imageTopic);
                    camera_info_pub = ros2Node.CreatePublisher<sensor_msgs.msg.CameraInfo>(cameraInfoTopic);
                }

                sensor_msgs.msg.Image image_msg = new sensor_msgs.msg.Image();

                Color[] pixels;
                pixels = CreateTexture2D(renderTexture);

                image_msg.Height = (UInt32)(height);
                image_msg.Width = (UInt32)(width);
                image_msg.Encoding = "rgb8";
                image_msg.Is_bigendian = (byte)(0);
                image_msg.Step = (UInt32)(width * 3); // 3byte width is cols

                var data_array_byte = new byte[width * height * 3];
                var height_1 = 0;
                for (int i = 0; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        height_1 = height - 1 - i;
                        data_array_byte[i * width * 3 + j * 3 + 0] = (byte)(pixels[height_1 * width + j].r * 255);
                        data_array_byte[i * width * 3 + j * 3 + 1] = (byte)(pixels[height_1 * width + j].g * 255);
                        data_array_byte[i * width * 3 + j * 3 + 2] = (byte)(pixels[height_1 * width + j].b * 255);
                    }
                }
                image_msg.Data = data_array_byte;
                image_pub.Publish(image_msg);

                // Generate CameraInfo message
                sensor_msgs.msg.CameraInfo camera_info_msg = new sensor_msgs.msg.CameraInfo();
                camera_info_msg.Height = (uint)height;
                camera_info_msg.Width = (uint)width;
                camera_info_msg.Distortion_model = distortionModel;
                
                // Set distortion coefficients
                camera_info_msg.D = new double[distortionCoefficients.Length];
                Array.Copy(distortionCoefficients, camera_info_msg.D, distortionCoefficients.Length);
                
                // Set intrinsic parameter matrix (read-only, so set elements individually)
                for (int i = 0; i < intrinsicMatrix.Length; i++)
                {
                    camera_info_msg.K[i] = intrinsicMatrix[i];
                }
                
                // Set projection matrix (read-only, so set elements individually)
                for (int i = 0; i < projectionMatrix.Length; i++)
                {
                    camera_info_msg.P[i] = projectionMatrix[i];
                }
                
                camera_info_msg.Binning_x = 1;
                camera_info_msg.Binning_y = 1;
                camera_info_msg.Roi = new sensor_msgs.msg.RegionOfInterest();
                camera_info_msg.Roi.X_offset = 0;
                camera_info_msg.Roi.Y_offset = 0;
                camera_info_msg.Roi.Height = 0;
                camera_info_msg.Roi.Width = 0;
                camera_info_msg.Roi.Do_rectify = false;
                camera_info_msg.Header = new std_msgs.msg.Header();
                camera_info_msg.Header.Frame_id = frameId;
                ros2Node.clock.UpdateROSClockTime(camera_info_msg.Header.Stamp);

                camera_info_pub.Publish(camera_info_msg);
            }
        }

        // Input: RenderTexture
        // Output: Color[]
        Color[] CreateTexture2D(RenderTexture rt)
        {
            width = rt.width;
            height = rt.height;

            var tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            var oldActive = RenderTexture.active;
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();

            var colors = tex.GetPixels();
            RenderTexture.active = oldActive;

            Destroy(tex);
            return colors;
        }
    }
}