/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <rcutils/cmdline_parser.h>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "image_proc/debayer.hpp"
#include "image_proc/rectify.hpp"
#include "stereo_image_proc/disparity.hpp"
#include "stereo_image_proc/point_cloud2.hpp"

void print_usage()
{
  printf("Usage for stere_image_proc:\n");
  printf("options:\n");
  printf("--camera_namespace : Specifies the camera.\n");
  printf("[--approximate_sync] : Sets approximate sync policy.\n");
}

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  std::string camera_namespace;
  bool approximate_sync = false;

  // Check for command line arguments
  if(rcutils_cli_option_exist(argv, argv + argc, "--approximate_sync"))
  {
    approximate_sync = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--camera_namespace")) {
    camera_namespace = std::string(rcutils_cli_get_option(argv, argv + argc, "--camera_namespace"));
  } else {
    print_usage();
    return 0;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  const rclcpp::NodeOptions options;

  auto debayer_node_left = std::make_shared<image_proc::DebayerNode>(options, "DebayerLeftNode");
  debayer_node_left->declare_parameter("camera_namespace", camera_namespace+"/left");
  debayer_node_left->declare_parameter("image_raw", "/image_raw");

  auto debayer_node_right = std::make_shared<image_proc::DebayerNode>(options, "DebayerRightNode");
  debayer_node_right->declare_parameter("camera_namespace", camera_namespace+"/right");
  debayer_node_right->declare_parameter("image_raw", "/image_raw");

  // Rectify components, image_mono -> image_rect
  auto rectify_mono_node_left = std::make_shared<image_proc::RectifyNode>(options, "RectifyMonoLeftNode");
  rectify_mono_node_left->declare_parameter("camera_namespace", camera_namespace+"/left");
  rectify_mono_node_left->declare_parameter("image_mono", "/image_mono");

  auto rectify_mono_node_right = std::make_shared<image_proc::RectifyNode>(options, "RectifyMonoRightNode");
  rectify_mono_node_right->declare_parameter("camera_namespace", camera_namespace+"/right");
  rectify_mono_node_right->declare_parameter("image_mono", "/image_mono");

  // Rectify components, image_color -> image_rect_color
  auto rectify_color_node_left = std::make_shared<image_proc::RectifyNode>(options,"RectifyColorLeftNode");
  rectify_color_node_left->declare_parameter("camera_namespace", camera_namespace+"/left");
  rectify_color_node_left->declare_parameter("image_color", "/image_color");

  auto rectify_color_node_right = std::make_shared<image_proc::RectifyNode>(options, "RectifyColorRightNode");
  rectify_color_node_right->declare_parameter("camera_namespace", camera_namespace+"/right");
  rectify_color_node_right->declare_parameter("image_color", "/image_color");
  
  // Stereo nodelets also need to know the synchronization policy

  // Disparity component
  // Inputs: left/image_rect, left/camera_info, right/image_rect, right/camera_info
  // Outputs: disparity
  auto disparity_node = std::make_shared<stereo_image_proc::DisparityNode>(options);
  disparity_node->declare_parameter("camera_namespace", camera_namespace);
  disparity_node->declare_parameter("approximate_sync", approximate_sync);

  // PointCloud2 component
  // Inputs: left/image_rect_color, left/camera_info, right/camera_info, disparity
  // Outputs: points2
  auto point_cloud2_node = std::make_shared<stereo_image_proc::PointCloud2Node>(options);
  point_cloud2_node->declare_parameter("camera_namespace", camera_namespace);
  point_cloud2_node->declare_parameter("approximate_sync", approximate_sync);


  exec.add_node(debayer_node_left);
  exec.add_node(debayer_node_right);
  exec.add_node(rectify_mono_node_left);
  exec.add_node(rectify_mono_node_right);
  exec.add_node(rectify_color_node_left);
  exec.add_node(rectify_color_node_right);
  exec.add_node(disparity_node);
  exec.add_node(point_cloud2_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
