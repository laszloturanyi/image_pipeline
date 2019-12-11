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

#ifndef STEREO_IMAGE_PROC__POINT_CLOUD2_HPP_
#define STEREO_IMAGE_PROC__POINT_CLOUD2_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>

namespace stereo_image_proc
{

using namespace sensor_msgs::msg;
using namespace stereo_msgs::msg;
using namespace message_filters::sync_policies;

class PointCloud2Node : public rclcpp::Node
{
public:
  explicit PointCloud2Node(const rclcpp::NodeOptions &, const std::string & = "PointCloud2Node");
  void connectCb();
  void setSync();
  void imageCb(const Image::ConstSharedPtr& l_image_msg, const CameraInfo::ConstSharedPtr& l_info_msg,
          const CameraInfo::ConstSharedPtr& r_info_msg, const DisparityImage::ConstSharedPtr& disp_msg);
private:
  std::string camera_namespace_;

  // Publications
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_points2_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;

  typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  bool approx_;
  int queue_size_;

  std::mutex connect_mutex_;  

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer

};

}   // namespace image_proc

#endif // STEREO_IMAGE_PROC__POINT_CLOUD2_HPP_