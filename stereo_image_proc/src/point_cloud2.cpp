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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.hpp>

#include "stereo_image_proc/point_cloud2.hpp"

namespace stereo_image_proc
{
using namespace stereo_msgs;
using namespace std::placeholders;

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

PointCloud2Node::PointCloud2Node(const rclcpp::NodeOptions & options,  const std::string & node_name)
: Node(node_name, options)
{
    auto parameter_change_cb =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      for (auto parameter : parameters) {
        if (parameter.get_name() == "camera_namespace") {
          camera_namespace_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "reset camera_namespace to %s! ", camera_namespace_.c_str());
          
          std::string points2_topic = camera_namespace_ + "/points2";
          connectCb();

          // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
          std::lock_guard<std::mutex> lock(connect_mutex_);
          pub_points2_ = create_publisher<PointCloud2>(points2_topic, 10);
        }
        else if (parameter.get_name() == "approximate_sync") {
          approx_ = parameter.as_bool();
          setSync();
        }
      }


      return result;
    };
  
  // Sync queue size
  queue_size_ = this->declare_parameter("queue_size", 5);

  this->set_on_parameters_set_callback(parameter_change_cb);
}

// Synchronize inputs. Topic subscriptions happen on demand in the connection
// callback. Optionally do approximate synchronization.
void PointCloud2Node::setSync()
{
  RCLCPP_INFO(get_logger(), "Using %s policy! ", approx_ ? "approximate sync" : "exact sync");
  if (approx_)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_info_, sub_disparity_) );
    approximate_sync_->registerCallback(std::bind(&PointCloud2Node::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_info_, sub_disparity_) );
    exact_sync_->registerCallback(std::bind(&PointCloud2Node::imageCb,
                                              this, _1, _2, _3, _4));
  }
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloud2Node::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  //if (pub_points2_->get_subscription_count() == 0)
  if (0)
  {
    sub_l_image_  .unsubscribe();
    sub_l_info_   .unsubscribe();
    sub_r_info_   .unsubscribe();
    sub_disparity_.unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    image_transport::TransportHints hints(this, "raw");
    sub_l_image_  .subscribe(this, camera_namespace_+"/left/image_rect_color", hints.getTransport());
    sub_l_info_   .subscribe(this, camera_namespace_+"/left/camera_info");
    sub_r_info_   .subscribe(this, camera_namespace_+"/right/camera_info");
    sub_disparity_.subscribe(this, camera_namespace_+"/disparity");
  }
}


void PointCloud2Node::imageCb(const Image::ConstSharedPtr& l_image_msg,
                                 const CameraInfo::ConstSharedPtr& l_info_msg,
                                 const CameraInfo::ConstSharedPtr& r_info_msg,
                                 const DisparityImage::ConstSharedPtr& disp_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  // Calculate point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud2 message (2D image-like layout)
  auto points_msg = std::make_shared<PointCloud2>();
  points_msg->header = disp_msg->header;
  points_msg->height = mat.rows;
  points_msg->width  = mat.cols;
  points_msg->is_bigendian = false;
  points_msg->is_dense = false; // there may be invalid points

  sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  for (int v = 0; v < mat.rows; ++v)
  {
    for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      if (isValidPoint(mat(v,u)))
      {
        // x,y,z
        *iter_x = mat(v, u)[0];
        *iter_y = mat(v, u)[1];
        *iter_z = mat(v, u)[2];
      }
      else
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  const std::string& encoding = l_image_msg->encoding;
  if (encoding == enc::MONO8)
  {
    const cv::Mat_<uint8_t> color(l_image_msg->height, l_image_msg->width,
                                  (uint8_t*)&l_image_msg->data[0],
                                  l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        uint8_t g = color(v,u);
        *iter_r = *iter_g = *iter_b = g;
      }
    }
  }
  else if (encoding == enc::RGB8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& rgb = color(v,u);
        *iter_r = rgb[0];
        *iter_g = rgb[1];
        *iter_b = rgb[2];
      }
    }
  }
  else if (encoding == enc::BGR8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& bgr = color(v,u);
        *iter_r = bgr[2];
        *iter_g = bgr[1];
        *iter_b = bgr[0];
      }
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),"Could not fill color channel of the point cloud, unsupported encoding '%s'", encoding.c_str());
  }

  pub_points2_->publish(*points_msg);
  RCLCPP_INFO(get_logger(), "publish points2");
}

} // namespace stereo_image_proc

// Register nodelet
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::PointCloud2Node)
