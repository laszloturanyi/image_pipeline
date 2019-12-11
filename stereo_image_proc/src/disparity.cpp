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


#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>

#include "stereo_image_proc/disparity.hpp"


namespace stereo_image_proc {

using namespace sensor_msgs::msg;
using namespace stereo_msgs::msg;
using namespace std::placeholders;

DisparityNode::DisparityNode(const rclcpp::NodeOptions & options, const std::string & node_name)
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
          
          std::string disparity_topic = camera_namespace_ + "/disparity";
          connectCb();

          // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
          std::lock_guard<std::mutex> lock(connect_mutex_);

          pub_disparity_ = create_publisher<DisparityImage>(disparity_topic, 10);
        }
        else if (parameter.get_name() == "approximate_sync") {
          approx_ = parameter.as_bool();
          setSync();
        }
      }
      return result;
    };

  //TODO Should we take care of these checks at every parameter change
  //Also, all other min,max values should be checked.
  //Tweak all settings to be valid
  //config.prefilter_size |= 0x1; // must be odd
  //config.correlation_window_size |= 0x1; // must be odd
  //config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

  // Set stereo algorithm (Block Matching or SemiGlobal Block Matching)
  std::string stereo_algorithm = this->declare_parameter("stereo_algorithm", "SGBM");
  
  // Disparity block matching pre-filtering parameters
  // Bound on normalized pixel values [1, 63]
  block_matcher_.setPreFilterCap(this->declare_parameter("prefilter_cap", 31));
  
  // Disparity block matching correlation parameters
  // SAD correlation window width, pixels [5, 255]
  block_matcher_.setCorrelationWindowSize(this->declare_parameter("correlation_window_size", 15));

  // Disparity to begin search at, pixels (may be negative) [-128, 128]
  block_matcher_.setMinDisparity(this->declare_parameter("min_disparity", 0));
  
  // Number of disparities to search, pixels [32, 256]
  block_matcher_.setDisparityRange(this->declare_parameter("disparity_range", 64));
  
  // Disparity block matching post-filtering parameters
  // Filter out if best match does not sufficiently exceed the next-best match [0, 100]
  block_matcher_.setUniquenessRatio(this->declare_parameter("uniqueness_ratio", (double) 15));  
  
  // Reject regions smaller than this size, pixels [0, 1000]
  block_matcher_.setSpeckleSize(this->declare_parameter("speckle_size", 100));
  
  // Max allowed difference between detected disparities [0, 31]
  block_matcher_.setSpeckleRange(this->declare_parameter("speckle_range", 4));

  if(stereo_algorithm == "BM")
  {
    block_matcher_.setStereoType(StereoProcessor::BM);
    
    // Normalization window size, pixels, pre-filtering parameter, only for BM [5, 255]
    block_matcher_.setPreFilterSize(this->declare_parameter("prefilter_size", 9));

    // Filter out if SAD window response does not exceed texture threshold [0, 10000]
    block_matcher_.setTextureThreshold(this->declare_parameter("texture_threshold", 10));
  }
  if(stereo_algorithm == "SGBM")
  {
    block_matcher_.setStereoType(StereoProcessor::SGBM);

    // Run the full variant of the algorithm, only available in SGBM
    block_matcher_.setSgbmMode(this->declare_parameter("fullDP", (int) false));
    
    // The first parameter controlling the disparity smoothness, only available in SGBM [0, 4000]
    block_matcher_.setP1(this->declare_parameter("P1", (double) 200));
    
    // The second parameter controlling the disparity smoothness., only available in SGBM [0, 4000]
    block_matcher_.setP2(this->declare_parameter("P2", (double) 400));
    
    // Maximum allowed difference (in integer pixel units) in the left-right disparity check, only available in SGBM [0, 128]
    block_matcher_.setDisp12MaxDiff(this->declare_parameter("disp12MaxDiff", 0));
  }
  
  // Sync queue size
  queue_size_ = this->declare_parameter("queue_size", 5);

  this->set_on_parameters_set_callback(parameter_change_cb);

}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_disparity_->get_subscription_count() == 0)
  if(0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints(this, "raw");
    sub_l_image_.subscribe(this, camera_namespace_+"/left/image_rect", hints.getTransport());
    sub_l_info_ .subscribe(this, camera_namespace_+"/left/camera_info");
    sub_r_image_.subscribe(this, camera_namespace_+"/right/image_rect", hints.getTransport());
    sub_r_info_ .subscribe(this, camera_namespace_+"/right/camera_info");
  }
}

// Synchronize inputs. Topic subscriptions happen on demand in the connection
// callback. Optionally do approximate synchronization.
void DisparityNode::setSync()
{
  RCLCPP_INFO(get_logger(), "Using %s policy! ", approx_ ? "approximate sync" : "exact sync");
  if (approx_)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),
                                                   sub_l_image_, sub_l_info_,
                                                   sub_r_image_, sub_r_info_) );
      approximate_sync_->registerCallback(std::bind(&DisparityNode::imageCb,
                                                      this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                       sub_l_image_, sub_l_info_,
                                       sub_r_image_, sub_r_info_) );
      exact_sync_->registerCallback(std::bind(&DisparityNode::imageCb,
                                                this, _1, _2, _3, _4));
    }
}

void DisparityNode::imageCb(const Image::ConstSharedPtr& l_image_msg,
                               const CameraInfo::ConstSharedPtr& l_info_msg,
                               const Image::ConstSharedPtr& r_image_msg,
                               const CameraInfo::ConstSharedPtr& r_info_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  RCLCPP_INFO(get_logger(), "publish disparity image");

  // Allocate new disparity image message
  auto disp_msg = std::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border   = block_matcher_.getCorrelationWindowSize() / 2;
  int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  pub_disparity_->publish(*disp_msg);
  RCLCPP_INFO(get_logger(), "publish disparity image");
}


} // namespace stereo_image_proc

// Register nodelet
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::DisparityNode)