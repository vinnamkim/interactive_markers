/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David Gossow
 */

#include "interactive_markers/detail/message_context.h"
#include "interactive_markers/tools.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer_interface.h>

//#define DBG_MSG( ... ) ROS_DEBUG( __VA_ARGS__ );
#define DBG_MSG( ... ) printf("   "); printf( __VA_ARGS__ ); printf("\n");

namespace interactive_markers
{

template<class MsgT>
MessageContext<MsgT>::MessageContext(
  tf2::BufferCore &tf,
  const std::string& target_frame,
  const typename MsgT::ConstSharedPtr& _msg,
  bool enable_autocomplete_transparency)
: tf_(tf)
, target_frame_(target_frame)
, enable_autocomplete_transparency_(enable_autocomplete_transparency)
{
  // copy message, as we will be modifying it
  msg = std::make_shared<MsgT>( *_msg );

  init();
}

template<class MsgT>
MessageContext<MsgT>& MessageContext<MsgT>::operator=( const MessageContext<MsgT>& other )
{
  open_marker_idx_ = other.open_marker_idx_;
  open_pose_idx_ = other.open_pose_idx_;
  target_frame_ = other.target_frame_;
  enable_autocomplete_transparency_ = other.enable_autocomplete_transparency_;
  return *this;
}

template<class MsgT>
bool MessageContext<MsgT>::getTransform( std_msgs::msg::Header& header, geometry_msgs::msg::Pose& pose_msg )
{
  try
  {
    if ( header.frame_id != target_frame_ )
    {
      // get transform
      auto time_point = tf2_ros::fromMsg(header.stamp);
      const auto transform =  tf_.lookupTransform( target_frame_, header.frame_id, time_point);
      DBG_MSG( "Transform %s -> %s at time %f is ready.", header.frame_id.c_str(), target_frame_.c_str(), tf2_ros::timeToSec(header.stamp) );

      // if timestamp is given, transform message into target frame
      if ( header.stamp != builtin_interfaces::msg::Time())
      {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = header;
        pose_in.pose = pose_msg;

        tf2::doTransform(pose_in, pose_out, transform);
        // store transformed pose in original message
        pose_msg = pose_out.pose;
        DBG_MSG("Changing %s to %s", header.frame_id.c_str(), target_frame_.c_str());
        header.frame_id = target_frame_;
      }
    }
  }
  catch ( tf2::ExtrapolationException& e )
  {
    tf2::TimePoint latest_time;
    std::string error_string;

    tf_._getLatestCommonTime(
      tf_._validateFrameId("get_latest_common_time", target_frame_),
      tf_._validateFrameId("get_latest_common_time", header.frame_id),
      latest_time, &error_string );

    auto zero_time = tf2_ros::fromMsg(builtin_interfaces::msg::Time());

    // if we have some tf info and it is newer than the requested time,
    // we are very unlikely to ever receive the old tf info in the future.
    if ( latest_time != zero_time && 
      latest_time > tf2_ros::fromMsg(header.stamp) )
    {
      std::ostringstream s;
      s << "The init message contains an old timestamp and cannot be transformed ";
      s << "('" << header.frame_id << "' to '" << target_frame_
        << "' at time " << header.stamp.sec << "." << header.stamp.nanosec << ").";
      throw InitFailException( s.str() );
    }
    return false;
  }
  return true;
  // all other exceptions need to be handled outside
}

template<class MsgT>
void MessageContext<MsgT>::getTfTransforms( std::vector<visualization_msgs::msg::InteractiveMarker>& msg_vec, std::list<size_t>& indices )
{
  std::list<size_t>::iterator idx_it;
  for ( idx_it = indices.begin(); idx_it != indices.end(); )
  {
    visualization_msgs::msg::InteractiveMarker& im_msg = msg_vec[ *idx_it ];
    // transform interactive marker
    bool success = getTransform( im_msg.header, im_msg.pose );
    // transform regular markers
    for ( unsigned c = 0; c<im_msg.controls.size(); c++ )
    {
      visualization_msgs::msg::InteractiveMarkerControl& ctrl_msg = im_msg.controls[c];
      for ( unsigned m = 0; m<ctrl_msg.markers.size(); m++ )
      {
        visualization_msgs::msg::Marker& marker_msg = ctrl_msg.markers[m];
        if ( !marker_msg.header.frame_id.empty() ) {
          success = success && getTransform( marker_msg.header, marker_msg.pose );
        }
      }
    }

    if ( success )
    {
      idx_it = indices.erase(idx_it);
    }
    else
    {
      DBG_MSG( "Transform %s -> %s at time %f is not ready.", im_msg.header.frame_id.c_str(), target_frame_.c_str(), tf2_ros::timeToSec(im_msg.header.stamp) );
      ++idx_it;
    }
  }
}

template<class MsgT>
void MessageContext<MsgT>::getTfTransforms( std::vector<visualization_msgs::msg::InteractiveMarkerPose>& msg_vec, std::list<size_t>& indices )
{
  std::list<size_t>::iterator idx_it;
  for ( idx_it = indices.begin(); idx_it != indices.end(); )
  {
    visualization_msgs::msg::InteractiveMarkerPose& msg = msg_vec[ *idx_it ];
    if ( getTransform( msg.header, msg.pose ) )
    {
      idx_it = indices.erase(idx_it);
    }
    else
    {
      DBG_MSG( "Transform %s -> %s at time %f is not ready.", msg.header.frame_id.c_str(), target_frame_.c_str(), tf2_ros::timeToSec(msg.header.stamp) );
      ++idx_it;
    }
  }
}

template<class MsgT>
bool MessageContext<MsgT>::isReady()
{
  return open_marker_idx_.empty() && open_pose_idx_.empty();
}

template<>
void MessageContext<visualization_msgs::msg::InteractiveMarkerUpdate>::init()
{
  // mark all transforms as being missing
  for ( size_t i=0; i<msg->markers.size(); i++ )
  {
    open_marker_idx_.push_back( i );
  }
  for ( size_t i=0; i<msg->poses.size(); i++ )
  {
    open_pose_idx_.push_back( i );
  }
  for( unsigned i=0; i<msg->markers.size(); i++ )
  {
    autoComplete( msg->markers[i], enable_autocomplete_transparency_  );
  }
  for( unsigned i=0; i<msg->poses.size(); i++ )
  {
    // correct empty orientation
    if ( msg->poses[i].pose.orientation.w == 0 && msg->poses[i].pose.orientation.x == 0 &&
        msg->poses[i].pose.orientation.y == 0 && msg->poses[i].pose.orientation.z == 0 )
    {
      msg->poses[i].pose.orientation.w = 1;
    }
  }
}

template<>
void MessageContext<visualization_msgs::msg::InteractiveMarkerInit>::init()
{
  // mark all transforms as being missing
  for ( size_t i=0; i<msg->markers.size(); i++ )
  {
    open_marker_idx_.push_back( i );
  }
  for( unsigned i=0; i<msg->markers.size(); i++ )
  {
    autoComplete( msg->markers[i], enable_autocomplete_transparency_ );
  }
}

template<>
void MessageContext<visualization_msgs::msg::InteractiveMarkerUpdate>::getTfTransforms( )
{
  getTfTransforms( msg->markers, open_marker_idx_ );
  getTfTransforms( msg->poses, open_pose_idx_ );
  if ( isReady() )
  {
    DBG_MSG( "Update message with seq_num=%lu is ready.", msg->seq_num );
  }
}

template<>
void MessageContext<visualization_msgs::msg::InteractiveMarkerInit>::getTfTransforms( )
{
  getTfTransforms( msg->markers, open_marker_idx_ );
  if ( isReady() )
  {
    DBG_MSG( "Init message with seq_num=%lu is ready.", msg->seq_num );
  }
}

// explicit template instantiation
template class MessageContext<visualization_msgs::msg::InteractiveMarkerUpdate>;
template class MessageContext<visualization_msgs::msg::InteractiveMarkerInit>;


}


