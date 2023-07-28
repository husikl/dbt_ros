// Copyright (c) 2018 Intel Corporation
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

#ifndef BT_CONVERSIONS_HPP_
#define BT_CONVERSIONS_HPP_

#include <string>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.


template<>
inline geometry_msgs::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}


template<>
inline geometry_msgs::Quaternion convertFromString(const StringView key)
{
  // four real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}


template<>
inline geometry_msgs::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 8) {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[0]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[1]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[7]);
    return pose_stamped;
  }
}

template<>
inline geometry_msgs::Pose convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 7) {
    throw std::runtime_error("invalid number of fields for Pose attribute)");
  } else {
    geometry_msgs::Pose pose;
    pose.position.x = BT::convertFromString<double>(parts[0]);
    pose.position.y = BT::convertFromString<double>(parts[1]);
    pose.position.z = BT::convertFromString<double>(parts[2]);
    pose.orientation.x = BT::convertFromString<double>(parts[3]);
    pose.orientation.y = BT::convertFromString<double>(parts[4]);
    pose.orientation.z = BT::convertFromString<double>(parts[5]);
    pose.orientation.w = BT::convertFromString<double>(parts[6]);
    // std::cout << "converted pose " << pose << std::endl;
    return pose;
  }
}

template<>
inline geometry_msgs::PoseArray convertFromString(const StringView key) {
    auto poseStrings = BT::splitString(key, ';');
    std::size_t numPoses = poseStrings.size();

    geometry_msgs::PoseArray poseArray;
    poseArray.poses.resize(numPoses);

    for (std::size_t i = 0; i < numPoses; ++i) {
        auto poseString = poseStrings[i];
        auto parts = BT::splitString(poseString, ',');

        if (parts.size() != 7) {
            throw std::runtime_error("Invalid number of fields for Pose attribute");
        } else {
            geometry_msgs::Pose& pose = poseArray.poses[i];
            pose.position.x = BT::convertFromString<double>(parts[0]);
            pose.position.y = BT::convertFromString<double>(parts[1]);
            pose.position.z = BT::convertFromString<double>(parts[2]);
            pose.orientation.x = BT::convertFromString<double>(parts[3]);
            pose.orientation.y = BT::convertFromString<double>(parts[4]);
            pose.orientation.z = BT::convertFromString<double>(parts[5]);
            pose.orientation.w = BT::convertFromString<double>(parts[6]);
        }
    }

    return poseArray;
}




template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

}  // namespace BT

#endif  // BT_CONVERSIONS_HPP_
