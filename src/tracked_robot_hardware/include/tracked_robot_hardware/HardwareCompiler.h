// Copyright 2021 ros2_control Development Team
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef TRACKED_ROBOT_HARDWARE__HARDWARE_COMPILER_H_
#define TRACKED_ROBOT_HARDWARE__HARDWARE_COMPILER_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TRACKED_ROBOT_HARDWARE_EXPORT __attribute__((dllexport))
#define TRACKED_ROBOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define TRACKED_ROBOT_HARDWARE_EXPORT __declspec(dllexport)
#define TRACKED_ROBOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef TRACKED_ROBOT_HARDWARE_BUILDING_DLL
#define TRACKED_ROBOT_HARDWARE_PUBLIC DIFFDRIVE_ARDUINO_EXPORT
#else
#define TRACKED_ROBOT_HARDWARE_PUBLIC DIFFDRIVE_ARDUINO_IMPORT
#endif
#define TRACKED_ROBOT_HARDWARE_PUBLIC_TYPE DIFFDRIVE_ARDUINO_PUBLIC
#define TRACKED_ROBOT_HARDWARE_LOCAL
#else
#define TRACKED_ROBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define TRACKED_ROBOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define TRACKED_ROBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define TRACKED_ROBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define TRACKED_ROBOT_HARDWARE_PUBLIC
#define TRACKED_ROBOT_HARDWARE_LOCAL
#endif
#define TRACKED_ROBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // TRACKED_ROBOT_HARDWARE__HARDWARE_COMPILER_H_
