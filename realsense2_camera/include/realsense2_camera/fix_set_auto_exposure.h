/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 */
/**
 * @license Apache 2.0. See LICENSE file in root directory.
 * @copyright Copyright 2020, Avidbots Corp.
 * @file      fix_set_auto_exposure.h
 * @brief     Header for fix_set_autoexposure.cpp, A utility to automatically
 * hardware-reset the camera until the auto-exposure setting works.
 * @author  Paul Belanger
 */

#ifndef REALSENSE2_CAMERA_FIX_SET_AUTO_EXPOSURE_H
#define REALSENSE2_CAMERA_FIX_SET_AUTO_EXPOSURE_H

#include <librealsense2/rs.hpp>
#include <ros/duration.h>

namespace realsense2_camera
{

// through experimentation, it takes the D435 sensor about 2 seconds to
// finish its hardware reset.
// unfortunately there isn't a way to know for sure that a device has finished resetting.
// if we wait for less than this time, then we'll end up picking up the original
// sensor before it finishes disconnecting from the USB bus, leading to all sorts of
// issues down the line when we try to pull frames.
const static double MIN_REACQUIRE_WAIT_TIME = 2.5; //[s]

/**
 * @brief Attempt to toggle the autoexposure value of the device.
 * @param device  The device to operate on
 * @param exposure A flag to enable or disable auto exposure. Enabled if true.
 * @return  True if the setting was applied successfully, otherwise false.
 */
static bool try_set_auto_exposure(rs2::device& device, bool exposure);

/**
 * @brief Tries to set autoexposure. If it fails, wait the specified duration and try again.
 * @param device device to operate on
 * @param fail_wait_duration Amount to wait if the first setting fails.
 * @param exposure A flag to enable or disable auto exposure. Enabled if true.
 * @return True if the setting finally succeeds, otherwise false.
 */
static bool try_set_auto_exposure_twice(rs2::device& device, ros::Duration fail_wait_duration, bool exposure);

/**
 * @brief Attempt to reacquire a device with a given serial number.
 * @param context The context used to query devices.
 * @param serial The serial of the device to find.
 * @param[out] out_device Pointer to the device will be placed here.
 * @param max_wait_duration The maximum amount of time to wait for the device
 * to come online.
 * @return True if succcesful
 */
static bool reacquire_device(rs2::context& context, const std::string& serial, rs2::device& out_device, ros::Duration max_wait_duration);

/**
 * @brief Fix the autoexposure setting for the given rs2 device.
 *
 * This function
 * attempts to modify the depth_autoexposure setting on the device. If that fails,
 * the device is issued a hardware reset command and the setting is tried again.
 * This continues until the device begins working properly or the max number of
 * retries is reached.
 *
 * @param context The context that was used to create the device.
 * @param device The device to test. Note: the underlying device pointer may change
 * during the execution of this function.
 * @param reset_wait_duration The amount of time to wait between issuing a
 * hardware reset and attempting to re-probe the device.
 * @param max_resets The maximum number of hardware resets to attempt.
 * @param fail_wait_duration If setting of the ae param fails, we wait this amount
 * of time before trying a second time, and then fall back to the hardware reset
 * if a second attempt fails.
 * @param exposure Auto exposure setting value to be set on the device. Enabled if true.
 * @return True if the device was successfully reset. False if the max number of
 * resets has been attempted but the device still isn't responding.
 *
 * @note This function always unconditionally performs at least one hardware_reset()
 * of the camera.
 */
bool fixSetAutoExposure(rs2::context& context, rs2::device& device, ros::Duration reset_wait_duration, int max_resets,
                       ros::Duration fail_wait_duration, bool exposure);

} // namespace realsense2_camera
#endif // REALSENSE2_CAMERA_FIX_SET_AUTO_EXPOSURE_H
