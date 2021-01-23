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
