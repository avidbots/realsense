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
 * @file      fix_set_auto_exposure.cpp
 * @brief     Brief description of file.
 * @author    Paul Belanger
 */

// most of this code is adapted from the fix_set_autoexposure script in the
// avidbots robot repo.

#include <realsense2_camera/fix_set_auto_exposure.h>
#include <ros/console.h>
#include <boost/interprocess/sync/named_mutex.hpp>

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
static bool try_set_auto_exposure(rs2::device& device, bool exposure)
{
  const auto option = rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE;

  rs2::sensor stereo_module;
  bool depth_sensor_found = false;
  for(auto& sen: device.query_sensors())
  {
    if(sen.is<rs2::depth_sensor>())
    {
      depth_sensor_found = true;
      stereo_module = sen;
    }
  }
  if(! depth_sensor_found)
  {
    ROS_ERROR("Could not find a depth sensor on the device! ");
    return false;
  }

  try
  {
    const float original_value = stereo_module.get_option(option);
    const float expected_new_value = (exposure ? 1.0f : 0.0f);
    ROS_DEBUG_STREAM("Set autoexposure " << original_value << " -> " << expected_new_value);
    stereo_module.set_option(option, expected_new_value);
    const float actual_new_value = stereo_module.get_option(option);
    ROS_DEBUG_STREAM("Actual autoexposure value: " << actual_new_value);
    return (actual_new_value == expected_new_value);
  }
  catch(const rs2::error& ex)
  {
    ROS_WARN_STREAM("try_set_autoexposure failed: " << ex.what());
    return false;
  }
}

/**
 * @brief Tries to set autoexposure. If it fails, wait the specified duration and try again.
 * @param device device to operate on
 * @param fail_wait_duration Amount to wait if the first setting fails.
 * @param exposure A flag to enable or disable auto exposure. Enabled if true.
 * @return True if the setting finally succeeds, otherwise false.
 */
static bool try_set_auto_exposure_twice(rs2::device& device, ros::Duration fail_wait_duration, bool exposure)
{
  if(try_set_auto_exposure(device, exposure))
  {
    return true;
  }
  else
  {
    fail_wait_duration.sleep();
    return try_set_auto_exposure(device, exposure);
  }
}

/**
 * @brief Attempt to reacquire a device with a given serial number.
 * @param context The context used to query devices.
 * @param serial The serial of the device to find.
 * @param[out] out_device Pointer to the device will be placed here.
 * @param max_wait_duration The maximum amount of time to wait for the device
 * to come online.
 * @return True if succcesful
 */
static bool reacquire_device(rs2::context& context, const std::string& serial, rs2::device& out_device, ros::Duration max_wait_duration)
{
  ros::Duration amount_waited_so_far(0.0);

  try
  {
    while(amount_waited_so_far < max_wait_duration)
    {
      // we always sleep first to allow the device time to drop off the bus so
      // we don't accidentally acquire the disconnecting device.
      auto sleep_time = std::min(ros::Duration(MIN_REACQUIRE_WAIT_TIME), max_wait_duration - amount_waited_so_far);
      amount_waited_so_far += sleep_time;
      sleep_time.sleep();

      namespace bi = boost::interprocess;
      bi::named_mutex usb_mutex{bi::open_or_create, "usb_mutex"};
      usb_mutex.lock();
      auto devlist = context.query_devices();
      usb_mutex.unlock();

      for(auto dev: devlist)
      {
        std::string dev_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if(dev_serial == serial)
        {
          out_device = dev;
          return true;
        }
      }
      // no device with matching serial found. Try again after a short delay.
    }
  }
  catch(const rs2::error& e)
  {
    ROS_FATAL_STREAM("Error when enumerating devices: " << e.what());
  }
  // no device found after waiting the max duration.
  return false;
}

/**
 * @brief Attempt to fix auto exposure setting by reset hardware and checking the actual configuration on devices.
 * @param context The context used to query devices.
 * @param serial The serial of the device to find.
 * @param reset_wait_duration Time to wait after reset for the device to come back online.
 * @param max_resets The maximum number of times to try resetting a device.
 * @param fail_wait_duration Time to wait before another attempt to try fixing auto exposure setting.
 * @param exposure Auto exposure setting value to be set on the device. Enabled if true.
 * @return True if succcesful.
 */
bool fixSetAutoExposure(rs2::context& context, rs2::device &device, ros::Duration reset_wait_duration,
                        int max_resets, ros::Duration fail_wait_duration, bool exposure)
{
  // since the hardware_reset causes the device to re-enumerate on the USB
  // bus, we need to track it by its unique serial number across resets.
  std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  for(int i = 0; i < max_resets; i++)
  {
    // changing the setting failed, so reset the hardware and try again.
    ROS_INFO("Resetting device...");
    try
    {
      device.hardware_reset();
    }
    catch(const rs2::error& ex)
    {
      ROS_FATAL_STREAM("hardware reset of device " << serial << " failed! ");
      return false;
    }
    ROS_INFO("Reacquiring...");
    if( ! reacquire_device(context, serial, device, reset_wait_duration))
    {
      // device couldn't be reacquired.
      ROS_FATAL_STREAM("Could not reacquire device " << serial << "after a hardware reset! ");
      return false;
    }

    ROS_INFO("Reacquired! ");

    if(try_set_auto_exposure_twice(device, fail_wait_duration, exposure))
    {
      // success!
      return true;
    }
  }

  ROS_FATAL_STREAM("Device " << serial << " did not respond after " << max_resets << "attempts! ");
  return false;
}

} // namespace realsense2_camera
