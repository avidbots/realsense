// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#ifndef REALSENSE2_CAMERA_REALSENSE_NODELET_H
#define REALSENSE2_CAMERA_REALSENSE_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <memory>

#include <realsense2_camera/realsense_node.h>

namespace realsense2_camera
{
    inline void signalHandler(int signum)
    {
        ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating RealSense Node...");
        ros::shutdown();
        exit(signum);
    }

    class RealSenseNodelet : public nodelet::Nodelet
    {
    public:
        RealSenseNodelet();
        virtual ~RealSenseNodelet() {}

    private:
        virtual void onInit() override;
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        std::unique_ptr<RealSenseNode> _realSenseNode;
    };
}  // namespace realsense2_camera

#endif  // REALSENSE2_CAMERA_REALSENSE_NODE_H
