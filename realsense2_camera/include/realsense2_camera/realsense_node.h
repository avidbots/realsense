// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#ifndef REALSENSE2_CAMERA_REALSENSE_NODE_H
#define REALSENSE2_CAMERA_REALSENSE_NODE_H

#include <csignal>
#include <fstream>
#include <atomic>
#include <mutex>

#include <eigen3/Eigen/Geometry>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <realsense2_camera/constants.h>
#include <realsense2_camera/Extrinsics.h>
#include <realsense2_camera/IMUInfo.h>
#include <realsense2_camera/realsense_node.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <iostream>


namespace realsense2_camera
{

class DepthSyncer {

    struct SyncerImplementation {
        SyncerImplementation() {
            bundler = std::make_unique<rs2::processing_block>([&](rs2::frame f, rs2::frame_source& src) {
                    bundle.push_back(f);
                    if (bundle.size() == 2)
            {
                    auto fs = src.allocate_composite_frame(bundle);
                    src.frame_ready(fs);
                    bundle.clear();
        }
        });
        }

        void Start(std::function<void(rs2::frame)> on_frame)
        {
            on_frame_callbak = on_frame;
            bundler->start(q);
        }

        void Invoke(rs2::frame f)
        {
            const auto stream_type = f.get_profile().stream_type();
            if (stream_type == RS2_STREAM_DEPTH) {
                depth.push_back(f);
                got_depth = true;
            }
            if (stream_type == RS2_STREAM_COLOR) {
                color.push_back(f);
                got_color = true;
            }

            if (got_color && got_depth){
                const auto& d = depth.front();
                auto best = color.back();
                auto best_diff = std::abs(d.get_timestamp() - color.back().get_timestamp());
                for (const auto& c : color) {
                    const auto diff = std::abs(d.get_timestamp() - c.get_timestamp());
                    if (diff < best_diff) {
                        best_diff = diff;
                        best = c;
                    }
                }

                if (best_diff < 20 || color.back().get_timestamp() > depth.back().get_timestamp()) {
                    acc(best_diff);

                    ROS_INFO_STREAM("[Sync Error: " << best_diff << "] [Average Sync Error: " << boost::accumulators::mean(acc) <<
                                    "] Variance of Sync Error: " << boost::accumulators::variance(acc) << "]");
                    ROS_INFO_STREAM("Depth queue size " << depth.size());

                    if (depth.size() > 1) {
                      throw std::runtime_error("Depth queue size should not go over 1");
                    }

                    bundler->invoke(d);
                    bundler->invoke(best);

                    depth.clear();
                    color.clear();
                    got_color = false;
                    got_depth = false;
                    rs2::frameset fs = q.wait_for_frame();
                    on_frame_callbak(fs);
                }
            }
        }

        std::vector<rs2::frame> bundle;
        std::unique_ptr<rs2::processing_block> bundler;
        std::function<void(rs2::frame)> on_frame_callbak;
        rs2::frame_queue q;
        std::deque<rs2::frame> depth;
        std::deque<rs2::frame> color;
        bool got_depth = false;
        bool got_color = false;

        boost::accumulators::accumulator_set<double, boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::variance>> acc;
    };

public:
    DepthSyncer(){
        _impl = std::make_shared<SyncerImplementation>();
    }

    DepthSyncer(const DepthSyncer& other)
    {
        _impl = other._impl;
    }

    void start(std::function<void(rs2::frame)> on_frame)
    {
        _impl->Start(on_frame);
    }

    void operator()(rs2::frame f)
    {
        _impl->Invoke(f);
    }
private:
    std::shared_ptr<SyncerImplementation> _impl;

};

class RealSenseParamManagerBase;
template<uint16_t Model>
class RealSenseParamManager;

    enum filters{
        DEPTH_TO_DISPARITY,
        SPATIAL,
        TEMPORAL,
        DISPARITY_TO_DEPTH
    };

    struct FrequencyDiagnostics
    {
      FrequencyDiagnostics(double expected_frequency, std::string name, std::string hardware_id) :
        expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName() + "_" + name)
      {
        ROS_INFO("Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
        diagnostic_updater_.setHardwareID(hardware_id);
        diagnostic_updater_.add(frequency_status_);
      }

      void update()
      {
        frequency_status_.tick();
        diagnostic_updater_.update();
      }

      double expected_frequency_;
      diagnostic_updater::FrequencyStatus frequency_status_;
      diagnostic_updater::Updater diagnostic_updater_;
    };
    typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;

    /**
    Class to encapsulate a filter alongside its options
    */
    class filter_options
    {
    public:
        filter_options(const std::string name, rs2::process_interface &filter);
        filter_options(filter_options&& other);
        std::string filter_name;           // Friendly name of the filter
        rs2::process_interface& filter;    // The filter in use
        std::atomic_bool is_enabled;       // A boolean controlled by the user that determines whether to apply the filter or not
    };

    class RealSenseNode
    {
    public:
        RealSenseNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle);

        void resetNode();
        void getDevice();

        void createParamsManager();

        void publishTopics();
        ~RealSenseNode() {}

        static constexpr stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
        static constexpr stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
        static constexpr stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
        static constexpr stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
        static constexpr stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
        static constexpr stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
        static constexpr stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

    private:
        rs2::context _ctx;
        const uint32_t set_default_dynamic_reconfig_values = 0xffffffff;
        rs2::device _dev;
        std::string _rosbag_filename;
        ros::NodeHandle _node_handle, _pnh;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        rs2::spatial_filter  spat_filter;    // Spatial    - edge-preserving spatial smoothing
        rs2::temporal_filter temp_filter;    // Temporal   - reduces temporal noise
        rs2::disparity_transform depth_to_disparity{true};
        rs2::disparity_transform disparity_to_depth{false};
        std::vector<filter_options> filters;
        std::mutex _mutex;


        struct float3
        {
            float x, y, z;
        };

        struct quaternion
        {
            double x, y, z, w;
        };

        static std::string getNamespaceStr();
        void getParameters();
        bool enableStreams(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);
        void setupDevice();
        void setupPublishers();
        void setupServices();
        void enable_devices();
        void setupStreams();
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        tf::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const ros::Time& t,
                               const float3& trans,
                               const quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void publishStaticTransforms();
        void publishRgbToDepthPCTopic(const ros::Time& t, const std::map<stream_index_pair, bool>& is_frame_arrived);
        void publishDepthPCTopic(const ros::Time& t, const std::map<stream_index_pair, bool>& is_frame_arrived);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;
        rs2_extrinsics getRsExtrinsics(const stream_index_pair& from_stream, const stream_index_pair& to_stream);

        IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void filterFrame(rs2::frame& f);
        void publishFrame(rs2::frame f, const ros::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                          const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                          const std::map<stream_index_pair, std::string>& optical_frame_id,
                          const std::map<stream_index_pair, std::string>& encoding,
                          bool copy_data_from_frame = true);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void updateIsFrameArrived(std::map<stream_index_pair, bool>& is_frame_arrived,
                                  rs2_stream stream_type, int stream_index);

        void publishAlignedDepthToOthers(rs2::frame depth_frame, const std::vector<rs2::frame>& frames, const ros::Time& t);

        void alignFrame(const rs2_intrinsics& from_intrin,
                        const rs2_intrinsics& other_intrin,
                        rs2::frame from_image,
                        uint32_t output_image_bytes_per_pixel,
                        const rs2_extrinsics& from_to_other,
                        std::vector<uint8_t>& out_vec);

        void TemperatureUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat);

        void setHealthTimers();

        
        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<stream_index_pair, bool> _enable;
        std::map<stream_index_pair, std::string> _stream_name;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::map<stream_index_pair, int> _image_format;
        std::map<stream_index_pair, rs2_format> _format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<stream_index_pair, std::string> _encoding;
        std::map<stream_index_pair, std::vector<uint8_t>> _aligned_depth_images;

        std::string _base_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, int> _seq;
        std::map<stream_index_pair, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        bool _intialize_time_base;
        double _camera_time_base;
        double _prev_camera_time_stamp;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        ros::Publisher _pointcloud_xyz_publisher;
        ros::Publisher _pointcloud_xyzrgb_publisher;
        ros::ServiceServer _enable_streams_service;
        ros::Time _ros_time_base;
        bool _align_depth;
        bool _sync_frames;
        bool _pointcloud;
        bool _use_ros_time;
        DepthSyncer _syncer;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, ros::Publisher> _depth_aligned_info_publisher;
        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _depth_aligned_image_publishers;
        std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
        std::map<stream_index_pair, ros::Publisher> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;

        std::function<void(rs2::frame)> _frame_callback;

        std::map<stream_index_pair, bool> _is_frame_arrived;
        const std::string _namespace;

        diagnostic_updater::Updater temp_diagnostic_updater_;
        ros::Timer  temp_update_timer_;
        int temperature_;
        ros::Timer depth_callback_timer_;
        ros::Duration depth_callback_timeout_;
        std::unique_ptr<RealSenseParamManagerBase> _params;

        const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
                                                                            {COLOR},
                                                                            {FISHEYE}}};

        const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

        template <uint16_t Model>
        friend class RealSenseParamManager;

    };  // end class
}  // namespace realsense2_camera

#endif  REALSENSE2_CAMERA_REALSENSE_NODE_H
