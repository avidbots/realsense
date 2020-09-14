#ifndef REALSENSE2_CAMERA_PARAM_MANAGER_H
#define REALSENSE2_CAMERA_PARAM_MANAGER_H

#include <dynamic_reconfigure/server.h>
#include <realsense2_camera/base_d400_paramsConfig.h>
#include <realsense2_camera/rs415_paramsConfig.h>
#include <realsense2_camera/rs435_paramsConfig.h>
#include <realsense2_camera/sr300_paramsConfig.h>
#include <realsense2_camera/realsense_node.h>

namespace realsense2_camera
{

enum base_depth_param{
    base_depth_gain = 1,
    base_depth_enable_auto_exposure,
    base_depth_visual_preset,
    base_depth_frames_queue_size,
    base_depth_error_polling_enabled,
    base_depth_output_trigger_enabled,
    base_depth_units,
    base_JSON_file_path,
    base_enable_depth_to_disparity_filter,
    base_enable_spatial_filter,
    base_enable_temporal_filter,
    base_enable_disparity_to_depth_filter,
    base_spatial_filter_magnitude,
    base_spatial_filter_smooth_alpha,
    base_spatial_filter_smooth_delta,
    base_spatial_filter_holes_fill,
    base_temporal_filter_smooth_alpha,
    base_temporal_filter_smooth_delta,
    base_temporal_filter_holes_fill,
    base_depth_count
};

enum rs435_param{
    rs435_depth_exposure = 20,
    rs435_depth_laser_power,
    rs435_depth_emitter_enabled,
    rs435_color_backlight_compensation,
    rs435_color_brightness,
    rs435_color_contrast,
    rs435_color_exposure,
    rs435_color_gain,
    rs435_color_gamma,
    rs435_color_hue,
    rs435_color_saturation,
    rs435_color_sharpness,
    rs435_color_white_balance,
    rs435_color_enable_auto_exposure,
    rs435_color_enable_auto_white_balance,
    rs435_color_frames_queue_size,
    rs435_color_power_line_frequency,
    rs435_color_auto_exposure_priority,
    rs435_param_count
};

enum rs415_param{
    rs415_depth_enable_auto_white_balance = 20,
    rs415_depth_exposure,
    rs415_depth_laser_power,
    rs415_depth_emitter_enabled,
    rs415_color_backlight_compensation,
    rs415_color_brightness,
    rs415_color_contrast,
    rs415_color_exposure,
    rs415_color_gain,
    rs415_color_gamma,
    rs415_color_hue,
    rs415_color_saturation,
    rs415_color_sharpness,
    rs415_color_white_balance,
    rs415_color_enable_auto_exposure,
    rs415_color_enable_auto_white_balance,
    rs415_color_frames_queue_size,
    rs415_color_power_line_frequency,
    rs415_color_auto_exposure_priority,
    rs415_param_count
};

enum sr300_param{
    sr300_param_color_backlight_compensation = 1,
    sr300_param_color_brightness,
    sr300_param_color_contrast,
    sr300_param_color_gain,
    sr300_param_color_gamma,
    sr300_param_color_hue,
    sr300_param_color_saturation,
    sr300_param_color_sharpness,
    sr300_param_color_white_balance,
    sr300_param_color_enable_auto_white_balance,
    sr300_param_color_exposure,
    sr300_param_color_enable_auto_exposure,
    sr300_param_depth_visual_preset,
    sr300_param_depth_laser_power,
    sr300_param_depth_accuracy,
    sr300_param_depth_motion_range,
    sr300_param_depth_filter_option,
    sr300_param_depth_confidence_threshold,
    sr300_param_depth_frames_queue_size,
    sr300_param_depth_units,
    sr300_param_count
};


class RealSenseParamManagerBase {
public:
    virtual ~RealSenseParamManagerBase() {};
    virtual void registerDynamicReconfigCb(RealSenseNode *node_ptr) = 0;
};


template<uint16_t Model>
struct ModelTraits {};

template<>
struct ModelTraits<RS400_PID> {
 using Config = base_d400_paramsConfig;
 using Param = base_depth_param;
 static const auto param_count = base_depth_param::base_depth_count;
};

template<> struct ModelTraits<RS405_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS410_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS460_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS420_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS420_MM_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS430_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS430_MM_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS430_MM_RGB_PID> : ModelTraits<RS400_PID> {};
template<> struct ModelTraits<RS_USB2_PID> : ModelTraits<RS400_PID> {};

template<>
struct ModelTraits<RS415_PID> {
 using Config = rs415_paramsConfig;
 using Param = rs415_param;
 static const auto param_count = rs415_param::rs415_param_count;
};

template<>
struct ModelTraits<RS435_RGB_PID> {
  using Config =rs435_paramsConfig;
  using Param = rs435_param;
  static const auto param_count = rs435_param::rs435_param_count;
};

template<>
struct ModelTraits<SR300_PID> {
  using Config = sr300_paramsConfig;
  using Param = sr300_param;
  static const auto param_count = sr300_param::sr300_param_count;
};


template<uint16_t Model>
class RealSenseParamManager : public RealSenseParamManagerBase
{
public:
    using Param = typename ModelTraits<Model>::Param;
    void setParam(RealSenseNode* node_ptr, typename ModelTraits<Model>::Config &config, Param param);
    virtual void registerDynamicReconfigCb(RealSenseNode *node_ptr) override;

private:
    void callback(RealSenseNode* node_ptr, typename ModelTraits<Model>::Config &config, uint32_t level);
    void setOption(RealSenseNode *node_ptr, stream_index_pair sip, rs2_option opt, float val);

    std::shared_ptr<dynamic_reconfigure::Server<typename ModelTraits<Model>::Config>> _server;
    typename dynamic_reconfigure::Server<typename ModelTraits<Model>::Config>::CallbackType _f;
};


template<uint16_t Model>
void  RealSenseParamManager<Model>::callback(RealSenseNode* node_ptr, typename ModelTraits<Model>::Config &config, uint32_t level)
{
    if (node_ptr->set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1 ; i < ModelTraits<Model>::param_count ; ++i)
        {
            setParam(node_ptr, config , static_cast<Param>(i));
        }
    }
    else
    {
        setParam(node_ptr, config, static_cast<Param>(level));
    }
}

using ParamManagerMaker = std::function<std::unique_ptr<RealSenseParamManagerBase>()>;
template<uint16_t Model>
using RSPM =  RealSenseParamManager<Model>;

const std::map<uint16_t, ParamManagerMaker> param_makers =
{
    {RS400_PID, [](){return std::unique_ptr<RSPM<RS400_PID>>(new RSPM<RS400_PID>);}},
    {RS405_PID, [](){return std::unique_ptr<RSPM<RS405_PID>>(new RSPM<RS405_PID>);}},
    {RS410_PID, [](){return std::unique_ptr<RSPM<RS410_PID>>(new RSPM<RS410_PID>);}},
    {RS460_PID, [](){return std::unique_ptr<RSPM<RS460_PID>>(new RSPM<RS460_PID>);}},
    {RS420_PID, [](){return std::unique_ptr<RSPM<RS420_PID>>(new RSPM<RS420_PID>);}},
    {RS420_MM_PID, [](){return std::unique_ptr<RSPM<RS420_MM_PID>>(new RSPM<RS420_MM_PID>);}},
    {RS430_PID, [](){return std::unique_ptr<RSPM<RS430_PID>>(new RSPM<RS430_PID>);}},
    {RS430_MM_PID, [](){return std::unique_ptr<RSPM<RS430_MM_PID>>(new RSPM<RS430_MM_PID>);}},
    {RS430_MM_RGB_PID, [](){return std::unique_ptr<RSPM<RS430_MM_RGB_PID>>(new RSPM<RS430_MM_RGB_PID>);}},
    {RS_USB2_PID, [](){return std::unique_ptr<RSPM<RS_USB2_PID>>(new RSPM<RS_USB2_PID>);}},
    {RS415_PID, [](){return std::unique_ptr<RSPM<RS415_PID>>(new RSPM<RS415_PID>);}},
    {RS435_RGB_PID, [](){return std::unique_ptr<RSPM<RS435_RGB_PID>>(new RSPM<RS435_RGB_PID>);}},
    {SR300_PID, [](){return std::unique_ptr<RSPM<SR300_PID>>(new RSPM<SR300_PID>);}}
};

}
#endif // REALSENSE2_CAMERA_PARAM_MANAGER_H
