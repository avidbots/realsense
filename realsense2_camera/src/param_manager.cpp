#include <realsense2_camera/param_manager.h>

namespace realsense2_camera
{

template<uint16_t Model>
void RealSenseParamManager<Model>::registerDynamicReconfigCb(RealSenseNode *node_ptr)
{
    _server = std::make_shared<dynamic_reconfigure::Server<typename ModelTraits<Model>::Config>>();
    _f = boost::bind(&RealSenseParamManager<Model>::callback, this, node_ptr, _1, _2);
    _server->setCallback(_f);
}

template<uint16_t Model>
void RealSenseParamManager<Model>::setOption(RealSenseNode* node_ptr,stream_index_pair sip, rs2_option opt, float val)
{
    node_ptr->_sensors[sip].set_option(opt, val);
}


template<>
void RealSenseParamManager<SR300_PID>::setParam(RealSenseNode* node_ptr, typename ModelTraits<SR300_PID>::Config &config, Param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case sr300_param_color_backlight_compensation:
        ROS_DEBUG_STREAM("sr300_param_color_backlight_compensation: " << config.sr300_color_backlight_compensation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.sr300_color_backlight_compensation);
        break;
    case sr300_param_color_brightness:
        ROS_DEBUG_STREAM("sr300_color_backlight_compensation: " << config.sr300_color_backlight_compensation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.sr300_color_brightness);
        break;
    case sr300_param_color_contrast:
        ROS_DEBUG_STREAM("sr300_param_color_contrast: " << config.sr300_color_contrast);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.sr300_color_contrast);
        break;
    case sr300_param_color_gain:
        ROS_DEBUG_STREAM("sr300_param_color_gain: " << config.sr300_color_gain);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.sr300_color_gain);
        break;
    case sr300_param_color_gamma:
        ROS_DEBUG_STREAM("sr300_param_color_gain: " << config.sr300_color_gamma);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.sr300_color_gamma);
        break;
    case sr300_param_color_hue:
        ROS_DEBUG_STREAM("sr300_param_color_hue: " << config.sr300_color_hue);
       node_ptr-> _sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.sr300_color_hue);
        break;
    case sr300_param_color_saturation:
        ROS_DEBUG_STREAM("sr300_param_color_saturation: " << config.sr300_color_saturation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.sr300_color_saturation);
        break;
    case sr300_param_color_sharpness:
        ROS_DEBUG_STREAM("sr300_param_color_sharpness: " << config.sr300_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.sr300_color_sharpness);
        break;
    case sr300_param_color_white_balance:
        ROS_DEBUG_STREAM("sr300_param_color_white_balance: " << config.sr300_color_white_balance);
        if (node_ptr->_sensors[RealSenseNode::COLOR].get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
            node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);

        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.sr300_color_white_balance);
        break;
    case sr300_param_color_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs435_depth_emitter_enabled: " << config.sr300_color_enable_auto_white_balance);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.sr300_color_enable_auto_white_balance);
        break;
    case sr300_param_color_exposure:
        ROS_DEBUG_STREAM("sr300_param_color_exposure: " << config.sr300_color_exposure);
        if (node_ptr->_sensors[RealSenseNode::COLOR].get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.sr300_color_exposure);
        break;
    case sr300_param_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_color_enable_auto_white_balance);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.sr300_color_enable_auto_exposure);
        break;
    case sr300_param_depth_visual_preset:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_depth_visual_preset);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, config.sr300_depth_visual_preset);
        break;
    case sr300_param_depth_laser_power:
        ROS_DEBUG_STREAM("sr300_param_color_enable_auto_exposure: " << config.sr300_depth_laser_power);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.sr300_depth_laser_power);
        break;
    case sr300_param_depth_accuracy:
        ROS_DEBUG_STREAM("sr300_param_depth_accuracy: " << config.sr300_depth_accuracy);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_ACCURACY, config.sr300_depth_accuracy);
        break;
    case sr300_param_depth_motion_range:
        ROS_DEBUG_STREAM("sr300_param_depth_motion_range: " << config.sr300_depth_motion_range);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_MOTION_RANGE, config.sr300_depth_motion_range);
        break;
    case sr300_param_depth_filter_option:
        ROS_DEBUG_STREAM("sr300_param_depth_filter_option: " << config.sr300_depth_filter_option);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_FILTER_OPTION, config.sr300_depth_filter_option);
        break;
    case sr300_param_depth_confidence_threshold:
        ROS_DEBUG_STREAM("sr300_param_depth_confidence_threshold: " << config.sr300_depth_confidence_threshold);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_CONFIDENCE_THRESHOLD, config.sr300_depth_confidence_threshold);
        break;
    case sr300_param_depth_frames_queue_size:
        ROS_DEBUG_STREAM("sr300_param_depth_frames_queue_size: " << config.sr300_depth_frames_queue_size);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.sr300_depth_frames_queue_size);
        break;
    case sr300_param_depth_units:
        break;
    default:
            ROS_WARN_STREAM("Unrecognized sr300 param (" << param << ")");
        break;
    }
}

template<>
void RealSenseParamManager<RS400_PID>::setParam(RealSenseNode* node_ptr, typename ModelTraits<RS400_PID>::Config &config, Param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case base_depth_gain:
        ROS_DEBUG_STREAM("base_depth_gain: " << config.base_depth_gain);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_GAIN, config.base_depth_gain);
        break;
    case base_depth_enable_auto_exposure:
        ROS_DEBUG_STREAM("base_depth_enable_auto_exposure: " << config.base_depth_enable_auto_exposure);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.base_depth_enable_auto_exposure);
        break;
    case base_depth_visual_preset:
        ROS_DEBUG_STREAM("base_depth_visual_preset: " << config.base_depth_visual_preset);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_VISUAL_PRESET, config.base_depth_visual_preset);
        break;
    case base_depth_frames_queue_size:
        ROS_DEBUG_STREAM("base_depth_frames_queue_size: " << config.base_depth_frames_queue_size);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_FRAMES_QUEUE_SIZE, config.base_depth_frames_queue_size);
        break;
    case base_depth_error_polling_enabled:
        ROS_DEBUG_STREAM("base_depth_error_polling_enabled: " << config.base_depth_error_polling_enabled);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_ERROR_POLLING_ENABLED, config.base_depth_error_polling_enabled);
        break;
    case base_depth_output_trigger_enabled:
        ROS_DEBUG_STREAM("base_depth_output_trigger_enabled: " << config.base_depth_output_trigger_enabled);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_OUTPUT_TRIGGER_ENABLED, config.base_depth_output_trigger_enabled);
        break;
    case base_depth_units:
        break;
    case base_JSON_file_path:
    {
        ROS_DEBUG_STREAM("base_JSON_file_path: " << config.base_JSON_file_path);
        auto adv_dev = node_ptr->_dev.as<rs400::advanced_mode>();
        if (!adv_dev)
        {
            ROS_WARN_STREAM("Device doesn't support Advanced Mode!");
            return;
        }
        if (!config.base_JSON_file_path.empty())
        {
            std::ifstream in(config.base_JSON_file_path);
            if (!in.is_open())
            {
                ROS_WARN_STREAM("JSON file provided doesn't exist!");
                return;
            }

            adv_dev.load_json(config.base_JSON_file_path);
        }
        break;
    }
    case base_enable_depth_to_disparity_filter:
        ROS_DEBUG_STREAM("base_enable_depth_to_disparity_filter: " << config.base_enable_depth_to_disparity_filter);
        node_ptr->filters[DEPTH_TO_DISPARITY].is_enabled = config.base_enable_depth_to_disparity_filter;
        break;
    case base_enable_spatial_filter:
        ROS_DEBUG_STREAM("base_enable_spatial_filter: " << config.base_enable_spatial_filter);
        node_ptr->filters[SPATIAL].is_enabled = config.base_enable_spatial_filter;
        break;
    case base_enable_temporal_filter:
        ROS_DEBUG_STREAM("base_enable_temporal_filter: " << config.base_enable_temporal_filter);
        node_ptr->filters[TEMPORAL].is_enabled = config.base_enable_temporal_filter;
        break;
    case base_enable_disparity_to_depth_filter:
        ROS_DEBUG_STREAM("base_enable_disparity_to_depth_filter: " << config.base_enable_disparity_to_depth_filter);
        node_ptr->filters[DISPARITY_TO_DEPTH].is_enabled = config.base_enable_disparity_to_depth_filter;
        break;
    case base_spatial_filter_magnitude:
        ROS_DEBUG_STREAM("base_spatial_filter_magnitude: " << config.base_spatial_filter_magnitude);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, config.base_spatial_filter_magnitude);
        break;
    case base_spatial_filter_smooth_alpha:
        ROS_DEBUG_STREAM("base_spatial_filter_smooth_alpha: " << config.base_spatial_filter_smooth_alpha);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config.base_spatial_filter_smooth_alpha);
        break;
    case base_spatial_filter_smooth_delta:
        ROS_DEBUG_STREAM("base_spatial_filter_smooth_delta: " << config.base_spatial_filter_smooth_delta);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config.base_spatial_filter_smooth_delta);
        break;
    case base_spatial_filter_holes_fill:
        ROS_DEBUG_STREAM("base_spatial_filter_holes_fill: " << config.base_spatial_filter_holes_fill);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_HOLES_FILL, config.base_spatial_filter_holes_fill);
        break;
    case base_temporal_filter_smooth_alpha:
        ROS_DEBUG_STREAM("base_temporal_filter_smooth_alpha: " << config.base_temporal_filter_smooth_alpha);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config.base_temporal_filter_smooth_alpha);
        break;
    case base_temporal_filter_smooth_delta:
        ROS_DEBUG_STREAM("base_temporal_filter_smooth_delta: " << config.base_temporal_filter_smooth_delta);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config.base_temporal_filter_smooth_delta);
        break;
    case base_temporal_filter_holes_fill:
        ROS_DEBUG_STREAM("base_temporal_filter_holes_fill: " << config.base_temporal_filter_holes_fill);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_HOLES_FILL, config.base_temporal_filter_holes_fill);
        break;
    default:
        ROS_WARN_STREAM("Unrecognized D400 param (" << param << ")");
        break;
    }
}


template<>
void RealSenseParamManager<RS435_RGB_PID>::setParam(RealSenseNode* node_ptr, typename ModelTraits<RS435_RGB_PID>::Config &config, Param param)
{
    // W/O for zero param
    if (0 == param)
        return;
    switch (param) {
    if (0 == param)
        return;

    switch (param) {
    case base_depth_gain:
        ROS_DEBUG_STREAM("base_depth_gain: " << config.rs435_depth_gain);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_GAIN, config.rs435_depth_gain);
        break;
    case base_depth_enable_auto_exposure:
        ROS_DEBUG_STREAM("base_depth_enable_auto_exposure: " << config.rs435_depth_enable_auto_exposure);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.rs435_depth_enable_auto_exposure);
        break;
    case base_depth_visual_preset:
        ROS_DEBUG_STREAM("base_depth_visual_preset: " << config.rs435_depth_visual_preset);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_VISUAL_PRESET, config.rs435_depth_visual_preset);
        break;
    case base_depth_frames_queue_size:
        ROS_DEBUG_STREAM("base_depth_frames_queue_size: " << config.rs435_depth_frames_queue_size);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_FRAMES_QUEUE_SIZE, config.rs435_depth_frames_queue_size);
        break;
    case base_depth_error_polling_enabled:
        ROS_DEBUG_STREAM("base_depth_error_polling_enabled: " << config.rs435_depth_error_polling_enabled);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_ERROR_POLLING_ENABLED, config.rs435_depth_error_polling_enabled);
        break;
    case base_depth_output_trigger_enabled:
        ROS_DEBUG_STREAM("base_depth_output_trigger_enabled: " << config.rs435_depth_output_trigger_enabled);
        setOption(node_ptr, RealSenseNode::DEPTH, RS2_OPTION_OUTPUT_TRIGGER_ENABLED, config.rs435_depth_output_trigger_enabled);
        break;
    case base_depth_units:
        break;
    case base_JSON_file_path:
    {
        ROS_DEBUG_STREAM("base_JSON_file_path: " << config.rs435_JSON_file_path);
        auto adv_dev = node_ptr->_dev.as<rs400::advanced_mode>();
        if (!adv_dev)
        {
            ROS_WARN_STREAM("Device doesn't support Advanced Mode!");
            return;
        }
        if (!config.rs435_JSON_file_path.empty())
        {
            std::ifstream in(config.rs435_JSON_file_path);
            if (!in.is_open())
            {
                ROS_WARN_STREAM("JSON file provided doesn't exist!");
                return;
            }

            adv_dev.load_json(config.rs435_JSON_file_path);
        }
        break;
    }
    case base_enable_depth_to_disparity_filter:
        ROS_DEBUG_STREAM("base_enable_depth_to_disparity_filter: " << config.rs435_enable_depth_to_disparity_filter);
        node_ptr->filters[DEPTH_TO_DISPARITY].is_enabled = config.rs435_enable_depth_to_disparity_filter;
        break;
    case base_enable_spatial_filter:
        ROS_DEBUG_STREAM("base_enable_spatial_filter: " << config.rs435_enable_spatial_filter);
        node_ptr->filters[SPATIAL].is_enabled = config.rs435_enable_spatial_filter;
        break;
    case base_enable_temporal_filter:
        ROS_DEBUG_STREAM("base_enable_temporal_filter: " << config.rs435_enable_temporal_filter);
        node_ptr->filters[TEMPORAL].is_enabled = config.rs435_enable_temporal_filter;
        break;
    case base_enable_disparity_to_depth_filter:
        ROS_DEBUG_STREAM("base_enable_disparity_to_depth_filter: " << config.rs435_enable_disparity_to_depth_filter);
        node_ptr->filters[DISPARITY_TO_DEPTH].is_enabled = config.rs435_enable_disparity_to_depth_filter;
        break;
    case base_spatial_filter_magnitude:
        ROS_DEBUG_STREAM("base_spatial_filter_magnitude: " << config.rs435_spatial_filter_magnitude);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, config.rs435_spatial_filter_magnitude);
        break;
    case base_spatial_filter_smooth_alpha:
        ROS_DEBUG_STREAM("base_spatial_filter_smooth_alpha: " << config.rs435_spatial_filter_smooth_alpha);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config.rs435_spatial_filter_smooth_alpha);
        break;
    case base_spatial_filter_smooth_delta:
        ROS_DEBUG_STREAM("base_spatial_filter_smooth_delta: " << config.rs435_spatial_filter_smooth_delta);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config.rs435_spatial_filter_smooth_delta);
        break;
    case base_spatial_filter_holes_fill:
        ROS_DEBUG_STREAM("base_spatial_filter_holes_fill: " << config.rs435_spatial_filter_holes_fill);
        node_ptr->filters[SPATIAL].filter.set_option(RS2_OPTION_HOLES_FILL, config.rs435_spatial_filter_holes_fill);
        break;
    case base_temporal_filter_smooth_alpha:
        ROS_DEBUG_STREAM("base_temporal_filter_smooth_alpha: " << config.rs435_temporal_filter_smooth_alpha);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config.rs435_temporal_filter_smooth_alpha);
        break;
    case base_temporal_filter_smooth_delta:
        ROS_DEBUG_STREAM("base_temporal_filter_smooth_delta: " << config.rs435_temporal_filter_smooth_delta);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config.rs435_temporal_filter_smooth_delta);
        break;
    case base_temporal_filter_holes_fill:
        ROS_DEBUG_STREAM("base_temporal_filter_holes_fill: " << config.rs435_temporal_filter_holes_fill);
        node_ptr->filters[TEMPORAL].filter.set_option(RS2_OPTION_HOLES_FILL, config.rs435_temporal_filter_holes_fill);
        break;
    }
    case rs435_color_backlight_compensation:
        ROS_DEBUG_STREAM("rs435_color_backlight_compensation: " << config.rs435_color_backlight_compensation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.rs435_color_backlight_compensation);
        break;
    case rs435_color_brightness:
        ROS_DEBUG_STREAM("rs435_color_brightness: " << config.rs435_color_brightness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.rs435_color_brightness);
        break;
    case rs435_color_contrast:
        ROS_DEBUG_STREAM("rs435_color_contrast: " << config.rs435_color_contrast);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.rs435_color_contrast);
        break;
    case rs435_color_gain:
        ROS_DEBUG_STREAM("rs435_color_gain: " << config.rs435_color_gain);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.rs435_color_gain);
        break;
    case rs435_color_gamma:
        ROS_DEBUG_STREAM("rs435_color_gamma: " << config.rs435_color_gamma);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.rs435_color_gamma);
        break;
    case rs435_color_hue:
        ROS_DEBUG_STREAM("rs435_color_hue: " << config.rs435_color_hue);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.rs435_color_hue);
        break;
    case rs435_color_saturation:
        ROS_DEBUG_STREAM("rs435_color_saturation: " << config.rs435_color_saturation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.rs435_color_saturation);
        break;
    case rs435_color_sharpness:
        ROS_DEBUG_STREAM("rs435_color_sharpness: " << config.rs435_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.rs435_color_sharpness);
        break;
    case rs435_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("rs435_color_enable_auto_exposure: " << config.rs435_color_enable_auto_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.rs435_color_enable_auto_exposure);
        break;
    case rs435_color_exposure:
        ROS_DEBUG_STREAM("rs435_color_exposure: " << config.rs435_color_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs435_color_exposure);
        break;
    case rs435_color_white_balance:
        ROS_DEBUG_STREAM("rs435_color_white_balance: " << config.rs435_color_white_balance * 10);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.rs435_color_white_balance * 10);
        break;
    case rs435_color_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs435_color_enable_auto_white_balance: " << config.rs435_color_enable_auto_white_balance);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.rs435_color_enable_auto_white_balance);
        break;
    case rs435_color_frames_queue_size:
        ROS_DEBUG_STREAM("rs435_color_frames_queue_size: " << config.rs435_color_frames_queue_size);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.rs435_color_frames_queue_size);
        break;
    case rs435_color_power_line_frequency:
        ROS_DEBUG_STREAM("rs435_color_power_line_frequency: " << config.rs435_color_power_line_frequency);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_POWER_LINE_FREQUENCY, config.rs435_color_power_line_frequency);
        break;
    case rs435_color_auto_exposure_priority:
        ROS_DEBUG_STREAM("rs435_color_auto_exposure_priority: " << config.rs435_color_auto_exposure_priority);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_PRIORITY, config.rs435_color_auto_exposure_priority);
        break;
    case rs435_depth_exposure:
    {
        static const auto rs435_depth_exposure_factor = 20;
        ROS_DEBUG_STREAM("rs435_depth_exposure: " << config.rs435_depth_exposure * rs435_depth_exposure_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs435_depth_exposure * rs435_depth_exposure_factor);
    }
        break;
    case rs435_depth_laser_power:
    {
        static const auto rs435_depth_laser_power_factor = 30;
        ROS_DEBUG_STREAM("rs435_depth_laser_power: " << config.rs435_depth_laser_power * rs435_depth_laser_power_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.rs435_depth_laser_power * rs435_depth_laser_power_factor);
    }
        break;
    case rs435_depth_emitter_enabled:
        ROS_DEBUG_STREAM("rs435_depth_emitter_enabled: " << config.rs435_depth_emitter_enabled);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, config.rs435_depth_emitter_enabled);
        break;
    default:
        base_d400_paramsConfig base_config;
        base_config.base_depth_gain = config.rs435_depth_gain;
        base_config.base_depth_enable_auto_exposure = config.rs435_depth_enable_auto_exposure;
        base_config.base_depth_visual_preset = config.rs435_depth_visual_preset;
        base_config.base_depth_frames_queue_size = config.rs435_depth_frames_queue_size;
        base_config.base_depth_error_polling_enabled = config.rs435_depth_error_polling_enabled;
        base_config.base_depth_output_trigger_enabled = config.rs435_depth_output_trigger_enabled;
        base_config.base_depth_units = config.rs435_depth_units;
        base_config.base_JSON_file_path = config.rs435_JSON_file_path;
        base_config.base_enable_depth_to_disparity_filter = config.rs435_enable_depth_to_disparity_filter;
        base_config.base_enable_spatial_filter = config.rs435_enable_spatial_filter;
        base_config.base_enable_temporal_filter = config.rs435_enable_temporal_filter;
        base_config.base_enable_disparity_to_depth_filter = config.rs435_enable_disparity_to_depth_filter;
        base_config.base_spatial_filter_magnitude = config.rs435_spatial_filter_magnitude;
        base_config.base_spatial_filter_smooth_alpha = config.rs435_spatial_filter_smooth_alpha;
        base_config.base_spatial_filter_smooth_delta = config.rs435_spatial_filter_smooth_delta;
        base_config.base_spatial_filter_holes_fill = config.rs435_spatial_filter_holes_fill;
        base_config.base_temporal_filter_smooth_alpha = config.rs435_temporal_filter_smooth_alpha;
        base_config.base_temporal_filter_smooth_delta = config.rs435_temporal_filter_smooth_delta;
        base_config.base_temporal_filter_holes_fill = config.rs435_temporal_filter_holes_fill;
        RealSenseParamManager<RS400_PID> d400_param;
        d400_param.setParam(node_ptr, base_config, static_cast<typename RealSenseParamManager<RS400_PID>::Param>(param));
        break;
    }
}


template<>
void RealSenseParamManager<RS415_PID>::setParam(RealSenseNode* node_ptr, typename ModelTraits<RS415_PID>::Config &config, Param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case rs415_color_backlight_compensation:
        ROS_DEBUG_STREAM("base_JSON_file_path: " << config.rs415_color_backlight_compensation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.rs415_color_backlight_compensation);
        break;
    case rs415_color_brightness:
        ROS_DEBUG_STREAM("rs415_color_brightness: " << config.rs415_color_brightness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.rs415_color_brightness);
        break;
    case rs415_color_contrast:
        ROS_DEBUG_STREAM("rs415_color_contrast: " << config.rs415_color_contrast);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.rs415_color_contrast);
        break;
    case rs415_color_gain:
        ROS_DEBUG_STREAM("rs415_color_gain: " << config.rs415_color_gain);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.rs415_color_gain);
        break;
    case rs415_color_gamma:
        ROS_DEBUG_STREAM("rs415_color_gamma: " << config.rs415_color_gamma);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.rs415_color_gamma);
        break;
    case rs415_color_hue:
        ROS_DEBUG_STREAM("rs415_color_hue: " << config.rs415_color_hue);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.rs415_color_hue);
        break;
    case rs415_color_saturation:
        ROS_DEBUG_STREAM("rs415_color_saturation: " << config.rs415_color_saturation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.rs415_color_saturation);
        break;
    case rs415_color_sharpness:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.rs415_color_sharpness);
        break;
    case rs415_color_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.rs415_color_enable_auto_white_balance);
        break;
    case rs415_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_enable_auto_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.rs415_color_enable_auto_exposure);
        break;
    case rs415_color_exposure:
        ROS_DEBUG_STREAM("rs415_color_exposure: " << config.rs415_color_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs415_color_exposure);
        break;
    case rs415_color_white_balance:
    {
        static const auto rs415_color_white_balance_factor = 10;
        ROS_DEBUG_STREAM("rs415_color_white_balance: " << config.rs415_color_white_balance * rs415_color_white_balance_factor);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.rs415_color_white_balance * rs415_color_white_balance_factor);
    }
        break;
    case rs415_color_frames_queue_size:
        ROS_DEBUG_STREAM("rs415_color_frames_queue_size: " << config.rs415_color_frames_queue_size);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.rs415_color_frames_queue_size);
        break;
    case rs415_color_power_line_frequency:
        ROS_DEBUG_STREAM("rs415_color_power_line_frequency: " << config.rs415_color_power_line_frequency);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_POWER_LINE_FREQUENCY, config.rs415_color_power_line_frequency);
        break;
    case rs415_color_auto_exposure_priority:
        ROS_DEBUG_STREAM("rs415_color_power_line_frequency: " << config.rs415_color_auto_exposure_priority);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_PRIORITY, config.rs415_color_auto_exposure_priority);
        break;
    case rs415_depth_exposure:
    {
        static const auto rs415_depth_exposure_factor = 20;
        ROS_DEBUG_STREAM("rs415_depth_exposure: " << config.rs415_depth_exposure * rs415_depth_exposure_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs415_depth_exposure * rs415_depth_exposure_factor);
    }
        break;
    case rs415_depth_laser_power:
    {
        static const auto rs415_depth_laser_power_factor = 30;
        ROS_DEBUG_STREAM("rs415_depth_laser_power: " << config.rs415_depth_laser_power * rs415_depth_laser_power_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.rs415_depth_laser_power * rs415_depth_laser_power_factor);
    }
        break;
    case rs415_depth_emitter_enabled:
        ROS_DEBUG_STREAM("rs415_depth_emitter_enabled: " << config.rs415_depth_emitter_enabled);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, config.rs415_depth_emitter_enabled);
        break;
        case rs415_depth_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs415_depth_enable_auto_white_balance: " << config.rs415_depth_enable_auto_white_balance);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.rs415_depth_enable_auto_white_balance);
        break;
    default:
        base_d400_paramsConfig base_config;
        base_config.base_depth_gain = config.rs415_depth_gain;
        base_config.base_depth_enable_auto_exposure = config.rs415_depth_enable_auto_exposure;
        base_config.base_depth_visual_preset = config.rs415_depth_visual_preset;
        base_config.base_depth_frames_queue_size = config.rs415_depth_frames_queue_size;
        base_config.base_depth_error_polling_enabled = config.rs415_depth_error_polling_enabled;
        base_config.base_depth_output_trigger_enabled = config.rs415_depth_output_trigger_enabled;
        base_config.base_depth_units = config.rs415_depth_units;
        base_config.base_JSON_file_path = config.rs415_JSON_file_path;
        base_config.base_enable_depth_to_disparity_filter = config.rs415_enable_depth_to_disparity_filter;
        base_config.base_enable_spatial_filter = config.rs415_enable_spatial_filter;
        base_config.base_enable_temporal_filter = config.rs415_enable_temporal_filter;
        base_config.base_enable_disparity_to_depth_filter = config.rs415_enable_disparity_to_depth_filter;
        base_config.base_spatial_filter_magnitude = config.rs415_spatial_filter_magnitude;
        base_config.base_spatial_filter_smooth_alpha = config.rs415_spatial_filter_smooth_alpha;
        base_config.base_spatial_filter_smooth_delta = config.rs415_spatial_filter_smooth_delta;
        base_config.base_spatial_filter_holes_fill = config.rs415_spatial_filter_holes_fill;
        base_config.base_temporal_filter_smooth_alpha = config.rs415_temporal_filter_smooth_alpha;
        base_config.base_temporal_filter_smooth_delta = config.rs415_temporal_filter_smooth_delta;
        base_config.base_temporal_filter_holes_fill = config.rs415_temporal_filter_holes_fill;
        RealSenseParamManager<RS400_PID> d400_param;
        d400_param.setParam(node_ptr, base_config, static_cast<typename RealSenseParamManager<RS400_PID>::Param>(param));
        break;
    }
}

}
