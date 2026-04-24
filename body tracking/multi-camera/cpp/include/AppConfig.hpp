#ifndef ZED_BODY_FUSION_APP_CONFIG_HPP
#define ZED_BODY_FUSION_APP_CONFIG_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

struct PublisherConfig {
    sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::NEURAL;
    sl::UNIT coordinate_units = sl::UNIT::METER;
    int depth_stabilization = 30;
    bool positional_tracking_static = true;
    sl::BODY_TRACKING_MODEL detection_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
    sl::BODY_FORMAT body_format = sl::BODY_FORMAT::BODY_18;
    bool enable_body_fitting = false;
    bool enable_tracking = false;
    bool enable_segmentation = false;
    bool allow_reduced_precision_inference = true;
    int runtime_detection_confidence = 40;
    float runtime_skeleton_smoothing = 0.4f;
    int grab_confidence_threshold = 50;
};

struct FusionConfig {
    sl::COORDINATE_SYSTEM coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    sl::UNIT coordinate_units = sl::UNIT::METER;
    int working_resolution_width = 512;
    int working_resolution_height = 360;
    bool enable_tracking = true;
    bool enable_body_fitting = true;
    int minimum_keypoints = 7;
    int minimum_cameras = 1;
    float skeleton_smoothing = 0.4f;
};

struct OscConfig {
    bool enabled = true;
    std::string ip = "127.0.0.1";
    uint16_t port = 9000;
    bool use_bundle = true;
    int send_interval_ms = 40;
    bool only_tracked_bodies = true;
};

struct AppConfig {
    std::string calibration_file;
    bool verbose_logging = true;
    PublisherConfig publisher;
    FusionConfig fusion;
    OscConfig osc;
};

struct FileSearchResult {
    std::string selected_file;
    std::vector<std::filesystem::path> searched_roots;
};

AppConfig makeDefaultAppConfig(bool isJetson);
bool looksLikeConfigFile(const std::string& path);
std::string resolveInputPath(const std::string& input_path, const std::filesystem::path& base_dir);
FileSearchResult findDefaultAppConfigFile(const std::filesystem::path& executable_path);
FileSearchResult findLatestCalibrationFile(const std::filesystem::path& executable_path);
bool loadAppConfig(const std::string& config_path, AppConfig& config, std::string& error);

#endif