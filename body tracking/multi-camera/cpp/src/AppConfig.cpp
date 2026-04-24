#include "AppConfig.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace {

std::string trim(const std::string& input) {
    const auto begin = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    if (begin >= end)
        return {};
    return std::string(begin, end);
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

bool parseBool(const std::string& value, bool& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on") {
        out = true;
        return true;
    }
    if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off") {
        out = false;
        return true;
    }
    return false;
}

bool parseInt(const std::string& value, int& out) {
    try {
        out = std::stoi(trim(value));
        return true;
    } catch (...) {
        return false;
    }
}

bool parseUInt16(const std::string& value, uint16_t& out) {
    int parsed = 0;
    if (!parseInt(value, parsed) || parsed < 0 || parsed > 65535)
        return false;
    out = static_cast<uint16_t>(parsed);
    return true;
}

bool parseFloat(const std::string& value, float& out) {
    try {
        out = std::stof(trim(value));
        return true;
    } catch (...) {
        return false;
    }
}

bool parseDepthMode(const std::string& value, sl::DEPTH_MODE& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "performance") out = sl::DEPTH_MODE::PERFORMANCE;
    else if (lowered == "quality") out = sl::DEPTH_MODE::QUALITY;
    else if (lowered == "ultra") out = sl::DEPTH_MODE::ULTRA;
    else if (lowered == "neural") out = sl::DEPTH_MODE::NEURAL;
    else if (lowered == "neural_light") out = sl::DEPTH_MODE::NEURAL_LIGHT;
    else return false;
    return true;
}

bool parseUnit(const std::string& value, sl::UNIT& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "meter" || lowered == "meters" || lowered == "m") out = sl::UNIT::METER;
    else if (lowered == "centimeter" || lowered == "centimeters" || lowered == "cm") out = sl::UNIT::CENTIMETER;
    else if (lowered == "millimeter" || lowered == "millimeters" || lowered == "mm") out = sl::UNIT::MILLIMETER;
    else return false;
    return true;
}

bool parseCoordinateSystem(const std::string& value, sl::COORDINATE_SYSTEM& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "right_handed_y_up") out = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    else if (lowered == "right_handed_z_up") out = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
    else if (lowered == "left_handed_y_up") out = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
    else if (lowered == "left_handed_z_up") out = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    else if (lowered == "right_handed_z_up_x_fwd") out = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    else return false;
    return true;
}

bool parseDetectionModel(const std::string& value, sl::BODY_TRACKING_MODEL& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "human_body_fast" || lowered == "fast") out = sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
    else if (lowered == "human_body_medium" || lowered == "medium") out = sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
    else if (lowered == "human_body_accurate" || lowered == "accurate") out = sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
    else return false;
    return true;
}

bool parseBodyFormat(const std::string& value, sl::BODY_FORMAT& out) {
    const auto lowered = toLower(trim(value));
    if (lowered == "18" || lowered == "body_18") out = sl::BODY_FORMAT::BODY_18;
    else if (lowered == "34" || lowered == "body_34") out = sl::BODY_FORMAT::BODY_34;
    else if (lowered == "38" || lowered == "body_38") out = sl::BODY_FORMAT::BODY_38;
    else return false;
    return true;
}

std::vector<std::filesystem::path> searchRootsFromExecutable(const std::filesystem::path& executable_path) {
    std::vector<std::filesystem::path> search_roots;
    const auto executable_dir = executable_path.parent_path();
    if (!executable_dir.empty()) {
        search_roots.push_back(executable_dir);
        if (!executable_dir.parent_path().empty())
            search_roots.push_back(executable_dir.parent_path());
        if (!executable_dir.parent_path().parent_path().empty())
            search_roots.push_back(executable_dir.parent_path().parent_path());
    }
    return search_roots;
}

bool applyConfigValue(const std::string& section, const std::string& key, const std::string& value, AppConfig& config, std::string& error) {
    const auto full_key = section.empty() ? key : section + "." + key;

    if (full_key == "app.verbose_logging")
        return parseBool(value, config.verbose_logging);
    if (full_key == "input.calibration_file") {
        config.calibration_file = trim(value);
        return true;
    }
    if (full_key == "publisher.depth_mode")
        return parseDepthMode(value, config.publisher.depth_mode);
    if (full_key == "publisher.coordinate_units")
        return parseUnit(value, config.publisher.coordinate_units);
    if (full_key == "publisher.depth_stabilization")
        return parseInt(value, config.publisher.depth_stabilization);
    if (full_key == "publisher.positional_tracking_static")
        return parseBool(value, config.publisher.positional_tracking_static);
    if (full_key == "publisher.detection_model")
        return parseDetectionModel(value, config.publisher.detection_model);
    if (full_key == "publisher.body_format")
        return parseBodyFormat(value, config.publisher.body_format);
    if (full_key == "publisher.enable_body_fitting")
        return parseBool(value, config.publisher.enable_body_fitting);
    if (full_key == "publisher.enable_tracking")
        return parseBool(value, config.publisher.enable_tracking);
    if (full_key == "publisher.enable_segmentation")
        return parseBool(value, config.publisher.enable_segmentation);
    if (full_key == "publisher.allow_reduced_precision_inference")
        return parseBool(value, config.publisher.allow_reduced_precision_inference);
    if (full_key == "publisher.runtime_detection_confidence")
        return parseInt(value, config.publisher.runtime_detection_confidence);
    if (full_key == "publisher.runtime_skeleton_smoothing")
        return parseFloat(value, config.publisher.runtime_skeleton_smoothing);
    if (full_key == "publisher.grab_confidence_threshold")
        return parseInt(value, config.publisher.grab_confidence_threshold);
    if (full_key == "fusion.coordinate_system")
        return parseCoordinateSystem(value, config.fusion.coordinate_system);
    if (full_key == "fusion.coordinate_units")
        return parseUnit(value, config.fusion.coordinate_units);
    if (full_key == "fusion.working_resolution_width")
        return parseInt(value, config.fusion.working_resolution_width);
    if (full_key == "fusion.working_resolution_height")
        return parseInt(value, config.fusion.working_resolution_height);
    if (full_key == "fusion.enable_tracking")
        return parseBool(value, config.fusion.enable_tracking);
    if (full_key == "fusion.enable_body_fitting")
        return parseBool(value, config.fusion.enable_body_fitting);
    if (full_key == "fusion.minimum_keypoints")
        return parseInt(value, config.fusion.minimum_keypoints);
    if (full_key == "fusion.minimum_cameras")
        return parseInt(value, config.fusion.minimum_cameras);
    if (full_key == "fusion.skeleton_smoothing")
        return parseFloat(value, config.fusion.skeleton_smoothing);
    if (full_key == "osc.enabled")
        return parseBool(value, config.osc.enabled);
    if (full_key == "osc.ip") {
        config.osc.ip = trim(value);
        return !config.osc.ip.empty();
    }
    if (full_key == "osc.port")
        return parseUInt16(value, config.osc.port);
    if (full_key == "osc.use_bundle")
        return parseBool(value, config.osc.use_bundle);
    if (full_key == "osc.send_interval_ms")
        return parseInt(value, config.osc.send_interval_ms);
    if (full_key == "osc.only_tracked_bodies")
        return parseBool(value, config.osc.only_tracked_bodies);

    return true;
}

} // namespace

AppConfig makeDefaultAppConfig(bool isJetson) {
    AppConfig config;
    config.publisher.depth_mode = isJetson ? sl::DEPTH_MODE::NEURAL_LIGHT : sl::DEPTH_MODE::NEURAL;
    config.fusion.enable_body_fitting = !isJetson;
    return config;
}

bool looksLikeConfigFile(const std::string& path) {
    const auto extension = toLower(std::filesystem::path(path).extension().string());
    return extension == ".ini" || extension == ".cfg" || extension == ".conf";
}

std::string resolveInputPath(const std::string& input_path, const std::filesystem::path& base_dir) {
    if (input_path.empty())
        return {};
    const std::filesystem::path path(input_path);
    if (path.is_absolute())
        return path.lexically_normal().string();
    return (base_dir / path).lexically_normal().string();
}

FileSearchResult findDefaultAppConfigFile(const std::filesystem::path& executable_path) {
    FileSearchResult result;
    result.searched_roots = searchRootsFromExecutable(executable_path);

    const std::vector<std::string> candidate_names = {"zed_bodyfusion.ini", "bodyfusion.ini"};
    for (const auto& root : result.searched_roots) {
        if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root))
            continue;
        for (const auto& candidate : candidate_names) {
            const auto full_path = root / candidate;
            if (std::filesystem::exists(full_path) && std::filesystem::is_regular_file(full_path)) {
                result.selected_file = full_path.string();
                return result;
            }
        }
    }

    return result;
}

FileSearchResult findLatestCalibrationFile(const std::filesystem::path& executable_path) {
    FileSearchResult result;
    result.searched_roots = searchRootsFromExecutable(executable_path);

    bool found = false;
    std::filesystem::path newest_file;
    std::filesystem::file_time_type newest_time;

    for (const auto& root : result.searched_roots) {
        if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root))
            continue;

        for (const auto& entry : std::filesystem::directory_iterator(root)) {
            if (!entry.is_regular_file())
                continue;

            const auto filename = entry.path().filename().string();
            if (filename.rfind("calib_", 0) != 0 || entry.path().extension() != ".json")
                continue;

            const auto write_time = entry.last_write_time();
            if (!found || write_time > newest_time) {
                newest_file = entry.path();
                newest_time = write_time;
                found = true;
            }
        }
    }

    if (found)
        result.selected_file = newest_file.string();

    return result;
}

bool loadAppConfig(const std::string& config_path, AppConfig& config, std::string& error) {
    std::ifstream input(config_path);
    if (!input.is_open()) {
        error = "Unable to open config file: " + config_path;
        return false;
    }

    std::string section;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        auto trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == ';' || trimmed[0] == '#')
            continue;

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            section = toLower(trim(trimmed.substr(1, trimmed.size() - 2)));
            continue;
        }

        const auto separator = trimmed.find('=');
        if (separator == std::string::npos)
            continue;

        const auto key = toLower(trim(trimmed.substr(0, separator)));
        const auto value = trim(trimmed.substr(separator + 1));
        if (!applyConfigValue(section, key, value, config, error)) {
            std::ostringstream oss;
            oss << "Invalid value for " << section << "." << key << " at line " << line_number;
            error = oss.str();
            return false;
        }
    }

    return true;
}