#include "ClientPublisher.hpp"

namespace {

void setDiagnostic(CameraOpenDiagnostic* diagnostic, const char* stage, sl::ERROR_CODE error_code) {
    if (!diagnostic)
        return;
    diagnostic->stage = stage;
    diagnostic->error_code = error_code;
}

bool configureCamera(sl::Camera& zed,
                     const sl::InputType& input,
                     int sdk_gpu_id,
                     const PublisherConfig& config,
                     CameraOpenDiagnostic* diagnostic,
                     int* serial_out) {
    setDiagnostic(diagnostic, "open", sl::ERROR_CODE::SUCCESS);

    sl::InitParameters init_parameters;
    init_parameters.depth_mode = config.depth_mode;
    init_parameters.input = input;
    init_parameters.coordinate_units = config.coordinate_units;
    init_parameters.depth_stabilization = config.depth_stabilization;
    init_parameters.sdk_gpu_id = sdk_gpu_id;

    auto state = zed.open(init_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        setDiagnostic(diagnostic, "open", state);
        return false;
    }

    if (serial_out)
        *serial_out = zed.getCameraInformation().serial_number;

    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = config.positional_tracking_static;

    state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        setDiagnostic(diagnostic, "enablePositionalTracking", state);
        if (zed.isOpened())
            zed.close();
        return false;
    }

    sl::BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.detection_model = config.detection_model;
    body_tracking_parameters.body_format = config.body_format;
    body_tracking_parameters.enable_body_fitting = config.enable_body_fitting;
    body_tracking_parameters.enable_tracking = config.enable_tracking;
    body_tracking_parameters.enable_segmentation = config.enable_segmentation;
    body_tracking_parameters.allow_reduced_precision_inference = config.allow_reduced_precision_inference;

    state = zed.enableBodyTracking(body_tracking_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        setDiagnostic(diagnostic, "enableBodyTracking", state);
        if (zed.isOpened())
            zed.close();
        return false;
    }

    setDiagnostic(diagnostic, "ready", sl::ERROR_CODE::SUCCESS);
    return true;
}

} // namespace

ClientPublisher::ClientPublisher() { }

ClientPublisher::~ClientPublisher() {
    zed.close();
}

bool ClientPublisher::open(sl::InputType input, Trigger* ref, int sdk_gpu_id, const PublisherConfig& config, CameraOpenDiagnostic* diagnostic) {

    p_trigger = ref;
    config_ = config;

    if (!configureCamera(zed, input, sdk_gpu_id, config_, diagnostic, &serial))
        return false;

    {
        std::lock_guard<std::mutex> lock(health_mtx);
        runtime_health_.healthy = true;
        runtime_health_.consecutive_grab_failures = 0;
        runtime_health_.last_grab_error = sl::ERROR_CODE::SUCCESS;
    }

    p_trigger->states[serial] = false;
    return true;
}

bool ClientPublisher::probe(const sl::InputType& input, int sdk_gpu_id, const PublisherConfig& config, CameraOpenDiagnostic& diagnostic) {
    sl::Camera probe_camera;
    const bool success = configureCamera(probe_camera, input, sdk_gpu_id, config, &diagnostic, nullptr);
    probe_camera.close();
    return success;
}

void ClientPublisher::start() {
    if (zed.isOpened()) {
        // the camera should stream its data so the fusion can subscibe to it to gather the detected body and others metadata needed for the
        // process.
        zed.startPublishing();
        // the thread can start to process the camera grab in background
        runner = std::thread(&ClientPublisher::work, this);
    }
}

void ClientPublisher::stop() {
    if (runner.joinable())
        runner.join();
    zed.close();
}

void ClientPublisher::work() {
    sl::BodyTrackingRuntimeParameters body_runtime_parameters;
    body_runtime_parameters.detection_confidence_threshold = config_.runtime_detection_confidence;
    body_runtime_parameters.skeleton_smoothing = config_.runtime_skeleton_smoothing;
    zed.setBodyTrackingRuntimeParameters(body_runtime_parameters);

    sl::RuntimeParameters rt;
    rt.confidence_threshold = config_.grab_confidence_threshold;

    // In this sample we use a dummy thread to process the ZED data.
    // you can replace it by your own application and use the ZED like you use to, retrieve its images, depth, sensors data and so on.
    // As long as you call the grab method, since the camera is subscribed to fusion it will run the detection and
    // the camera will be able to seamlessly transmit the data to the fusion module.
    while (p_trigger->running) {
        std::unique_lock<std::mutex> lk(mtx);
        p_trigger->cv.wait(lk);
        if (p_trigger->running) {
            CameraRuntimeHealth current_health;
            {
                std::lock_guard<std::mutex> health_lock(health_mtx);
                current_health = runtime_health_;
            }

            if (current_health.healthy) {
                const auto grab_state = zed.grab(rt);
                std::lock_guard<std::mutex> health_lock(health_mtx);
                if (grab_state <= sl::ERROR_CODE::SUCCESS) {
                    runtime_health_.healthy = true;
                    runtime_health_.consecutive_grab_failures = 0;
                    runtime_health_.last_grab_error = sl::ERROR_CODE::SUCCESS;
                } else {
                    runtime_health_.last_grab_error = grab_state;
                    runtime_health_.consecutive_grab_failures += 1;
                    if (runtime_health_.consecutive_grab_failures >= config_.watchdog_grab_failure_threshold)
                        runtime_health_.healthy = false;
                }
            }
        }
        p_trigger->states[serial] = true;
    }
}

void ClientPublisher::setStartSVOPosition(unsigned pos) {
    zed.setSVOPosition(pos);
}

bool ClientPublisher::isOpened() const {
    return zed.isOpened();
}

CameraRuntimeHealth ClientPublisher::getRuntimeHealth() const {
    std::lock_guard<std::mutex> lock(health_mtx);
    return runtime_health_;
}
