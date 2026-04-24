#include "ClientPublisher.hpp"

ClientPublisher::ClientPublisher() { }

ClientPublisher::~ClientPublisher() {
    zed.close();
}

bool ClientPublisher::open(sl::InputType input, Trigger* ref, int sdk_gpu_id, const PublisherConfig& config) {

    p_trigger = ref;
    config_ = config;

    sl::InitParameters init_parameters;
    init_parameters.depth_mode = config_.depth_mode;
    init_parameters.input = input;
    init_parameters.coordinate_units = config_.coordinate_units;
    init_parameters.depth_stabilization = config_.depth_stabilization;
    init_parameters.sdk_gpu_id = sdk_gpu_id;
    auto state = zed.open(init_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    serial = zed.getCameraInformation().serial_number;
    p_trigger->states[serial] = false;

    // in most cases in body tracking setup, the cameras are static
    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = config_.positional_tracking_static;

    state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    // define the body tracking parameters, as the fusion can does the tracking and fitting you don't need to enable them here, unless you
    // need it for your app
    sl::BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.detection_model = config_.detection_model;
    body_tracking_parameters.body_format = config_.body_format;
    body_tracking_parameters.enable_body_fitting = config_.enable_body_fitting;
    body_tracking_parameters.enable_tracking = config_.enable_tracking;
    body_tracking_parameters.enable_segmentation = config_.enable_segmentation;
    body_tracking_parameters.allow_reduced_precision_inference = config_.allow_reduced_precision_inference;
    state = zed.enableBodyTracking(body_tracking_parameters);
    if (state > sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    return true;
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
    sl::Bodies bodies;
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
            if (zed.grab(rt) <= sl::ERROR_CODE::SUCCESS) { }
        }
        p_trigger->states[serial] = true;
    }
}

void ClientPublisher::setStartSVOPosition(unsigned pos) {
    zed.setSVOPosition(pos);
}
