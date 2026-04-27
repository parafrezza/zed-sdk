///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2025, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// ZED include
#include "AppConfig.hpp"
#include "ClientPublisher.hpp"
#include "GLViewer.hpp"
#include "OscSender.hpp"
#include "utils.hpp"

#include <filesystem>

int main(int argc, char** argv) {

#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif

    const auto executable_path = std::filesystem::absolute(argv[0]);
    AppConfig app_config = makeDefaultAppConfig(isJetson);

    std::string app_config_file;
    std::filesystem::path app_config_base_dir = executable_path.parent_path();
    std::string calibration_override;

    if (argc >= 2) {
        const std::string input_arg = argv[1];
        if (looksLikeConfigFile(input_arg)) {
            app_config_file = std::filesystem::absolute(input_arg).string();
            std::string error;
            if (!loadAppConfig(app_config_file, app_config, error)) {
                std::cerr << error << std::endl;
                return EXIT_FAILURE;
            }
            app_config_base_dir = std::filesystem::path(app_config_file).parent_path();
            std::cout << "Using app config from command line: " << app_config_file << std::endl;
        } else {
            calibration_override = std::filesystem::absolute(input_arg).string();
            std::cout << "Using calibration file from command line: " << calibration_override << std::endl;
        }
    } else {
        const auto config_search = findDefaultAppConfigFile(executable_path);
        if (!config_search.selected_file.empty()) {
            app_config_file = config_search.selected_file;
            std::string error;
            if (!loadAppConfig(app_config_file, app_config, error)) {
                std::cerr << error << std::endl;
                return EXIT_FAILURE;
            }
            app_config_base_dir = std::filesystem::path(app_config_file).parent_path();
            std::cout << "Using app config discovered automatically: " << app_config_file << std::endl;
            std::cout << "Config search folders:" << std::endl;
            for (const auto& root : config_search.searched_roots)
                std::cout << "  - " << root.string() << std::endl;
        } else {
            std::cout << "No app config found. Using built-in defaults." << std::endl;
        }
    }

    std::string calibration_file;
    if (!calibration_override.empty()) {
        calibration_file = calibration_override;
    } else if (!app_config.calibration_file.empty()) {
        calibration_file = resolveInputPath(app_config.calibration_file, app_config_base_dir);
        std::cout << "Using calibration file from app config: " << calibration_file << std::endl;
    } else {
        const auto search_result = findLatestCalibrationFile(executable_path);
        calibration_file = search_result.selected_file;
        if (calibration_file.empty()) {
            std::cout << "Need a Configuration file in input" << std::endl;
            std::cout << "No calib_*.json file found in these folders:" << std::endl;
            for (const auto& root : search_result.searched_roots)
                std::cout << "  - " << root.string() << std::endl;
            return 1;
        }
        std::cout << "Using latest calibration file discovered automatically: " << calibration_file << std::endl;
        std::cout << "Calibration search folders:" << std::endl;
        for (const auto& root : search_result.searched_roots)
            std::cout << "  - " << root.string() << std::endl;
    }

    // Defines the Coordinate system and unit used in this sample
    const auto coordinate_system = app_config.fusion.coordinate_system;
    const auto coordinate_unit = app_config.fusion.coordinate_units;

    // Read json file containing the configuration of your multicamera setup.
    auto configurations = sl::readFusionConfigurationFile(calibration_file.c_str(), coordinate_system, coordinate_unit);

    if (configurations.empty()) {
        std::cout << "Empty configuration File." << std::endl;
        return EXIT_FAILURE;
    }

    Trigger trigger;

    // Check if the ZED camera should run within the same process or if they are running on the edge.
    std::vector<ClientPublisher> clients(configurations.size());
    int id_ = 0;
    int gpu_id = 0;

    int nb_gpu = 0;
    if (!isJetson)
        cudaGetDeviceCount(&nb_gpu);

    std::map<int, std::string> svo_files;
    for (auto conf : configurations) {
        // if the ZED camera should run locally, then start a thread to handle it
        if (conf.communication_parameters.getType() == sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS) {
            std::cout << "Try to open ZED " << conf.serial_number << ".." << std::flush;
            gpu_id = nb_gpu > 0 ? id_ % nb_gpu : 0;
            auto state = clients[id_].open(conf.input_type, &trigger, gpu_id, app_config.publisher);
            if (!state) {
                std::cerr << "Could not open ZED: " << conf.input_type.getConfiguration() << ". Skipping..." << std::endl;
                continue;
            }

            if (conf.input_type.getType() == sl::InputType::INPUT_TYPE::SVO_FILE)
                svo_files.insert(std::make_pair(id_, conf.input_type.getConfiguration()));

            std::cout << ". ready !" << std::endl;

            id_++;
        }
    }

    // Synchronize SVO files in SVO mode
    bool enable_svo_sync = (svo_files.size() > 1);
    if (enable_svo_sync) {
        std::cout << "Starting SVO sync process..." << std::endl;
        std::map<int, int> cam_idx_to_svo_frame_idx = syncDATA(svo_files);

        for (auto& it : cam_idx_to_svo_frame_idx) {
            std::cout << "Setting camera " << it.first << " to frame " << it.second << std::endl;
            clients[it.first].setStartSVOPosition(it.second);
        }
    }

    // start camera threads
    for (auto& it : clients)
        it.start();

    // Now that the ZED camera are running, we need to initialize the fusion module
    sl::InitFusionParameters init_params;
    init_params.coordinate_units = coordinate_unit;
    init_params.coordinate_system = coordinate_system;

    sl::Resolution low_res(app_config.fusion.working_resolution_width, app_config.fusion.working_resolution_height);
    init_params.maximum_working_resolution = low_res;
    // create and initialize it
    sl::Fusion fusion;
    fusion.init(init_params);

    // subscribe to every cameras of the setup to internally gather their data
    std::vector<sl::CameraIdentifier> cameras;
    for (auto& it : configurations) {
        sl::CameraIdentifier uuid(it.serial_number);
        // to subscribe to a camera you must give its serial number, the way to communicate with it (shared memory or local network), and
        // its world pose in the setup.
        auto state = fusion.subscribe(uuid, it.communication_parameters, it.pose, it.override_gravity);
        if (state != sl::FUSION_ERROR_CODE::SUCCESS)
            std::cout << "Unable to subscribe to " << std::to_string(uuid.sn) << " . " << state << std::endl;
        else
            cameras.push_back(uuid);
    }

    // check that at least one camera is connected
    if (cameras.empty()) {
        std::cout << "no connections " << std::endl;
        return EXIT_FAILURE;
    }

    // as this sample shows how to fuse body detection from the multi camera setup
    // we enable the Body Tracking module with its options
    sl::BodyTrackingFusionParameters body_fusion_init_params;
    body_fusion_init_params.enable_tracking = app_config.fusion.enable_tracking;
    body_fusion_init_params.enable_body_fitting = app_config.fusion.enable_body_fitting;
    fusion.enableBodyTracking(body_fusion_init_params);

    // define fusion behavior
    sl::BodyTrackingFusionRuntimeParameters body_tracking_runtime_parameters;
    // be sure that the detection skeleton is complete enough
    body_tracking_runtime_parameters.skeleton_minimum_allowed_keypoints = app_config.fusion.minimum_keypoints;
    // allow a body seen by a single camera while still letting fusion refine it when multiple views overlap
    body_tracking_runtime_parameters.skeleton_minimum_allowed_camera = app_config.fusion.minimum_cameras;
    body_tracking_runtime_parameters.skeleton_smoothing = app_config.fusion.skeleton_smoothing;

    OscSender osc_sender;
    if (app_config.osc.enabled) {
        if (app_config.osc.log_messages && app_config.osc.log_file.empty())
            app_config.osc.log_file = (executable_path.parent_path() / "zed_bodyfusion_osc.log").string();
        else if (!app_config.osc.log_file.empty())
            app_config.osc.log_file = resolveInputPath(app_config.osc.log_file, app_config_base_dir);

        std::string error;
        if (!osc_sender.initialize(app_config.osc, app_config.fusion.body_format, app_config.verbose_logging, error)) {
            std::cerr << error << std::endl;
            return EXIT_FAILURE;
        }
    }

    // creation of a 3D viewer
    GLViewer viewer;
    if (app_config.preview.enabled) {
        viewer.init(argc, argv);

        std::cout << "Viewer Shortcuts\n"
                  << "\t- 'q': quit the application\n"
                  << "\t- 'r': switch on/off for raw skeleton display\n"
                  << "\t- 'p': switch on/off for live point cloud display\n"
                  << "\t- 'c': switch on/off point cloud display with raw color\n"
                  << std::endl;
    } else {
        std::cout << "Preview disabled. Running headless. Press Ctrl+C to exit.\n" << std::endl;
    }

    // fusion outputs
    sl::Bodies fused_bodies;
    std::map<sl::CameraIdentifier, sl::Bodies> camera_raw_data;
    sl::FusionMetrics metrics;
    std::map<sl::CameraIdentifier, sl::Mat> views;
    std::map<sl::CameraIdentifier, sl::Mat> pointClouds;
    sl::CameraIdentifier fused_camera(0);

    // run the fusion as long as the viewer is available.
    while (!app_config.preview.enabled || viewer.isAvailable()) {
        trigger.notifyZED();

        // run the fusion process (which gather data from all camera, sync them and process them)
        if (fusion.process() == sl::FUSION_ERROR_CODE::SUCCESS) {

            // Retrieve fused body
            fusion.retrieveBodies(fused_bodies, body_tracking_runtime_parameters);
            osc_sender.send(fused_bodies);
            // for debug, you can retrieve the data sent by each camera
            if (app_config.preview.enabled) {
                for (auto& id : cameras) {
                    fusion.retrieveBodies(camera_raw_data[id], body_tracking_runtime_parameters, id);

                    auto state_view = fusion.retrieveImage(views[id], id, low_res);
                    auto state_pc = fusion.retrieveMeasure(pointClouds[id], id, sl::MEASURE::XYZBGRA, low_res);

                    if (state_view == sl::FUSION_ERROR_CODE::SUCCESS && state_pc == sl::FUSION_ERROR_CODE::SUCCESS)
                        viewer.updateCamera(id.sn, views[id], pointClouds[id]);

                    sl::Pose pose;
                    if (fusion.getPosition(pose, sl::REFERENCE_FRAME::WORLD, id, sl::POSITION_TYPE::RAW) == sl::POSITIONAL_TRACKING_STATE::OK)
                        viewer.setCameraPose(id.sn, pose.pose_data);
                }
            }

            // get metrics about the fusion process for monitoring purposes
            fusion.getProcessMetrics(metrics);
        }
        // update the 3D view
        if (app_config.preview.enabled)
            viewer.updateBodies(fused_bodies, camera_raw_data, metrics);
    }

    trigger.running = false;
    trigger.notifyZED();

    for (auto& it : clients)
        it.stop();

    osc_sender.shutdown();
    fusion.close();

    return EXIT_SUCCESS;
}
