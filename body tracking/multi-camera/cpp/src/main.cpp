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

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <sstream>

#ifdef _WIN32
#include <Windows.h>
#endif

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

    int nb_gpu = 0;
    if (!isJetson)
        cudaGetDeviceCount(&nb_gpu);

    const auto formatSdkError = [](sl::ERROR_CODE code) {
        std::ostringstream oss;
        oss << "ERROR_CODE(" << static_cast<int>(code) << ")";
        return oss.str();
    };

    const auto formatFusionError = [](sl::FUSION_ERROR_CODE code) {
        std::ostringstream oss;
        oss << code << " (code=" << static_cast<int>(code) << ")";
        return oss.str();
    };

    const auto logCountdown = [](const std::string& label, int seconds) {
        for (int remaining = seconds; remaining > 0; --remaining) {
            std::cout << label << " retry in " << remaining << "s" << std::endl;
            sl::sleep_ms(1000);
        }
    };

    const auto isLocalCameraEnumerated = [](const sl::FusionConfiguration& conf) {
        if (conf.communication_parameters.getType() != sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS)
            return true;

        if (conf.input_type.getType() == sl::InputType::INPUT_TYPE::SVO_FILE)
            return true;

        const auto devices = sl::Camera::getDeviceList();
        return std::any_of(devices.begin(), devices.end(), [&](const sl::DeviceProperties& device) {
            return device.serial_number == conf.serial_number;
        });
    };

    const auto probeLocalCameraAvailability = [&](const sl::FusionConfiguration& conf,
                                                  int gpu_id,
                                                  CameraOpenDiagnostic& diagnostic) {
#ifdef _WIN32
        __try {
            return ClientPublisher::probe(conf.input_type, gpu_id, app_config.publisher, diagnostic);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            diagnostic.stage = "probe/access_violation";
            diagnostic.error_code = static_cast<sl::ERROR_CODE>(GetExceptionCode());
            return false;
        }
#else
        return ClientPublisher::probe(conf.input_type, gpu_id, app_config.publisher, diagnostic);
#endif
    };

    const int initial_attempts = std::max(1, app_config.startup.initial_attempts);
    const int retry_interval_seconds = std::max(1, app_config.startup.retry_interval_seconds);
    const int recovery_initial_interval_seconds = std::max(1, app_config.startup.recovery_initial_interval_seconds);
    const int recovery_backoff_factor = std::max(1, app_config.startup.recovery_backoff_factor);
    const int recovery_max_interval_seconds = std::max(recovery_initial_interval_seconds, app_config.startup.recovery_max_interval_seconds);
    const int expected_camera_count = static_cast<int>(configurations.size());

    bool restart_requested = false;
    GLViewer viewer;
    bool viewer_initialized = false;
    do {
        restart_requested = false;

        Trigger trigger;
        std::vector<ClientPublisher> clients(configurations.size());
        std::vector<bool> opened_local(configurations.size(), false);
        std::vector<bool> local_probe_unstable(configurations.size(), false);
        std::vector<std::string> local_unavailable_reason(configurations.size());
        std::vector<int> startup_open_attempts(configurations.size(), 0);
        std::vector<bool> subscribed(configurations.size(), false);
        std::vector<bool> runtime_loss_reported(configurations.size(), false);
        std::vector<int> missing_configuration_indices;
        std::map<int, std::string> svo_files;

        const auto setLocalUnavailableReason = [&](size_t index, const std::string& reason) {
            local_unavailable_reason[index] = reason;
        };

        const auto logLocalUnavailableReason = [&](const char* prefix, size_t index) {
            if (!local_unavailable_reason[index].empty())
                std::cerr << prefix << local_unavailable_reason[index] << std::endl;
        };

        const auto tryOpenLocalCamera = [&](size_t index, int attempt) {
            const auto& conf = configurations[index];
            const int gpu_id = nb_gpu > 0 ? static_cast<int>(index % static_cast<size_t>(nb_gpu)) : 0;
            CameraOpenDiagnostic diagnostic;
            startup_open_attempts[index] = attempt;

            std::cout << "[Startup] Opening ZED " << conf.serial_number
                      << " attempt " << attempt << "/" << initial_attempts << std::endl;

            if (!isLocalCameraEnumerated(conf)) {
                setLocalUnavailableReason(index,
                                          "camera not enumerated by the ZED device list; likely disconnected, powered off, or not visible on USB.");
                std::cerr << "[Startup] ZED " << conf.serial_number
                          << " is not present in the current device list. Skipping open attempt." << std::endl;
                return false;
            }

            if (conf.input_type.getType() != sl::InputType::INPUT_TYPE::SVO_FILE) {
                CameraOpenDiagnostic probe_diagnostic;
                if (!probeLocalCameraAvailability(conf, gpu_id, probe_diagnostic)) {
                    if (probe_diagnostic.stage == "probe/access_violation") {
                        local_probe_unstable[index] = true;
                        setLocalUnavailableReason(index,
                                                  "camera is enumerated, but probing it triggers an SDK access violation; likely USB bandwidth, stream-start, or driver instability.");
                    } else {
                        setLocalUnavailableReason(index,
                                                  "camera is enumerated, but the SDK probe failed before streaming could start.");
                    }
                    std::cerr << "[Startup] ZED " << conf.serial_number << " failed availability probe during "
                              << probe_diagnostic.stage << ": " << formatSdkError(probe_diagnostic.error_code) << std::endl;
                    logLocalUnavailableReason("[Startup] Diagnosis: ", index);
                    return false;
                }
            }

            if (!clients[index].open(conf.input_type, &trigger, gpu_id, app_config.publisher, &diagnostic)) {
                if (diagnostic.stage == "open" && static_cast<int>(diagnostic.error_code) == 28) {
                    setLocalUnavailableReason(index,
                                              "camera is enumerated, but its stream could not start; check if another process is using it, or if USB/firewall/driver conditions block the stream.");
                } else {
                    setLocalUnavailableReason(index,
                                              "camera is enumerated, but the SDK open path failed before the publisher became ready.");
                }
                std::cerr << "[Startup] ZED " << conf.serial_number << " failed during " << diagnostic.stage
                          << ": " << formatSdkError(diagnostic.error_code) << std::endl;
                logLocalUnavailableReason("[Startup] Diagnosis: ", index);
                return false;
            }

            opened_local[index] = true;
            local_unavailable_reason[index].clear();
            if (conf.input_type.getType() == sl::InputType::INPUT_TYPE::SVO_FILE)
                svo_files.insert(std::make_pair(static_cast<int>(index), conf.input_type.getConfiguration()));
            std::cout << "[Startup] ZED " << conf.serial_number << " ready." << std::endl;
            return true;
        };

        for (size_t index = 0; index < configurations.size(); ++index) {
            const auto& conf = configurations[index];
            if (conf.communication_parameters.getType() != sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS)
                continue;

            tryOpenLocalCamera(index, 1);
        }

        const auto hasOpenedLocalCamera = [&opened_local]() {
            return std::any_of(opened_local.begin(), opened_local.end(), [](bool opened) {
                return opened;
            });
        };

        if (!hasOpenedLocalCamera()) {
            for (int attempt = 2; attempt <= initial_attempts && !hasOpenedLocalCamera(); ++attempt) {
                logCountdown("[Startup] No local camera ready", retry_interval_seconds);
                for (size_t index = 0; index < configurations.size(); ++index) {
                    const auto& conf = configurations[index];
                    if (conf.communication_parameters.getType() != sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS)
                        continue;
                    if (opened_local[index])
                        continue;

                    if (tryOpenLocalCamera(index, attempt) && hasOpenedLocalCamera())
                        break;
                }
            }
        }

        for (size_t index = 0; index < configurations.size(); ++index) {
            const auto& conf = configurations[index];
            if (conf.communication_parameters.getType() != sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS)
                continue;
            if (!opened_local[index]) {
                std::cerr << "[Startup] ZED " << conf.serial_number
                          << " unavailable after " << startup_open_attempts[index]
                          << " attempts. Continuing without it for now." << std::endl;
                logLocalUnavailableReason("[Startup] Final diagnosis: ", index);
            }
        }

        if (svo_files.size() > 1) {
            std::cout << "Starting SVO sync process..." << std::endl;
            const std::map<int, int> cam_idx_to_svo_frame_idx = syncDATA(svo_files);

            for (const auto& it : cam_idx_to_svo_frame_idx) {
                std::cout << "Setting camera " << it.first << " to frame " << it.second << std::endl;
                clients[it.first].setStartSVOPosition(it.second);
            }
        }

        for (auto& client : clients)
            client.start();

        sl::InitFusionParameters init_params;
        init_params.coordinate_units = coordinate_unit;
        init_params.coordinate_system = coordinate_system;

        sl::Resolution low_res(app_config.fusion.working_resolution_width, app_config.fusion.working_resolution_height);
        init_params.maximum_working_resolution = low_res;

        sl::Fusion fusion;
        fusion.init(init_params);

        std::vector<sl::CameraIdentifier> cameras;
        cameras.reserve(configurations.size());
        for (size_t index = 0; index < configurations.size(); ++index) {
            const auto& conf = configurations[index];
            const bool is_local = conf.communication_parameters.getType() == sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS;
            if (is_local && !opened_local[index]) {
                missing_configuration_indices.push_back(static_cast<int>(index));
                continue;
            }

            sl::CameraIdentifier uuid(conf.serial_number);
            for (int attempt = 1; attempt <= initial_attempts; ++attempt) {
                std::cout << "[Startup] Subscribing ZED " << conf.serial_number
                          << " attempt " << attempt << "/" << initial_attempts << std::endl;
                const auto state = fusion.subscribe(uuid, conf.communication_parameters, conf.pose, conf.override_gravity);
                if (state == sl::FUSION_ERROR_CODE::SUCCESS) {
                    subscribed[index] = true;
                    cameras.push_back(uuid);
                    std::cout << "[Startup] ZED " << conf.serial_number << " subscribed." << std::endl;
                    break;
                }

                std::cerr << "[Startup] Unable to subscribe to ZED " << conf.serial_number
                          << ": " << formatFusionError(state) << std::endl;
                if (attempt < initial_attempts)
                    logCountdown("[Startup] Subscribe ZED " + std::to_string(conf.serial_number), retry_interval_seconds);
            }

            if (!subscribed[index]) {
                missing_configuration_indices.push_back(static_cast<int>(index));
                std::cerr << "[Startup] ZED " << conf.serial_number
                          << " unavailable for fusion startup. Continuing without it for now." << std::endl;
            }
        }

        if (cameras.empty()) {
            std::cerr << "[Startup] No cameras available after startup retries. Exiting." << std::endl;
            trigger.running = false;
            trigger.notifyZED();
            for (auto& client : clients)
                client.stop();
            fusion.close();
            return EXIT_FAILURE;
        }

        if (static_cast<int>(cameras.size()) < expected_camera_count) {
            std::cout << "[Startup] Running in degraded mode with " << cameras.size() << "/" << expected_camera_count
                      << " configured cameras." << std::endl;
        } else {
            std::cout << "[Startup] All " << cameras.size() << " configured cameras are available." << std::endl;
        }

        sl::BodyTrackingFusionParameters body_fusion_init_params;
        body_fusion_init_params.enable_tracking = app_config.fusion.enable_tracking;
        body_fusion_init_params.enable_body_fitting = app_config.fusion.enable_body_fitting;
        fusion.enableBodyTracking(body_fusion_init_params);

        sl::BodyTrackingFusionRuntimeParameters body_tracking_runtime_parameters;
        body_tracking_runtime_parameters.skeleton_minimum_allowed_keypoints = app_config.fusion.minimum_keypoints;
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
                trigger.running = false;
                trigger.notifyZED();
                for (auto& client : clients)
                    client.stop();
                fusion.close();
                return EXIT_FAILURE;
            }
        }

        if (app_config.preview.enabled) {
            if (!viewer_initialized) {
                viewer.init(argc, argv);
                viewer_initialized = true;

                std::cout << "Viewer Shortcuts\n"
                          << "\t- 'q': quit the application\n"
                          << "\t- 'r': switch on/off for raw skeleton display\n"
                          << "\t- 'p': switch on/off for live point cloud display\n"
                          << "\t- 'c': switch on/off point cloud display with raw color\n"
                          << std::endl;
            }
        } else {
            std::cout << "Preview disabled. Running headless. Press Ctrl+C to exit.\n" << std::endl;
        }

        std::atomic<bool> recovery_thread_running(app_config.startup.restart_on_recovered_camera && !missing_configuration_indices.empty());
        std::atomic<bool> recovery_restart_requested(false);
        std::thread recovery_thread;
        if (recovery_thread_running) {
            const auto missing_copy = missing_configuration_indices;
            recovery_thread = std::thread([&, missing_copy]() {
                int delay_seconds = recovery_initial_interval_seconds;
                while (recovery_thread_running && !recovery_restart_requested) {
                    std::cout << "[Recovery] Missing cameras detected. Next probe in " << delay_seconds << "s." << std::endl;
                    for (int remaining = delay_seconds; remaining > 0; --remaining) {
                        if (!recovery_thread_running || recovery_restart_requested)
                            return;
                        std::cout << "[Recovery] Probe in " << remaining << "s" << std::endl;
                        sl::sleep_ms(1000);
                    }

                    for (const int index : missing_copy) {
                        if (!recovery_thread_running || recovery_restart_requested)
                            return;

                        const auto& conf = configurations[static_cast<size_t>(index)];
                        const bool is_local = conf.communication_parameters.getType() == sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS;

                        if (is_local && !opened_local[static_cast<size_t>(index)]) {
                            if (!isLocalCameraEnumerated(conf)) {
                                std::cerr << "[Recovery] ZED " << conf.serial_number
                                          << " is still absent from the device list." << std::endl;
                                logLocalUnavailableReason("[Recovery] Diagnosis: ", static_cast<size_t>(index));
                                continue;
                            }

                            CameraOpenDiagnostic probe_diagnostic;
                            const int gpu_id = nb_gpu > 0 ? index % nb_gpu : 0;
                            if (!probeLocalCameraAvailability(conf, gpu_id, probe_diagnostic)) {
                                if (probe_diagnostic.stage == "probe/access_violation")
                                    local_probe_unstable[static_cast<size_t>(index)] = true;
                                std::cerr << "[Recovery] ZED " << conf.serial_number
                                          << " is visible in the device list, but the availability probe still fails during "
                                          << probe_diagnostic.stage << ": " << formatSdkError(probe_diagnostic.error_code) << std::endl;
                                logLocalUnavailableReason("[Recovery] Diagnosis: ", static_cast<size_t>(index));
                                continue;
                            }

                            std::cout << "[Recovery] ZED " << conf.serial_number
                                      << " passed the availability probe again. Restarting application." << std::endl;
                            recovery_restart_requested = true;
                            return;
                        }

                        sl::Fusion probe_fusion;
                        probe_fusion.init(init_params);
                        const auto probe_state = probe_fusion.subscribe(sl::CameraIdentifier(conf.serial_number),
                                                                       conf.communication_parameters,
                                                                       conf.pose,
                                                                       conf.override_gravity);
                        probe_fusion.close();
                        if (probe_state == sl::FUSION_ERROR_CODE::SUCCESS) {
                            std::cout << "[Recovery] ZED " << conf.serial_number << " is subscribable again. Restarting application." << std::endl;
                            recovery_restart_requested = true;
                            return;
                        }

                        std::cerr << "[Recovery] ZED " << conf.serial_number
                                  << " still not ready for fusion subscribe: " << formatFusionError(probe_state) << std::endl;
                    }

                    delay_seconds = std::min(delay_seconds * recovery_backoff_factor, recovery_max_interval_seconds);
                }
            });
        }

        sl::Bodies fused_bodies;
        std::map<sl::CameraIdentifier, sl::Bodies> camera_raw_data;
        sl::FusionMetrics metrics;
        std::map<sl::CameraIdentifier, sl::Mat> views;
        std::map<sl::CameraIdentifier, sl::Mat> pointClouds;
        bool main_loop_exited_for_viewer = false;
        bool main_loop_exited_for_recovery = false;

        while ((!app_config.preview.enabled || viewer.isAvailable()) && !recovery_restart_requested) {
            trigger.notifyZED();

            for (size_t index = 0; index < configurations.size(); ++index) {
                if (!opened_local[index] || !subscribed[index])
                    continue;

                const auto health = clients[index].getRuntimeHealth();
                if (health.healthy)
                    continue;

                if (!runtime_loss_reported[index]) {
                    runtime_loss_reported[index] = true;
                    const auto& conf = configurations[index];
                    std::cerr << "[Watchdog] Lost ZED " << conf.serial_number
                              << " after " << health.consecutive_grab_failures
                              << " consecutive grab failures: " << formatSdkError(health.last_grab_error) << std::endl;
                    std::cerr << "[Watchdog] Restarting fusion session in degraded mode if needed." << std::endl;
                }

                recovery_restart_requested = true;
                break;
            }

            const auto process_state = fusion.process();
            if (process_state == sl::FUSION_ERROR_CODE::SUCCESS) {
                fusion.retrieveBodies(fused_bodies, body_tracking_runtime_parameters);
                osc_sender.send(fused_bodies);
                if (app_config.preview.enabled) {
                    for (auto& id : cameras) {
                        fusion.retrieveBodies(camera_raw_data[id], body_tracking_runtime_parameters, id);

                        const auto state_view = fusion.retrieveImage(views[id], id, low_res);
                        const auto state_pc = fusion.retrieveMeasure(pointClouds[id], id, sl::MEASURE::XYZBGRA, low_res);

                        if (state_view == sl::FUSION_ERROR_CODE::SUCCESS && state_pc == sl::FUSION_ERROR_CODE::SUCCESS)
                            viewer.updateCamera(id.sn, views[id], pointClouds[id]);

                        sl::Pose pose;
                        if (fusion.getPosition(pose, sl::REFERENCE_FRAME::WORLD, id, sl::POSITION_TYPE::RAW) == sl::POSITIONAL_TRACKING_STATE::OK)
                            viewer.setCameraPose(id.sn, pose.pose_data);
                    }
                }

                fusion.getProcessMetrics(metrics);
            } else if (app_config.verbose_logging) {
                std::cerr << "[Fusion] process() returned " << formatFusionError(process_state) << std::endl;
            }

            if (app_config.preview.enabled)
                viewer.updateBodies(fused_bodies, camera_raw_data, metrics);
        }

        main_loop_exited_for_viewer = app_config.preview.enabled && !viewer.isAvailable();
        main_loop_exited_for_recovery = recovery_restart_requested;

        if (main_loop_exited_for_viewer)
            std::cerr << "[Loop] Main loop exited because the preview viewer is no longer available." << std::endl;
        if (main_loop_exited_for_recovery)
            std::cerr << "[Loop] Main loop exited because a recovery restart was requested." << std::endl;

        if (recovery_restart_requested)
            restart_requested = true;

        recovery_thread_running = false;
        trigger.running = false;
        trigger.notifyZED();

        if (recovery_thread.joinable())
            recovery_thread.join();

        for (auto& client : clients)
            client.stop();

        osc_sender.shutdown();
        fusion.close();

        if (restart_requested)
            std::cout << "[Recovery] Restarting fusion session with recovered cameras..." << std::endl;

    } while (restart_requested);

    if (app_config.preview.enabled && viewer_initialized)
        viewer.exit();

    return EXIT_SUCCESS;
}
