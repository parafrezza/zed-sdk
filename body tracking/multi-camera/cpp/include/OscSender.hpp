#ifndef ZED_BODY_FUSION_OSC_SENDER_HPP
#define ZED_BODY_FUSION_OSC_SENDER_HPP

#include "AppConfig.hpp"

#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

#include <sl/Camera.hpp>

#ifdef _WIN32
#include <winsock2.h>
typedef SOCKET osc_socket_t;
#else
typedef int osc_socket_t;
#endif

class OscSender {
public:
    OscSender();
    ~OscSender();

    bool initialize(const OscConfig& config, sl::BODY_FORMAT body_format, bool verbose_logging, std::string& error);
    bool send(const sl::Bodies& bodies);
    void shutdown();

    bool isEnabled() const;
    const std::string& standardTag() const;

private:
    bool shouldSendNow();
    bool shouldExportBody(const sl::BodyData& body) const;
    bool sendAliveMessage(int body_id, int alive_value);
    bool sendBody(const sl::BodyData& body);
    bool sendBodyBundle(const sl::BodyData& body);
    bool sendBodyPerJoint(const sl::BodyData& body);
    bool sendPacket(const uint8_t* data, size_t size);

    static std::string standardTagForBodyFormat(sl::BODY_FORMAT body_format);
    static std::vector<std::string> jointNamesForBodyFormat(sl::BODY_FORMAT body_format);
    static size_t writePaddedString(uint8_t* dst, size_t offset, const std::string& value);
    static void writeUint32BE(uint8_t* dst, uint32_t value);
    static void writeFloatBE(uint8_t* dst, float value);

    bool enabled_;
    bool verbose_logging_;
    bool use_bundle_;
    bool only_tracked_bodies_;
    int send_interval_ms_;
    std::string target_ip_;
    uint16_t target_port_;
    std::string standard_tag_;
    std::vector<std::string> joint_names_;
    std::unordered_set<int> previous_body_ids_;
    bool last_send_valid_;
    std::chrono::steady_clock::time_point last_send_time_;
    bool warned_format_mismatch_;

    osc_socket_t socket_;
    bool socket_ready_;
    struct sockaddr_in* addr_storage_;
};

#endif