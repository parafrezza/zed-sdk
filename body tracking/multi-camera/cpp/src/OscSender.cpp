#include "OscSender.hpp"

#ifdef _WIN32
#include <ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <array>
#include <cstring>
#include <iostream>
#include <unordered_set>

namespace {

#ifdef _WIN32
bool ensureSocketLayer(std::string& error) {
    static bool initialized = false;
    if (initialized)
        return true;
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        error = "WSAStartup failed";
        return false;
    }
    initialized = true;
    return true;
}

void closeSocket(osc_socket_t socket) {
    closesocket(socket);
}

constexpr osc_socket_t kInvalidSocket = INVALID_SOCKET;
#else
bool ensureSocketLayer(std::string&) {
    return true;
}

void closeSocket(osc_socket_t socket) {
    close(socket);
}

constexpr osc_socket_t kInvalidSocket = -1;
#endif

struct SocketAddressStorage {
    sockaddr_in value {};
};

const std::array<const char*, 18> kBody18JointNames = {
    "nose", "neck", "right_shoulder", "right_elbow", "right_wrist", "left_shoulder", "left_elbow", "left_wrist",
    "right_hip", "right_knee", "right_ankle", "left_hip", "left_knee", "left_ankle", "right_eye", "left_eye", "right_ear", "left_ear"
};

const std::array<const char*, 34> kBody34JointNames = {
    "pelvis", "naval_spine", "chest_spine", "neck", "left_clavicle", "left_shoulder", "left_elbow", "left_wrist", "left_hand",
    "left_handtip", "left_thumb", "right_clavicle", "right_shoulder", "right_elbow", "right_wrist", "right_hand", "right_handtip",
    "right_thumb", "left_hip", "left_knee", "left_ankle", "left_foot", "right_hip", "right_knee", "right_ankle", "right_foot",
    "head", "nose", "left_eye", "left_ear", "right_eye", "right_ear", "left_heel", "right_heel"
};

const std::array<const char*, 38> kBody38JointNames = {
    "pelvis", "spine_1", "spine_2", "spine_3", "neck", "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_clavicle", "right_clavicle", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "left_big_toe", "right_big_toe",
    "left_small_toe", "right_small_toe", "left_heel", "right_heel", "left_hand_thumb_4", "right_hand_thumb_4", "left_hand_index_1",
    "right_hand_index_1", "left_hand_middle_4", "right_hand_middle_4", "left_hand_pinky_1", "right_hand_pinky_1"
};

} // namespace

OscSender::OscSender()
    : enabled_(false)
    , verbose_logging_(false)
    , use_bundle_(true)
    , only_tracked_bodies_(true)
    , send_interval_ms_(0)
    , target_port_(0)
    , last_send_valid_(false)
    , warned_format_mismatch_(false)
    , socket_(kInvalidSocket)
    , socket_ready_(false)
    , addr_storage_(nullptr) { }

OscSender::~OscSender() {
    shutdown();
}

bool OscSender::initialize(const OscConfig& config, sl::BODY_FORMAT body_format, bool verbose_logging, std::string& error) {
    shutdown();

    if (!config.enabled)
        return true;

    if (!ensureSocketLayer(error))
        return false;

    joint_names_ = jointNamesForBodyFormat(body_format);
    standard_tag_ = standardTagForBodyFormat(body_format);
    if (joint_names_.empty()) {
        error = "Unsupported body format for OSC export";
        return false;
    }

    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ == kInvalidSocket) {
        error = "Unable to create UDP socket";
        return false;
    }

    auto* storage = new SocketAddressStorage();
    std::memset(&storage->value, 0, sizeof(storage->value));
    storage->value.sin_family = AF_INET;
    storage->value.sin_port = htons(config.port);
    if (inet_pton(AF_INET, config.ip.c_str(), &storage->value.sin_addr) <= 0) {
        error = "Invalid OSC target IP: " + config.ip;
        delete storage;
        closeSocket(socket_);
        socket_ = kInvalidSocket;
        return false;
    }

    addr_storage_ = reinterpret_cast<sockaddr_in*>(storage);
    socket_ready_ = true;
    enabled_ = true;
    verbose_logging_ = verbose_logging;
    use_bundle_ = config.use_bundle;
    only_tracked_bodies_ = config.only_tracked_bodies;
    send_interval_ms_ = std::max(0, config.send_interval_ms);
    target_ip_ = config.ip;
    target_port_ = config.port;

    if (verbose_logging_) {
        std::cout << "[OSC] Target " << target_ip_ << ":" << target_port_ << " | standard=" << standard_tag_
                  << " | interval_ms=" << send_interval_ms_ << std::endl;
    }

    return true;
}

bool OscSender::isEnabled() const {
    return enabled_;
}

const std::string& OscSender::standardTag() const {
    return standard_tag_;
}

bool OscSender::shouldSendNow() {
    if (send_interval_ms_ <= 0)
        return true;
    const auto now = std::chrono::steady_clock::now();
    if (!last_send_valid_) {
        last_send_time_ = now;
        last_send_valid_ = true;
        return true;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time_).count();
    if (elapsed < send_interval_ms_)
        return false;
    last_send_time_ = now;
    return true;
}

bool OscSender::shouldExportBody(const sl::BodyData& body) const {
    if (!only_tracked_bodies_)
        return body.tracking_state == sl::OBJECT_TRACKING_STATE::OK || body.tracking_state == sl::OBJECT_TRACKING_STATE::OFF;
    return body.tracking_state == sl::OBJECT_TRACKING_STATE::OK;
}

bool OscSender::send(const sl::Bodies& bodies) {
    if (!enabled_ || !socket_ready_)
        return true;
    if (!shouldSendNow())
        return true;

    std::unordered_set<int> current_ids;
    for (const auto& body : bodies.body_list) {
        if (!shouldExportBody(body))
            continue;
        if (body.keypoint.size() != joint_names_.size()) {
            if (verbose_logging_ && !warned_format_mismatch_) {
                std::cerr << "[OSC] Skipping body with unexpected keypoint count: " << body.keypoint.size()
                          << " for standard " << standard_tag_ << std::endl;
                warned_format_mismatch_ = true;
            }
            continue;
        }
        current_ids.insert(body.id);
        sendBody(body);
    }

    for (const auto& previous_id : previous_body_ids_) {
        if (current_ids.find(previous_id) == current_ids.end())
            sendAliveMessage(previous_id, 0);
    }
    previous_body_ids_ = std::move(current_ids);
    return true;
}

void OscSender::shutdown() {
    if (enabled_ && socket_ready_) {
        for (const auto& body_id : previous_body_ids_)
            sendAliveMessage(body_id, 0);
    }
    previous_body_ids_.clear();
    if (socket_ != kInvalidSocket) {
        closeSocket(socket_);
        socket_ = kInvalidSocket;
    }
    socket_ready_ = false;
    enabled_ = false;
    if (addr_storage_) {
        delete reinterpret_cast<SocketAddressStorage*>(addr_storage_);
        addr_storage_ = nullptr;
    }
}

bool OscSender::sendAliveMessage(int body_id, int alive_value) {
    std::array<uint8_t, 128> buffer {};
    size_t offset = 0;
    offset = writePaddedString(buffer.data(), offset, "/skeleton/" + std::to_string(body_id) + "/" + standard_tag_ + "/alive");
    offset = writePaddedString(buffer.data(), offset, ",i");
    writeUint32BE(buffer.data() + offset, static_cast<uint32_t>(alive_value));
    offset += 4;
    return sendPacket(buffer.data(), offset);
}

bool OscSender::sendBody(const sl::BodyData& body) {
    return use_bundle_ ? sendBodyBundle(body) : sendBodyPerJoint(body);
}

bool OscSender::sendBodyBundle(const sl::BodyData& body) {
    const size_t max_message_size = 160;
    const size_t buffer_size = 16 + (joint_names_.size() + 1) * (4 + max_message_size);
    std::vector<uint8_t> buffer(buffer_size, 0);

    size_t offset = 0;
    offset = writePaddedString(buffer.data(), offset, "#bundle");
    offset += 8;

    std::array<uint8_t, max_message_size> message {};
    size_t message_offset = 0;
    message_offset = writePaddedString(message.data(), message_offset, "/skeleton/" + std::to_string(body.id) + "/" + standard_tag_ + "/alive");
    message_offset = writePaddedString(message.data(), message_offset, ",i");
    writeUint32BE(message.data() + message_offset, 1);
    message_offset += 4;
    writeUint32BE(buffer.data() + offset, static_cast<uint32_t>(message_offset));
    offset += 4;
    std::memcpy(buffer.data() + offset, message.data(), message_offset);
    offset += message_offset;

    for (size_t index = 0; index < joint_names_.size(); ++index) {
        message.fill(0);
        message_offset = 0;
        message_offset = writePaddedString(message.data(), message_offset, "/skeleton/" + std::to_string(body.id) + "/" + standard_tag_ + "/" + joint_names_[index] + "/");
        message_offset = writePaddedString(message.data(), message_offset, ",fff");
        writeFloatBE(message.data() + message_offset, body.keypoint[index].x);
        message_offset += 4;
        writeFloatBE(message.data() + message_offset, body.keypoint[index].y);
        message_offset += 4;
        writeFloatBE(message.data() + message_offset, body.keypoint[index].z);
        message_offset += 4;
        writeUint32BE(buffer.data() + offset, static_cast<uint32_t>(message_offset));
        offset += 4;
        std::memcpy(buffer.data() + offset, message.data(), message_offset);
        offset += message_offset;
    }

    return sendPacket(buffer.data(), offset);
}

bool OscSender::sendBodyPerJoint(const sl::BodyData& body) {
    if (!sendAliveMessage(body.id, 1))
        return false;

    std::array<uint8_t, 160> buffer {};
    for (size_t index = 0; index < joint_names_.size(); ++index) {
        size_t offset = 0;
        offset = writePaddedString(buffer.data(), offset, "/skeleton/" + std::to_string(body.id) + "/" + standard_tag_ + "/" + joint_names_[index] + "/");
        offset = writePaddedString(buffer.data(), offset, ",fff");
        writeFloatBE(buffer.data() + offset, body.keypoint[index].x);
        offset += 4;
        writeFloatBE(buffer.data() + offset, body.keypoint[index].y);
        offset += 4;
        writeFloatBE(buffer.data() + offset, body.keypoint[index].z);
        offset += 4;
        if (!sendPacket(buffer.data(), offset))
            return false;
    }
    return true;
}

bool OscSender::sendPacket(const uint8_t* data, size_t size) {
    if (!data || size == 0 || !socket_ready_ || !addr_storage_)
        return false;

    const auto* addr = reinterpret_cast<const sockaddr*>(addr_storage_);
    const int sent = sendto(socket_, reinterpret_cast<const char*>(data), static_cast<int>(size), 0, addr, sizeof(sockaddr_in));
    if (sent < 0 && verbose_logging_)
        std::cerr << "[OSC] sendto failed" << std::endl;
    return sent >= 0;
}

std::string OscSender::standardTagForBodyFormat(sl::BODY_FORMAT body_format) {
    switch (body_format) {
        case sl::BODY_FORMAT::BODY_18:
            return "zed18";
        case sl::BODY_FORMAT::BODY_34:
            return "zed34";
        case sl::BODY_FORMAT::BODY_38:
            return "zed38";
        default:
            return {};
    }
}

std::vector<std::string> OscSender::jointNamesForBodyFormat(sl::BODY_FORMAT body_format) {
    std::vector<std::string> names;
    switch (body_format) {
        case sl::BODY_FORMAT::BODY_18:
            names.assign(kBody18JointNames.begin(), kBody18JointNames.end());
            break;
        case sl::BODY_FORMAT::BODY_34:
            names.assign(kBody34JointNames.begin(), kBody34JointNames.end());
            break;
        case sl::BODY_FORMAT::BODY_38:
            names.assign(kBody38JointNames.begin(), kBody38JointNames.end());
            break;
        default:
            break;
    }
    return names;
}

size_t OscSender::writePaddedString(uint8_t* dst, size_t offset, const std::string& value) {
    const size_t length = value.size() + 1;
    std::memcpy(dst + offset, value.c_str(), length);
    const size_t padded_length = (length + 3u) & ~3u;
    std::memset(dst + offset + length, 0, padded_length - length);
    return offset + padded_length;
}

void OscSender::writeUint32BE(uint8_t* dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    dst[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[3] = static_cast<uint8_t>(value & 0xFF);
}

void OscSender::writeFloatBE(uint8_t* dst, float value) {
    union {
        float as_float;
        uint32_t as_uint32;
    } encoded;
    encoded.as_float = value;
    writeUint32BE(dst, encoded.as_uint32);
}