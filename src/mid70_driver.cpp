/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#define _USE_MATH_DEFINES
#include "mid70_driver.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>

namespace mid70_driver {

    /**
     * Mid-70 使用 Livox 私有 UDP 协议，两种包格式：
     *
     * 1. 0xAA 帧（发现广播 / 命令应答 / 心跳应答），帧头 9 字节 + 帧尾 CRC32，小端：
     *    sof(1)=0xAA | version(1)=1 | length(2, 整包长度) | packet_type(1) | seq_num(2) | preamble_crc(2)
     *    cmd_set(1) | cmd_id(1) | data[...] | crc32(4)
     *    preamble_crc = CRC16/MCRF4XX(seed 0x4C49) 校验前 7 字节，crc32 = CRC32(seed 0x564F580A) 覆盖 [0, length-4)
     *    packet_type: 0=CMD(主机请求) 1=ACK(应答) 2=MSG(设备主动上报)
     *
     * 2. 点云包（裸包，无 0xAA 帧头）：LivoxEthPacket = 18 字节头 + N 个点
     */
    static constexpr uint8_t kSof = 0xAA;
    static constexpr uint8_t kSdkVersion = 0x01;
    static constexpr std::size_t kFramePrefixLength = 9;
    static constexpr std::size_t kFrameCrcLength = 4;
    static constexpr std::size_t kFrameMinLength = kFramePrefixLength + 2 + kFrameCrcLength;
    static constexpr uint16_t kCrcSeed16 = 0x4C49;
    static constexpr uint32_t kCrcSeed32 = 0x564F580A;

    enum PacketType : uint8_t {
        kPacketTypeCmd = 0,// 主机请求
        kPacketTypeAck = 1,// 设备应答
        kPacketTypeMsg = 2 // 设备主动上报
    };

    enum CommandSet : uint8_t {
        kCommandSetGeneral = 0,// 通用命令集
        kCommandSetLidar = 1   // 雷达命令集
    };

    enum GeneralCommandId : uint8_t {
        kCommandBroadcast = 0x00,       // 设备广播（发现）
        kCommandHandshake = 0x01,       // 握手
        kCommandDeviceInfo = 0x02,      // 设备信息
        kCommandHeartbeat = 0x03,       // 心跳，必须 >= 1Hz
        kCommandControlSample = 0x04,   // 采样开关
        kCommandCoordinateSystem = 0x05,// 坐标格式
        kCommandDisconnect = 0x06,      // 断开连接
        kCommandPushAbnormal = 0x07     // 设备异常状态上报
    };

    enum LidarCommandId : uint8_t {
        kCommandSetReturnMode = 0x06,// 设置回波模式
        kCommandGetReturnMode = 0x07 // 查询回波模式
    };

    static constexpr std::size_t kLivoxEthHeaderLength = 18;
    // 每种 data_type 的单点字节数与回波数（与 Livox 协议一致，6 为 IMU）
    static constexpr std::size_t kPointSize[] = {13, 9, 14, 10, 28, 16, 24, 42, 22};
    static constexpr int kEchoCount[] = {1, 1, 1, 1, 2, 2, 0, 3, 3};

    static constexpr uint8_t kTimestampTypeGps = 3;// GPS/UTC 时间戳，其余类型为纳秒
    static constexpr double kPointPeriod = 1e-5;   // Mid-70 点率 100k 点/s
    static constexpr double kMaxValidDistance = 1000.0;
    static constexpr double kCommandTimeout = 1.5;// 单次命令等待应答时间 [s]
    static constexpr int kCommandRetryTimes = 3;
    static constexpr double kPointcloudWaitTime = 0.1;// 没有点云时的最长等待 [s]，用于发心跳与检查断流
    static constexpr int kMaxConnectFailureCount = 3;

    struct Frame {
        uint8_t packet_type = kPacketTypeCmd;
        uint8_t cmd_set = 0;
        uint8_t cmd_id = 0;
        std::vector<uint8_t> data;
    };

#pragma pack(1)

    struct EthHeader {
        uint8_t version;
        uint8_t slot;
        uint8_t id;
        uint8_t reserved;
        uint32_t error_code;
        uint8_t timestamp_type;// 0=无同步(ns) 1=PTP(ns) 3=GPS/UTC 4=PPS(ns)
        uint8_t data_type;     // 见 kPointSize
        uint8_t timestamp[8];  // 含义由 timestamp_type 决定
    };

#pragma pack()

    static_assert(sizeof(EthHeader) == kLivoxEthHeaderLength, "LivoxEthPacket 头必须是 18 字节");

    struct DecodedPoint {
        float x = 0;
        float y = 0;
        float z = 0;
        uint8_t reflectivity = 0;
        uint8_t tag = 0;
    };

    struct UtcTime {
        int year = 1970;
        int month = 1;
        int day = 1;
        int hour = 0;
        uint32_t microsecond = 0;
    };

    static uint16_t read_le16(const uint8_t *data) {
        return static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
    }

    static uint32_t read_le32(const uint8_t *data) {
        return static_cast<uint32_t>(data[0]) |
               (static_cast<uint32_t>(data[1]) << 8) |
               (static_cast<uint32_t>(data[2]) << 16) |
               (static_cast<uint32_t>(data[3]) << 24);
    }

    static void write_le16(uint8_t *data, uint16_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFF);
        data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    static void write_le32(uint8_t *data, uint32_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFF);
        data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    }

    static int32_t read_i32(const uint8_t *data) {
        return static_cast<int32_t>(read_le32(data));
    }

    // CRC16/MCRF4XX（反射多项式 0x8408，seed 0x4C49），校验 0xAA 帧头前 7 字节
    static uint16_t crc16_mcrf4xx(const uint8_t *data, std::size_t length) {
        static const std::array<uint16_t, 256> table = [] {
            std::array<uint16_t, 256> result{};
            for (std::size_t i = 0; i < result.size(); ++i) {
                uint16_t value = static_cast<uint16_t>(i);
                for (int bit = 0; bit < 8; ++bit) {
                    value = (value & 1) ? static_cast<uint16_t>((value >> 1) ^ 0x8408) : static_cast<uint16_t>(value >> 1);
                }
                result[i] = value;
            }
            return result;
        }();
        uint16_t crc = kCrcSeed16;
        for (std::size_t i = 0; i < length; ++i) {
            crc = static_cast<uint16_t>((crc >> 8) ^ table[(crc & 0xFF) ^ data[i]]);
        }
        return crc;
    }

    // CRC32（反射多项式 0xEDB88320，init = seed ^ 0xFFFFFFFF，末尾取反）
    static uint32_t crc32(const uint8_t *data, std::size_t length) {
        static const std::array<uint32_t, 256> table = [] {
            std::array<uint32_t, 256> result{};
            for (std::size_t i = 0; i < result.size(); ++i) {
                uint32_t value = static_cast<uint32_t>(i);
                for (int bit = 0; bit < 8; ++bit) {
                    value = (value & 1) ? ((value >> 1) ^ 0xEDB88320) : (value >> 1);
                }
                result[i] = value;
            }
            return result;
        }();
        uint32_t crc = kCrcSeed32 ^ 0xFFFFFFFF;
        for (std::size_t i = 0; i < length; ++i) {
            crc = (crc >> 8) ^ table[(crc ^ data[i]) & 0xFF];
        }
        return crc ^ 0xFFFFFFFF;
    }

    static std::vector<uint8_t> pack_frame(uint8_t packet_type, uint16_t sequence_number, uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data) {
        std::vector<uint8_t> buffer(kFramePrefixLength + 2 + data.size() + kFrameCrcLength);
        buffer[0] = kSof;
        buffer[1] = kSdkVersion;
        write_le16(&buffer[2], static_cast<uint16_t>(buffer.size()));
        buffer[4] = packet_type;
        write_le16(&buffer[5], sequence_number);
        write_le16(&buffer[7], crc16_mcrf4xx(buffer.data(), 7));
        buffer[9] = cmd_set;
        buffer[10] = cmd_id;
        if (!data.empty()) {
            std::memcpy(&buffer[11], data.data(), data.size());
        }
        write_le32(&buffer[buffer.size() - kFrameCrcLength], crc32(buffer.data(), buffer.size() - kFrameCrcLength));
        return buffer;
    }

    static bool unpack_frame(const uint8_t *buffer, std::size_t length, Frame &frame, std::string &error) {
        if (length < kFrameMinLength || buffer[0] != kSof) [[unlikely]] {
            error = "不是 0xAA 帧";
            return false;
        }
        const std::size_t total_length = read_le16(&buffer[2]);
        if (total_length < kFrameMinLength || total_length > length) [[unlikely]] {
            error = "帧长度非法";
            return false;
        }
        if (read_le16(&buffer[7]) != crc16_mcrf4xx(buffer, 7)) [[unlikely]] {
            error = "帧头 CRC16 校验失败";
            return false;
        }
        if (read_le32(&buffer[total_length - kFrameCrcLength]) != crc32(buffer, total_length - kFrameCrcLength)) [[unlikely]] {
            error = "帧尾 CRC32 校验失败";
            return false;
        }
        frame.packet_type = buffer[4];
        frame.cmd_set = buffer[9];
        frame.cmd_id = buffer[10];
        frame.data.assign(&buffer[11], &buffer[total_length - kFrameCrcLength]);
        return true;
    }

    // 球坐标（depth 单位 mm，theta 天顶角 / phi 方位角单位 0.01 度）转笛卡尔
    static void spherical_to_cartesian(uint32_t depth, uint16_t theta, uint16_t phi, DecodedPoint &point) {
        const double radius = static_cast<double>(depth) / 1000.0;
        const double zenith = static_cast<double>(theta) / 100.0 / 180 * M_PI;
        const double azimuth = static_cast<double>(phi) / 100.0 / 180 * M_PI;
        point.x = static_cast<float>(radius * sin(zenith) * cos(azimuth));
        point.y = static_cast<float>(radius * sin(zenith) * sin(azimuth));
        point.z = static_cast<float>(radius * cos(zenith));
    }

    // 解出一个点，只取首次回波（与 Mid-360 驱动一致：后续回波由回波模式参数在设备侧控制）
    static bool decode_first_echo(const uint8_t *data, uint8_t data_type, DecodedPoint &point) {
        switch (data_type) {
            case 0:// 笛卡尔坐标
                point.x = static_cast<float>(read_i32(data)) * 0.001f;
                point.y = static_cast<float>(read_i32(&data[4])) * 0.001f;
                point.z = static_cast<float>(read_i32(&data[8])) * 0.001f;
                point.reflectivity = data[12];
                return true;
            case 1:// 球坐标
                spherical_to_cartesian(read_le32(data), read_le16(&data[4]), read_le16(&data[6]), point);
                point.reflectivity = data[8];
                return true;
            case 2:// 扩展笛卡尔坐标
            case 4:// 双回波扩展笛卡尔坐标
            case 7:// 三回波扩展笛卡尔坐标
                point.x = static_cast<float>(read_i32(data)) * 0.001f;
                point.y = static_cast<float>(read_i32(&data[4])) * 0.001f;
                point.z = static_cast<float>(read_i32(&data[8])) * 0.001f;
                point.reflectivity = data[12];
                point.tag = data[13];
                return true;
            case 3:// 扩展球坐标
            case 5:// 双回波扩展球坐标
            case 8:// 三回波扩展球坐标
                spherical_to_cartesian(read_le32(&data[4]), read_le16(data), read_le16(&data[2]), point);
                point.reflectivity = data[8];
                point.tag = data[9];
                return true;
            default:
                return false;
        }
    }

    static bool is_leap_year(int year) {
        return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    }

    // GPS/UTC 时间戳：year + month + day + hour + 该小时内的微秒数
    static bool parse_utc_time(const uint8_t *timestamp, UtcTime &utc_time) {
        utc_time.year = 2000 + static_cast<int>(timestamp[0]);
        utc_time.month = static_cast<int>(timestamp[1]);
        utc_time.day = static_cast<int>(timestamp[2]);
        utc_time.hour = static_cast<int>(timestamp[3]);
        utc_time.microsecond = read_le32(&timestamp[4]);
        return utc_time.month >= 1 && utc_time.month <= 12 && utc_time.day >= 1 && utc_time.day <= 31 && utc_time.hour <= 23;
    }

    static bool utc_to_unix_seconds(const UtcTime &utc_time, double &seconds) {
        static constexpr int month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int64_t days = 0;
        for (int year = 1970; year < utc_time.year; ++year) {
            days += is_leap_year(year) ? 366 : 365;
        }
        for (int month = 1; month < utc_time.month; ++month) {
            days += month_days[month - 1] + ((month == 2 && is_leap_year(utc_time.year)) ? 1 : 0);
        }
        days += utc_time.day - 1;
        seconds = static_cast<double>(days) * 86400.0 + static_cast<double>(utc_time.hour) * 3600.0 + static_cast<double>(utc_time.microsecond) * 1e-6;
        return true;
    }

    // 包时间戳统一换算成秒：GPS/UTC 转 Unix 秒，其余类型本来就是纳秒
    static bool get_packet_timestamp(const EthHeader &header, double &timestamp) {
        if (header.timestamp_type == kTimestampTypeGps) {
            UtcTime utc_time;
            if (!parse_utc_time(header.timestamp, utc_time)) [[unlikely]] {
                return false;
            }
            return utc_to_unix_seconds(utc_time, timestamp);
        }
        uint64_t timestamp_ns = 0;
        for (int i = 7; i >= 0; --i) {
            timestamp_ns = (timestamp_ns << 8) | header.timestamp[i];
        }
        timestamp = static_cast<double>(timestamp_ns) * 1e-9;
        return true;
    }

    static double get_current_time() {
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    static std::chrono::steady_clock::time_point get_time_after(double seconds) {
        return std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
    }

    static std::string describe_command_result(const std::string &error, const std::vector<uint8_t> &response) {
        if (!error.empty()) {
            return error;
        }
        if (response.empty()) {
            return "应答数据为空";
        }
        return "ret_code=" + std::to_string(response[0]);
    }

    Mid70Driver::Mid70Driver(asio::io_context &io_context,
                             const Options &options,
                             const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                             const std::function<void(LogLevel level, const std::string &message)> &on_log)
        : io_context(io_context),
          options(options),
          discovery_socket(io_context),
          cmd_socket(io_context),
          data_socket(io_context),
          timeout_timer(io_context),
          sleep_timer(io_context),
          receive_buffer(65536),
          on_receive_pointcloud(on_receive_pointcloud),
          on_log(on_log) {
        co_spawn(io_context, run(), asio::detached);
    }

    Mid70Driver::~Mid70Driver() {
        stop();
    }

    void Mid70Driver::stop() {
        is_running.store(false, std::memory_order_relaxed);
        timeout_timer.cancel();
        sleep_timer.cancel();
        asio::error_code error_code;
        discovery_socket.close(error_code);
        // 收点云先停，run() 会走到 close_session() 主动断开；cmd socket 只打断可能在等的应答，
        // 保持可用才能把断开报文发出去（设备因此立刻回到广播态，不用等心跳超时）
        data_socket.close(error_code);
        cmd_socket.cancel(error_code);
    }

    void Mid70Driver::reset_frame() {
        is_frame_base_reset.store(true, std::memory_order_relaxed);
    }

    void Mid70Driver::log(LogLevel level, const std::string &message) {
        if (on_log) {
            on_log(level, message);
        }
    }

    asio::awaitable<void> Mid70Driver::run() {
        while (is_running.load(std::memory_order_relaxed)) {
            // 发现 -> 握手 -> 收点云，点云断流或出错后回到握手（保留已发现的雷达 IP）
            if (device_ip.is_unspecified() && !co_await discover_device()) {
                co_await sleep(1.0);
                continue;
            }
            if (!co_await connect_device()) {
                if (++connect_failure_count >= kMaxConnectFailureCount) {
                    connect_failure_count = 0;
                    if (options.lidar_ip.is_unspecified()) {
                        device_ip = asio::ip::address();// 反复连不上就重新等广播，雷达 IP 可能变了
                    }
                }
                co_await sleep(1.0);
                continue;
            }
            connect_failure_count = 0;
            co_await stream_pointcloud();
            close_session();
        }
    }

    asio::awaitable<bool> Mid70Driver::discover_device() {
        if (!options.lidar_ip.is_unspecified()) {
            device_ip = options.lidar_ip;
            device_port = options.device_port;
            log(LogLevel::kInfo, "使用配置的雷达 IP " + device_ip.to_string() + ":" + std::to_string(device_port));
            co_return true;
        }

        // 雷达每秒向 255.255.255.255:discovery_port 广播，数据段 = 序列号(16) + 设备类型(1) + 设备端口(2)
        asio::error_code error_code;
        discovery_socket.open(asio::ip::udp::v4(), error_code);
        if (!error_code) {
            discovery_socket.set_option(asio::socket_base::reuse_address(true), error_code);
        }
        if (!error_code) {
            discovery_socket.set_option(asio::socket_base::broadcast(true), error_code);
        }
        if (!error_code) {
            discovery_socket.bind(asio::ip::udp::endpoint(asio::ip::address_v4::any(), options.discovery_port), error_code);
        }
        if (error_code) [[unlikely]] {
            log(LogLevel::kError, "监听雷达广播端口 " + std::to_string(options.discovery_port) + " 失败：" + error_code.message());
            discovery_socket.close();
            co_return false;
        }

        log(LogLevel::kInfo, "等待雷达广播（最多 " + std::to_string(options.discovery_timeout) + " s）");
        arm_timeout(get_time_after(options.discovery_timeout), discovery_socket);
        while (is_running.load(std::memory_order_relaxed)) {
            uint8_t buffer[2048];
            asio::ip::udp::endpoint sender_endpoint;
            const std::size_t length = co_await discovery_socket.async_receive_from(
                    asio::buffer(buffer, sizeof(buffer)),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            if (error_code) {
                break;// 超时（定时器 cancel 掉等待）或驱动停止
            }
            Frame frame;
            std::string unpack_error;
            if (!unpack_frame(buffer, length, frame, unpack_error)) {
                continue;
            }
            if (frame.packet_type != kPacketTypeMsg || frame.cmd_set != kCommandSetGeneral || frame.cmd_id != kCommandBroadcast || frame.data.size() < 19) {
                continue;
            }
            device_ip = sender_endpoint.address();
            device_port = read_le16(&frame.data[17]);
            device_serial_number.assign(reinterpret_cast<const char *>(frame.data.data()), strnlen(reinterpret_cast<const char *>(frame.data.data()), 16));
            log(LogLevel::kInfo, "发现雷达 " + device_ip.to_string() + ":" + std::to_string(device_port) +
                                         " 序列号 " + device_serial_number +
                                         " 设备类型 " + std::to_string(frame.data[16]));
            timeout_timer.cancel();
            discovery_socket.close();
            co_return true;
        }
        timeout_timer.cancel();
        discovery_socket.close();
        if (is_running.load(std::memory_order_relaxed)) {
            log(LogLevel::kWarn, "未收到雷达广播（检查网线/网段/供电）");
        }
        co_return false;
    }

    asio::awaitable<bool> Mid70Driver::connect_device() {
        host_ip = options.host_ip;
        if (host_ip.is_unspecified()) {
            host_ip = detect_local_ip();
        }
        if (host_ip.is_unspecified()) {
            log(LogLevel::kError, "无法确定本机 IP（请设置 host_ip 参数）");
            co_return false;
        }
        if (!host_ip.is_v4()) {
            log(LogLevel::kError, "host_ip 必须是 IPv4 地址：" + host_ip.to_string());
            co_return false;
        }
        if (!open_socket(cmd_socket, options.cmd_port) || !open_socket(data_socket, options.data_port)) {
            close_session();
            co_return false;
        }

        // 握手：告知雷达本机 IP 与三个接收端口（Mid-70 无 IMU，imu_port 只为报文完整）
        std::vector<uint8_t> handshake(10);
        const auto host_ip_bytes = host_ip.to_v4().to_bytes();
        std::memcpy(handshake.data(), host_ip_bytes.data(), host_ip_bytes.size());
        write_le16(&handshake[4], options.data_port);
        write_le16(&handshake[6], options.cmd_port);
        write_le16(&handshake[8], options.imu_port);
        std::vector<uint8_t> response;
        std::string error;
        if (!co_await send_command(kCommandSetGeneral, kCommandHandshake, handshake, response, error)) {
            log(LogLevel::kWarn, "握手失败：" + error);
            co_return false;
        }
        if (response.empty() || response[0] != 0) {
            log(LogLevel::kWarn, "握手被拒绝：" + describe_command_result(error, response));
            co_return false;
        }

        // 使能采样，之后必须保持 >= 1Hz 心跳，否则设备会踢掉会话
        if (!co_await send_command(kCommandSetGeneral, kCommandControlSample, {1}, response, error)) {
            log(LogLevel::kWarn, "使能采样没有应答：" + error + "，继续尝试接收点云");
        }
        co_await apply_device_config();
        is_frame_base_reset.store(true, std::memory_order_relaxed);
        log(LogLevel::kInfo, "握手成功：雷达 " + device_ip.to_string() + ":" + std::to_string(device_port) +
                                     "，本机 " + host_ip.to_string() + "，点云端口 " + std::to_string(options.data_port));
        co_return true;
    }

    asio::awaitable<void> Mid70Driver::apply_device_config() {
        std::vector<uint8_t> response;
        std::string error;
        if (co_await send_command(kCommandSetLidar, kCommandGetReturnMode, {0}, response, error) && response.size() >= 2 && response[0] == 0) {
            log(LogLevel::kInfo, "设备当前回波模式 " + std::to_string(response[1]));
        } else {
            log(LogLevel::kWarn, "查询回波模式失败：" + describe_command_result(error, response));
        }
        if (options.return_mode != ReturnMode::kKeepCurrent) {
            error.clear();
            if (co_await send_command(kCommandSetLidar, kCommandSetReturnMode, {static_cast<uint8_t>(options.return_mode)}, response, error) && !response.empty() && response[0] == 0) {
                log(LogLevel::kInfo, "回波模式已设置（点云 data_type 从下个包开始变化）");
            } else {
                log(LogLevel::kError, "设置回波模式失败：" + describe_command_result(error, response));
            }
        }
        if (options.coordinate_mode != CoordinateMode::kKeepCurrent) {
            error.clear();
            if (co_await send_command(kCommandSetGeneral, kCommandCoordinateSystem, {static_cast<uint8_t>(options.coordinate_mode)}, response, error) && !response.empty() && response[0] == 0) {
                log(LogLevel::kInfo, "坐标格式已设置（设备重启后会复位为笛卡尔）");
            } else {
                log(LogLevel::kError, "设置坐标格式失败：" + describe_command_result(error, response));
            }
        }
        error.clear();
        if (co_await send_command(kCommandSetGeneral, kCommandDeviceInfo, {}, response, error) && response.size() >= 7 && response[0] == 0) {
            log(LogLevel::kInfo, "雷达固件版本 " + std::to_string(response[3]) + "." + std::to_string(response[4]) +
                                         "." + std::to_string(response[5]) + "." + std::to_string(response[6]));
        }
    }

    asio::awaitable<void> Mid70Driver::stream_pointcloud() {
        log(LogLevel::kInfo, "开始接收点云（心跳 >= 1Hz）");
        last_pointcloud_time = std::chrono::steady_clock::now();
        auto last_heartbeat_time = last_pointcloud_time;
        bool has_received_pointcloud = false;
        while (is_running.load(std::memory_order_relaxed)) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_heartbeat_time >= std::chrono::seconds(1)) {
                if (!co_await send_command_frame(cmd_socket, kCommandSetGeneral, kCommandHeartbeat, {})) {
                    break;
                }
                last_heartbeat_time = now;
            }
            // 等点云包；没有点云时定期醒过来发心跳、检查断流
            arm_timeout(get_time_after(kPointcloudWaitTime), data_socket);
            asio::ip::udp::endpoint sender_endpoint;
            asio::error_code error_code;
            const std::size_t length = co_await data_socket.async_receive_from(
                    asio::buffer(receive_buffer.data(), receive_buffer.size()),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            timeout_timer.cancel();
            if (error_code) {
                if (error_code != asio::error::operation_aborted) [[unlikely]] {
                    log(LogLevel::kError, "接收点云失败：" + error_code.message());
                    break;
                }
                if (!is_running.load(std::memory_order_relaxed)) {
                    break;
                }
            } else if (sender_endpoint.address() == device_ip && accumulate_pointcloud(receive_buffer.data(), length)) {
                last_pointcloud_time = std::chrono::steady_clock::now();
                has_received_pointcloud = true;
                on_receive_pointcloud(device_ip, points);
            }
            drain_command_socket();
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_pointcloud_time).count() > options.reconnect_timeout) {
                log(LogLevel::kWarn, has_received_pointcloud ? "点云中断，重新握手" : "未收到点云，重新握手");
                break;
            }
        }
    }

    asio::awaitable<bool> Mid70Driver::send_command(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data, std::vector<uint8_t> &response, std::string &error) {
        for (int attempt = 0; attempt < kCommandRetryTimes && is_running.load(std::memory_order_relaxed); ++attempt) {
            if (!co_await send_command_frame(cmd_socket, cmd_set, cmd_id, data)) {
                error = "发送失败";
                co_return false;
            }
            arm_timeout(get_time_after(kCommandTimeout), cmd_socket);
            while (is_running.load(std::memory_order_relaxed)) {
                uint8_t buffer[2048];
                asio::ip::udp::endpoint sender_endpoint;
                asio::error_code error_code;
                const std::size_t length = co_await cmd_socket.async_receive_from(
                        asio::buffer(buffer, sizeof(buffer)),
                        sender_endpoint,
                        asio::redirect_error(asio::use_awaitable, error_code));
                if (error_code) {
                    break;// 超时（定时器 cancel 掉等待）或 socket 出错
                }
                Frame frame;
                std::string unpack_error;
                if (!unpack_frame(buffer, length, frame, unpack_error)) {
                    continue;
                }
                if (handle_abnormal_push(frame.cmd_set, frame.cmd_id, frame.data)) {
                    continue;
                }
                if (frame.cmd_set == cmd_set && frame.cmd_id == cmd_id && (frame.packet_type == kPacketTypeAck || frame.packet_type == kPacketTypeMsg)) {
                    timeout_timer.cancel();
                    response = frame.data;
                    co_return true;
                }
            }
            timeout_timer.cancel();
            if (!is_running.load(std::memory_order_relaxed)) {
                break;
            }
        }
        error = "命令超时无应答（cmd_set=" + std::to_string(cmd_set) + " cmd_id=" + std::to_string(cmd_id) + "）";
        co_return false;
    }

    asio::awaitable<bool> Mid70Driver::send_command_frame(asio::ip::udp::socket &socket, uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data) {
        const std::vector<uint8_t> buffer = pack_frame(kPacketTypeCmd, ++sequence_number, cmd_set, cmd_id, data);
        asio::error_code error_code;
        co_await socket.async_send_to(
                asio::buffer(buffer),
                asio::ip::udp::endpoint(device_ip, device_port),
                asio::redirect_error(asio::use_awaitable, error_code));
        if (error_code) [[unlikely]] {
            if (is_running.load(std::memory_order_relaxed)) {
                log(LogLevel::kError, "发送命令失败：" + error_code.message());
            }
            co_return false;
        }
        co_return true;
    }

    asio::awaitable<void> Mid70Driver::sleep(double seconds) {
        sleep_timer.expires_at(get_time_after(seconds));
        asio::error_code error_code;
        co_await sleep_timer.async_wait(asio::redirect_error(asio::use_awaitable, error_code));
    }

    void Mid70Driver::arm_timeout(std::chrono::steady_clock::time_point deadline, asio::ip::udp::socket &socket) {
        timeout_timer.expires_at(deadline);
        timeout_timer.async_wait([this, &socket](const asio::error_code &error_code) {
            if (!error_code) {
                socket.cancel();
            }
        });
    }

    bool Mid70Driver::open_socket(asio::ip::udp::socket &socket, uint16_t port) {
        asio::error_code error_code;
        socket.open(asio::ip::udp::v4(), error_code);
        if (!error_code) {
            socket.bind(asio::ip::udp::endpoint(host_ip, port), error_code);
        }
        if (error_code) [[unlikely]] {
            log(LogLevel::kError, "绑定 " + host_ip.to_string() + ":" + std::to_string(port) + " 失败：" + error_code.message());
            socket.close();
            return false;
        }
        return true;
    }

    void Mid70Driver::close_session() {
        // 主动断开，让设备立刻回到广播态，不用等心跳超时
        if (cmd_socket.is_open() && !device_ip.is_unspecified()) {
            const std::vector<uint8_t> buffer = pack_frame(kPacketTypeCmd, ++sequence_number, kCommandSetGeneral, kCommandDisconnect, {});
            asio::error_code error_code;
            cmd_socket.send_to(asio::buffer(buffer), asio::ip::udp::endpoint(device_ip, device_port), 0, error_code);
        }
        asio::error_code error_code;
        cmd_socket.close(error_code);
        data_socket.close(error_code);
    }

    void Mid70Driver::drain_command_socket() {
        while (is_running.load(std::memory_order_relaxed) && cmd_socket.is_open()) {
            asio::error_code error_code;
            if (cmd_socket.available(error_code) == 0 || error_code) {
                return;
            }
            uint8_t buffer[2048];
            asio::ip::udp::endpoint sender_endpoint;
            const std::size_t length = cmd_socket.receive_from(asio::buffer(buffer, sizeof(buffer)), sender_endpoint, 0, error_code);
            if (error_code) {
                return;
            }
            Frame frame;
            std::string unpack_error;
            if (!unpack_frame(buffer, length, frame, unpack_error)) {
                continue;
            }
            handle_abnormal_push(frame.cmd_set, frame.cmd_id, frame.data);
        }
    }

    bool Mid70Driver::handle_abnormal_push(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data) {
        if (cmd_set != kCommandSetGeneral || cmd_id != kCommandPushAbnormal || data.size() < 6) {
            return false;
        }
        char status_code[16];
        std::snprintf(status_code, sizeof(status_code), "0x%08X", read_le32(&data[2]));
        log(LogLevel::kWarn, std::string("设备上报异常状态 status_code=") + status_code);
        return true;
    }

    bool Mid70Driver::accumulate_pointcloud(const uint8_t *buffer, std::size_t length) {
        if (length < kLivoxEthHeaderLength) [[unlikely]] {
            return false;
        }
        const auto &header = *reinterpret_cast<const EthHeader *>(buffer);
        const uint8_t data_type = header.data_type;
        if (data_type >= std::size(kPointSize) || kEchoCount[data_type] == 0) [[unlikely]] {
            return false;
        }
        double packet_timestamp;
        if (!get_packet_timestamp(header, packet_timestamp)) [[unlikely]] {
            return false;
        }
        const std::size_t point_size = kPointSize[data_type];
        const std::size_t point_count = (length - kLivoxEthHeaderLength) / point_size;
        if (point_count == 0) [[unlikely]] {
            return false;
        }
        // 一帧的第一个包建立时间基准：以它的到达时间为该帧点云时间戳的起点
        if (is_frame_base_reset.exchange(false, std::memory_order_relaxed) || !has_frame_base) {
            frame_base_device_time = packet_timestamp;
            frame_base_timestamp = get_current_time();
            has_frame_base = true;
        }
        const double packet_offset = packet_timestamp - frame_base_device_time;
        points.clear();
        points.reserve(point_count);
        for (std::size_t i = 0; i < point_count; ++i) {
            DecodedPoint decoded_point;
            if (!decode_first_echo(&buffer[kLivoxEthHeaderLength + i * point_size], data_type, decoded_point)) {
                continue;
            }
            if ((decoded_point.tag & 0b00111111) != 0) [[unlikely]] {
                continue;// 与 Mid-360 驱动一致：丢弃噪点与后续回波
            }
            if (options.drop_invalid_points && (decoded_point.x == 0.0f && decoded_point.y == 0.0f && decoded_point.z == 0.0f)) [[unlikely]] {
                continue;// 无回波的填充点
            }
            if (options.drop_invalid_points && (std::fabs(decoded_point.x) > kMaxValidDistance || std::fabs(decoded_point.y) > kMaxValidDistance || std::fabs(decoded_point.z) > kMaxValidDistance)) [[unlikely]] {
                continue;// 离群点
            }
            Point point;
            point.timestamp = frame_base_timestamp + packet_offset + static_cast<double>(i) * kPointPeriod;
            point.x = decoded_point.x;
            point.y = decoded_point.y;
            point.z = decoded_point.z;
            point.intensity = static_cast<float>(decoded_point.reflectivity);
            points.push_back(point);
        }
        return !points.empty();
    }

    asio::ip::address Mid70Driver::detect_local_ip() {
        if (device_ip.is_unspecified()) {
            return asio::ip::address();
        }
        // 用一次 connect 让内核按路由选出本机地址，UDP 不会真的发包
        asio::ip::udp::socket socket(io_context);
        asio::error_code error_code;
        socket.open(asio::ip::udp::v4(), error_code);
        if (!error_code) {
            socket.connect(asio::ip::udp::endpoint(device_ip, device_port), error_code);
        }
        asio::ip::address local_ip;
        if (!error_code) {
            local_ip = socket.local_endpoint(error_code).address();
        }
        socket.close();
        if (error_code) {
            return asio::ip::address();
        }
        return local_ip;
    }

}// namespace mid70_driver
