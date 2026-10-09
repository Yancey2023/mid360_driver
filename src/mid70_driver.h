/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
#include "mid360_driver.h"
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mid70_driver {

    // 复用 Mid-360 的点结构与发布器，两个驱动的点云格式与话题行为保持一致
    using Point = mid360_driver::Point;

    // 会话日志，由节点转成 ROS 日志
    enum class LogLevel : std::uint8_t {
        kInfo = 0,
        kWarn = 1,
        kError = 2
    };

    // 设备回波模式（Livox 私有协议 cmd_set=1 cmd_id=6）
    enum class ReturnMode : std::uint8_t {
        kKeepCurrent = 0xFF,// 不下发，保持设备当前设置
        kFirst = 0,         // 首次回波
        kStrongest = 1,     // 最强回波
        kDual = 2,          // 双回波
        kTriple = 3         // 三回波，Mid-70 不支持，下发会被设备拒绝
    };

    // 设备坐标格式（Livox 私有协议 cmd_set=0 cmd_id=5）
    enum class CoordinateMode : std::uint8_t {
        kKeepCurrent = 0xFF,// 不下发，保持设备当前设置
        kCartesian = 0,     // 笛卡尔坐标
        kSpherical = 1      // 球坐标
    };

    /**
     * Mid-70 的 UDP 会话：广播发现 -> 握手 -> 使能采样 -> 收点云 + 心跳，断流后自动重连。
     *
     * 与 Mid-360 不同，Mid-70 的会话是有状态的（没有心跳设备会踢掉连接），
     * 所以这里自己维护发现/握手/心跳，点云包按 Livox 私有格式解析后转成统一的 Point。
     */
    class Mid70Driver {
    public:
        struct Options {
            asio::ip::address host_ip;      // 本机 IP，未指定时按到雷达的路由自动探测
            asio::ip::address lidar_ip;     // 雷达 IP，未指定时监听广播自动发现
            uint16_t device_port = 65000;   // 雷达命令/点云端口
            uint16_t discovery_port = 55000;// 雷达广播的目标端口
            uint16_t cmd_port = 55001;      // 本机收命令应答
            uint16_t data_port = 56001;     // 本机收点云
            uint16_t imu_port = 56002;      // Mid-70 无 IMU，仅握手报文需要
            ReturnMode return_mode = ReturnMode::kKeepCurrent;
            CoordinateMode coordinate_mode = CoordinateMode::kKeepCurrent;
            double discovery_timeout = 10.0;// 单次发现等待时间 [s]
            double reconnect_timeout = 3.0; // 点云中断多久后重新握手 [s]
            bool drop_invalid_points = true;// 丢弃 (0,0,0) 无回波填充点与 >1000m 离群点
        };

        Mid70Driver(asio::io_context &io_context,
                    const Options &options,
                    const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                    const std::function<void(LogLevel level, const std::string &message)> &on_log);

        ~Mid70Driver();

        void stop();

        // 每发布一帧点云后调用：下一包重新建立该帧的时间基准
        void reset_frame();

    private:
        asio::awaitable<void> run();

        asio::awaitable<bool> discover_device();

        asio::awaitable<bool> connect_device();

        asio::awaitable<void> apply_device_config();

        asio::awaitable<void> stream_pointcloud();

        // 发送命令并等待同 cmd_set/cmd_id 的应答，超时自动重试；应答数据段放在 response
        asio::awaitable<bool> send_command(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data, std::vector<uint8_t> &response, std::string &error);

        asio::awaitable<bool> send_command_frame(asio::ip::udp::socket &socket, uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data);

        asio::awaitable<void> sleep(double seconds);

        // 给一次 socket 等待设置超时：定时器到点就 cancel 掉这个等待，等待会以 operation_aborted 返回
        void arm_timeout(std::chrono::steady_clock::time_point deadline, asio::ip::udp::socket &socket);

        bool open_socket(asio::ip::udp::socket &socket, uint16_t port);

        void close_session();

        void drain_command_socket();

        // 设备异常状态主动上报（cmd_set=0 cmd_id=7）：记录日志，返回是否已被消化
        bool handle_abnormal_push(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t> &data);

        bool accumulate_pointcloud(const uint8_t *buffer, std::size_t length);

        asio::ip::address detect_local_ip();

        void log(LogLevel level, const std::string &message);

        asio::io_context &io_context;
        Options options;
        asio::ip::udp::socket discovery_socket;
        asio::ip::udp::socket cmd_socket;
        asio::ip::udp::socket data_socket;
        asio::steady_timer timeout_timer;
        asio::steady_timer sleep_timer;
        std::atomic<bool> is_running = true;
        // 该帧的时间基准是否需要重建（发布一帧点云后或新会话开始时置位）
        std::atomic<bool> is_frame_base_reset = true;
        asio::ip::address host_ip;
        asio::ip::address device_ip;
        uint16_t device_port = 0;
        std::string device_serial_number;
        uint16_t sequence_number = 0;
        std::vector<uint8_t> receive_buffer;
        std::vector<Point> points;
        bool has_frame_base = false;
        double frame_base_device_time = 0.0;
        double frame_base_timestamp = 0.0;
        std::chrono::steady_clock::time_point last_pointcloud_time;
        int connect_failure_count = 0;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(LogLevel level, const std::string &message)> on_log;
    };

}// namespace mid70_driver
