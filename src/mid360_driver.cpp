/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#define _USE_MATH_DEFINES
#include "mid360_driver.h"
#include <chrono>
#include <cmath>

namespace mid360_driver {

    enum DataType : std::uint8_t {
        kLivoxLidarImuData = 0,
        kLivoxLidarCartesianCoordinateHighData = 0x01,
        kLivoxLidarCartesianCoordinateLowData = 0x02,
        kLivoxLidarSphericalCoordinateData = 0x03
    };

    enum TimestampType : std::uint8_t {
        kTimestampTypeNoSync = 0,   // 没有同步信号
        kTimestampTypeGptpOrPtp = 1,// gPTP 或 PTP 同步模式
        kTimestampTypeGps = 2       // GPS 同步模式
    };

#pragma pack(1)

    struct DataHeader {
        uint8_t version;        // 包协议版本：当前为0
        uint16_t length;        // 整个UDP数据段长度
        uint16_t time_interval; // 帧内点云采样时间(单位0.1us)；这帧点云数据中最后一个点减去第一个点时间
        uint16_t dot_num;       // 当前UDP包的点数目
        uint16_t udp_cnt;       // 点云UDP包计数，每个UDP包依次加1，点云帧起始包清0
        uint8_t frame_cnt;      // 点云帧计数，每帧点云(10Hz/15Hz等)依次加1
        DataType data_type;     // 数据类型
        TimestampType time_type;// 时间戳类型
        uint8_t reserved[12];   // 保留
        uint32_t crc32;         // timestamp+点云数据校验码，使用CRC-32算法
        uint64_t timestamp;     // 点云时间戳，单位: ns
    };

    struct Imu {
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct CartesianHighPoint {
        int32_t x;           // unit:mm
        int32_t y;           // unit:mm
        int32_t z;           // unit:mm
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

    struct CartesianLowPoint {
        int16_t x;           // unit:mm
        int16_t y;           // unit:mm
        int16_t z;           // unit:mm
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

    struct SphericalPoint {
        uint32_t depth;      // 深度
        uint16_t theta;      // 天顶角[0, 18000], Unit: 0.01 度
        uint16_t phi;        // 方位角[0, 36000], Unit: 0.01 度
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

#pragma pack()

    Mid360Driver::Mid360Driver(asio::io_context &io_context,
                               const asio::ip::address &host_ip,
                               const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                               const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu)
        : host_ip(host_ip),
          receive_pointcloud_socket(io_context),
          receive_imu_socket(io_context),
          on_receive_pointcloud(on_receive_pointcloud),
          on_receive_imu(on_receive_imu) {
        receive_pointcloud_socket.open(asio::ip::udp::v4());
        receive_pointcloud_socket.bind(asio::ip::udp::endpoint(host_ip, 56301));
        receive_imu_socket.open(asio::ip::udp::v4());
        receive_imu_socket.bind(asio::ip::udp::endpoint(host_ip, 56401));
        co_spawn(io_context, receive_pointcloud(), asio::detached);
        co_spawn(io_context, receive_imu(), asio::detached);
    }

    Mid360Driver::~Mid360Driver() {
        stop();
    }

    void Mid360Driver::stop() {
        is_running.store(false, std::memory_order_relaxed);
        asio::error_code error_code;
        receive_pointcloud_socket.close(error_code);
        receive_imu_socket.close(error_code);
    }

    asio::awaitable<void> Mid360Driver::receive_pointcloud() {
        uint8_t buffer[1400];
        asio::ip::udp::endpoint sender_endpoint;
        while (is_running.load(std::memory_order_relaxed)) {
            asio::error_code error_code;
            co_await receive_pointcloud_socket.async_receive_from(
                    asio::buffer(buffer, 1400),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            if (error_code || sender_endpoint.port() != 56300) [[unlikely]] {
                continue;
            }
            const auto &header = *reinterpret_cast<const DataHeader *>(buffer);
            double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                auto [iter, inserted] = delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    auto now = static_cast<double>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) * 1e-9;
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }
            points.clear();
            points.reserve(header.dot_num);
            if (header.data_type == DataType::kLivoxLidarCartesianCoordinateHighData) {
                const auto *raw_points = reinterpret_cast<const CartesianHighPoint *>(buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if ((raw_point.tag & 0b00110000) != 0b00000000 || (raw_point.tag & 0b00001100) != 0b00000000 || (raw_point.tag & 0b00000011) != 0b00000000) {
                        continue;
                    }
                    Point point;// NOLINT(cppcoreguidelines-pro-type-member-init)
                    point.timestamp = header_timestamp + static_cast<double>(header.time_interval * i) / header.dot_num * 1e-10;
                    point.x = static_cast<float>(raw_point.x * 0.001);
                    point.y = static_cast<float>(raw_point.y * 0.001);
                    point.z = static_cast<float>(raw_point.z * 0.001);
                    point.intensity = raw_point.reflectivity;
                    points.push_back(point);
                }
            } else if (header.data_type == DataType::kLivoxLidarCartesianCoordinateLowData) {
                const auto *raw_points = reinterpret_cast<const CartesianLowPoint *>(buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if ((raw_point.tag & 0b00110000) != 0b00000000 || (raw_point.tag & 0b00001100) != 0b00000000 || (raw_point.tag & 0b00000011) != 0b00000000) {
                        continue;
                    }
                    Point point;// NOLINT(cppcoreguidelines-pro-type-member-init)
                    point.timestamp = header_timestamp + static_cast<double>(header.time_interval * i) / header.dot_num * 1e-10;
                    point.x = static_cast<float>(raw_point.x * 0.001);
                    point.y = static_cast<float>(raw_point.y * 0.001);
                    point.z = static_cast<float>(raw_point.z * 0.001);
                    point.intensity = raw_point.reflectivity;
                    points.push_back(point);
                }
            } else if (header.data_type == DataType::kLivoxLidarSphericalCoordinateData) {
                const auto *raw_points = reinterpret_cast<const SphericalPoint *>(buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &raw_point = raw_points[i];
                    if ((raw_point.tag & 0b00110000) != 0b00000000 || (raw_point.tag & 0b00001100) != 0b00000000 || (raw_point.tag & 0b00000011) != 0b00000000) {
                        continue;
                    }
                    Point point;// NOLINT(cppcoreguidelines-pro-type-member-init)
                    double radius = raw_point.depth / 1000.0;
                    double theta = raw_point.theta / 100.0 / 180 * M_PI;
                    double phi = raw_point.phi / 100.0 / 180 * M_PI;
                    point.x = static_cast<float>(radius * sin(theta) * cos(phi));
                    point.y = static_cast<float>(radius * sin(theta) * sin(phi));
                    point.z = static_cast<float>(radius * cos(theta));
                    point.intensity = raw_point.reflectivity;
                    points.push_back(point);
                }
            }
            on_receive_pointcloud(sender_endpoint.address(), points);
        }
    }

    asio::awaitable<void> Mid360Driver::receive_imu() {
        uint8_t buffer[1400];
        asio::ip::udp::endpoint sender_endpoint;
        while (is_running.load(std::memory_order_relaxed)) {
            asio::error_code error_code;
            co_await receive_imu_socket.async_receive_from(
                    asio::buffer(buffer, 1400),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, error_code));
            if (error_code || sender_endpoint.port() != 56400) [[unlikely]] {
                continue;
            }
            const auto &header = *reinterpret_cast<const DataHeader *>(buffer);
            double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                auto [iter, inserted] = delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    auto now = static_cast<double>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) * 1e-9;
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }
            const auto &raw_imu = *reinterpret_cast<const Imu *>(buffer + sizeof(DataHeader));
            ImuMsg imu_msg;// NOLINT(cppcoreguidelines-pro-type-member-init)
            imu_msg.timestamp = header_timestamp;
            imu_msg.angular_velocity_x = raw_imu.angular_velocity_x;
            imu_msg.angular_velocity_y = raw_imu.angular_velocity_y;
            imu_msg.angular_velocity_z = raw_imu.angular_velocity_z;
            imu_msg.linear_acceleration_x = raw_imu.linear_acceleration_x;
            imu_msg.linear_acceleration_y = raw_imu.linear_acceleration_y;
            imu_msg.linear_acceleration_z = raw_imu.linear_acceleration_z;
            on_receive_imu(sender_endpoint.address(), imu_msg);
        }
    }

}// namespace mid360_driver
