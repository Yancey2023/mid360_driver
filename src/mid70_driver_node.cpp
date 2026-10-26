/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "mid70_driver_node.h"

namespace mid70_driver {

    static asio::ip::address parse_address(const std::string &text, const rclcpp::Logger &logger, const char *name) {
        if (text.empty()) {
            return asio::ip::address_v4::any();// 空 = 自动探测本机 IP / 自动发现雷达
        }
        asio::error_code error_code;
        const asio::ip::address address = asio::ip::make_address(text, error_code);
        if (error_code) [[unlikely]] {
            RCLCPP_ERROR(logger, "%s 不是合法 IP：%s，将按自动探测处理", name, text.c_str());
            return asio::ip::address_v4::any();
        }
        return address;
    }

    static ReturnMode parse_return_mode(const std::string &text, const rclcpp::Logger &logger) {
        if (text.empty()) {
            return ReturnMode::kKeepCurrent;
        }
        if (text == "first") {
            return ReturnMode::kFirst;
        }
        if (text == "strongest") {
            return ReturnMode::kStrongest;
        }
        if (text == "dual") {
            return ReturnMode::kDual;
        }
        if (text == "triple") {
            return ReturnMode::kTriple;
        }
        RCLCPP_ERROR(logger, "return_mode 只能是 first/strongest/dual/triple 或留空，%s 将被忽略", text.c_str());
        return ReturnMode::kKeepCurrent;
    }

    static CoordinateMode parse_coordinate_mode(const std::string &text, const rclcpp::Logger &logger) {
        if (text.empty()) {
            return CoordinateMode::kKeepCurrent;
        }
        if (text == "cartesian") {
            return CoordinateMode::kCartesian;
        }
        if (text == "spherical") {
            return CoordinateMode::kSpherical;
        }
        RCLCPP_ERROR(logger, "coordinate_mode 只能是 cartesian/spherical 或留空，%s 将被忽略", text.c_str());
        return CoordinateMode::kKeepCurrent;
    }

    Mid70DriverNode::Mid70DriverNode(const rclcpp::NodeOptions &options)
        : Node("mid70_driver_node", options),
          lidar_publisher(false) {
        const std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        const std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        const double lidar_publish_time_interval = declare_parameter<double>("lidar_publish_time_interval");
        Mid70Driver::Options driver_options;
        driver_options.host_ip = parse_address(declare_parameter<std::string>("host_ip"), get_logger(), "host_ip");
        driver_options.lidar_ip = parse_address(declare_parameter<std::string>("lidar_ip"), get_logger(), "lidar_ip");
        driver_options.device_port = static_cast<uint16_t>(declare_parameter<int>("device_port"));
        driver_options.discovery_port = static_cast<uint16_t>(declare_parameter<int>("discovery_port"));
        driver_options.cmd_port = static_cast<uint16_t>(declare_parameter<int>("cmd_port"));
        driver_options.data_port = static_cast<uint16_t>(declare_parameter<int>("data_port"));
        driver_options.imu_port = static_cast<uint16_t>(declare_parameter<int>("imu_port"));
        driver_options.return_mode = parse_return_mode(declare_parameter<std::string>("return_mode"), get_logger());
        driver_options.coordinate_mode = parse_coordinate_mode(declare_parameter<std::string>("coordinate_mode"), get_logger());
        driver_options.discovery_timeout = declare_parameter<double>("discovery_timeout");
        driver_options.reconnect_timeout = declare_parameter<double>("reconnect_timeout");
        driver_options.drop_invalid_points = declare_parameter<bool>("drop_invalid_points");

        // Mid-70 没有 IMU，imu_topic 只是占位（构造时已经关掉 IMU 发布者）
        lidar_publisher.make_sure_init(*this, lidar_topic, "");
        mid70_driver = std::make_unique<mid70_driver::Mid70Driver>(
                io_context,
                driver_options,
                [this](const asio::ip::address &, const std::vector<Point> &points) {
                    mutex.lock();
                    lidar_publisher.on_receive_pointcloud(points);
                    mutex.unlock();
                },
                [this](LogLevel level, const std::string &message) {
                    switch (level) {
                        case LogLevel::kInfo:
                            RCLCPP_INFO(get_logger(), "%s", message.c_str());
                            break;
                        case LogLevel::kWarn:
                            RCLCPP_WARN(get_logger(), "%s", message.c_str());
                            break;
                        case LogLevel::kError:
                            RCLCPP_ERROR(get_logger(), "%s", message.c_str());
                            break;
                    }
                });
        publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double, std::ratio<1, 1>>(lidar_publish_time_interval), [this, lidar_frame]() {
            mutex.lock();
            lidar_publisher.prepare_pointcloud_to_publish();
            mutex.unlock();
            mid70_driver->reset_frame();
            lidar_publisher.publish_pointcloud(lidar_frame);
        });
        io_thread = std::thread([this]() {
            io_context.run();
        });
    }

    Mid70DriverNode::~Mid70DriverNode() {
        if (mid70_driver) {
            mid70_driver->stop();
        }
        io_thread.join();
    }

}// namespace mid70_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mid70_driver::Mid70DriverNode)
