/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver_node.h"
#include "mid70_driver.h"
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <thread>

namespace mid70_driver {

    class Mid70DriverNode : public rclcpp::Node {
    private:
        asio::io_context io_context;
        std::mutex mutex;
        std::thread io_thread;
        std::unique_ptr<mid70_driver::Mid70Driver> mid70_driver;
        // 复用 Mid-360 的点云发布器，两个驱动的点云格式与话题行为一致（Mid-70 无 IMU）
        mid360_driver::LidarPublisher lidar_publisher;
        rclcpp::TimerBase::SharedPtr publish_pointcloud_timer;

    public:
        explicit Mid70DriverNode(const rclcpp::NodeOptions &options);

        ~Mid70DriverNode();
    };

}// namespace mid70_driver
