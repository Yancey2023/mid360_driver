/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver.h"
#include <map>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mid360_driver {

    class LidarPublisher {
    private:
        std::mutex mutex;
        std::vector<Point> points_to_publish;
        std::vector<ImuMsg> imu_to_publish;
        std::vector<Point> points_to_publish_temp;
        std::vector<ImuMsg> imu_to_publish_temp;
        bool is_init = false;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;

    public:
        LidarPublisher() = default;

        void make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic);

        void make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip);

        void on_receive_pointcloud(const std::vector<Point> &points);

        void on_receive_imu(const ImuMsg &imu_msg);

        void publish_pointcloud(const std::string &frame_id);

        void publish_imu(const std::string &frame_id);
    };

    class Mid360DriverNode : public rclcpp::Node {
    private:
        asio::io_context io_context;
        std::thread livox_driver_thread;
        std::unique_ptr<mid360_driver::Mid360Driver> mid360_driver;
        LidarPublisher lidar_publisher;
        std::map<asio::ip::address, LidarPublisher> muti_lidar_publisher;
        rclcpp::TimerBase::SharedPtr publish_pointcloud_timer;
        rclcpp::TimerBase::SharedPtr publish_imu_timer;

    public:
        explicit Mid360DriverNode(const rclcpp::NodeOptions &options);

        ~Mid360DriverNode();
    };

}// namespace mid360_driver
