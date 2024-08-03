#pragma once

#include <crow.h>
#include <memory.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using json = nlohmann::json;

class CameraDetection : public rclcpp::Node
{
    public:
        CameraDetection() : Node("camera_detection") ,bp("camera")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("detection", 10);

            CROW_BP_ROUTE(bp, "/detection").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                auto boxDetection = data["boxes"];
                auto robotDetection = data["robots"];
                auto personDetection = data["persons"];

                if (boxDetection == 1 || robotDetection == 1 || personDetection == 1)
                {
                    detection_msg.data = 1;
                    publisher_->publish(detection_msg);
                }
                else
                {
                    detection_msg.data = 0;
                    publisher_->publish(detection_msg);
                }

                return crow::response(200, "Detection published successfully");
            });
        }

    crow::Blueprint& getBlueprint()
    {
        return bp;
    }

    private:
        crow::Blueprint bp;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        std_msgs::msg::Int32 detection_msg;
};