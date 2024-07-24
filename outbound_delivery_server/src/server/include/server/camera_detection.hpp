#pragma once

#include <crow.h>
#include <memory.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;

class CameraDetection : public rclcpp::Node
{
    public:
        CameraDetection() : Node("camera_detection") ,bp("camera")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("detection", 10);

            CROW_BP_ROUTE(bp, "detection").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }
                std::cout << data << std::endl;

                auto boxDetection = data["boxes"];
                auto robotDetection = data["robots"];
                auto personDetection = data["persons"];

                if (boxDetection == "1")
                {
                    detection_msg.data = "box";
                    publisher_->publish(detection_msg);
                }

                if (robotDetection == "1")
                {
                    detection_msg.data = "robot";
                    publisher_->publish(detection_msg);
                }

                if (personDetection == "1")
                {
                    detection_msg.data = "person";
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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std_msgs::msg::String detection_msg;
};