#pragma once

#include <crow.h>
#include <memory.h>
#include <string>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>

#include "rclcpp/rclcpp.hpp"
#include "database_connection.hpp"
#include "outbound_delivery_robot_interfaces/msg/location.hpp"

class Destination : public DatabaseConnection, rclcpp::Node
{
    public:
        Destination(const std::string& configFile): DatabaseConnection(configFile), Node("send_destination"), bp("destination")
        {
            publisher_ = this->create_publisher<outbound_delivery_robot_interfaces::msg::Location>("location", 10);

            CROW_BP_ROUTE(bp, "/send").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) {return crow::response(400, "Invalid JSON"); }

                auto robot_id = data["robot_id"].s();
                auto product = data["task"].s();

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT * FROM location WHERE product=?"));
                pstmt->setString(1, std::string(product));

                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
                if (res->next())
                {
                    location_msg.robot_id = std::stoi(robot_id);
                    location_msg.section = res->getString("section");
                    location_msg.x = res->getDouble("x");
                    location_msg.y = res->getDouble("y");
                    location_msg.z = res->getDouble("z");
                    location_msg.w = res->getDouble("w");

                    publisher_->publish(location_msg);
                }

                return crow::response(200, "Location published successfully\n");
            });
        }

        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
        rclcpp::Publisher<outbound_delivery_robot_interfaces::msg::Location>::SharedPtr publisher_;
        outbound_delivery_robot_interfaces::msg::Location location_msg;
};