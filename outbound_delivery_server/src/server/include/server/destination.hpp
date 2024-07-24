#pragma once

#include <crow.h>
#include <memory.h>
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
                auto data = req.url_params.get("product");
                if (!data) { return crow::response(400, "Invalid param"); }

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT * FROM location WHERE product=?"));
                pstmt->setString(1, data);

                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
                if (res->next())
                {
                    location_msg.section = res->getString(1);
                    location_msg.x = res->getDouble(2);
                    location_msg.y = res->getDouble(3);
                    location_msg.z = res->getDouble(4);
                    location_msg.w = res->getDouble(5);

                    publisher_->publish(location_msg);
                }

                return crow::response(200, "Location published successfully");
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