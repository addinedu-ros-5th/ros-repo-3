#pragma once

#include <crow.h>
#include <memory.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <nlohmann/json.hpp>

#include "database_connection.hpp"

using json = nlohmann::json;

class Outbound : public DatabaseConnection
{
    public:
        Outbound(const std::string configFile): DatabaseConnection(configFile), bp("tag")
        {
            CROW_BP_ROUTE(bp, "/picking").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                auto robotId = data["robot_id"];
                auto productId = data["product_id"];

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt1(conn->prepareStatement("SELECT current_task_order_id FROM robot_info WHERE robot_id = ?"));
                pstmt1->setString(1, std::string(robotId));
                std::unique_ptr<sql::ResultSet> res1(pstmt1->executeQuery());

                int currentOrderId = 0;

                if (res1->next()) 
                {
                    currentOrderId = res1->getInt("current_task_order_id");
                } 
                else 
                {
                    return crow::response(404, "Robot ID not found");
                }

                std::unique_ptr<sql::PreparedStatement> pstmt2(conn->prepareStatement("SELECT product_name FROM product_info WHERE product_id = ?"));
                pstmt2->setString(1, std::string(productId));
                std::unique_ptr<sql::ResultSet> res2(pstmt2->executeQuery());

                std::string productName = "";

                if (res2->next()) 
                {
                    productName = res2->getString("product_name");
                } 
                else 
                {
                    return crow::response(404, "Product ID not found");
                }

                std::unique_ptr<sql::PreparedStatement> pstmt3(conn->prepareStatement("UPDATE order_details SET quantity = quantity - 1 WHERE order_id = ? AND product_name = ?"));
                pstmt3->setInt(1, currentOrderId);
                pstmt3->setString(2, productName);
                int affectedRows = pstmt3->executeUpdate();

                if (affectedRows > 0) 
                {
                    return crow::response(200, "Quantity updated successfully");
                } 
                else 
                {
                    return crow::response(404, "Product not found in order details");
                }
        });
    }
        
        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
};