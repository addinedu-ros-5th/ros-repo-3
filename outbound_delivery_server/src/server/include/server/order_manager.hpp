#pragma once

#include <crow.h>
#include <memory.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <nlohmann/json.hpp>

#include "database_connection.hpp"

using json = nlohmann::json;

class OrderManager : public DatabaseConnection
{
    public:
        OrderManager(const std::string& configFile): DatabaseConnection(configFile), bp("database")
        {
            CROW_BP_ROUTE(bp, "/customer_order").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) {return crow::response(400, "Invalid JSON"); }
                std::cout << data << std::endl;
                
                auto items = data["items"];
                std::string orderTime = data["orderTime"].s();

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("INSERT INTO orders (order_time) VALUE(?)"));
                pstmt->setString(1, orderTime);
                pstmt->execute();
                
                std::unique_ptr<sql::Statement> stmt(conn->createStatement());
                std::unique_ptr<sql::ResultSet> res(stmt->executeQuery("SELECT LAST_INSERT_ID()"));
                res->next();
                int64_t orderId = res->getInt64(1);

                std::unique_ptr<sql::PreparedStatement> pstmt_details(conn->prepareStatement("INSERT INTO order_details (order_id, product_name, order_time, quantity, price) VALUES (?, ?, ?, ?, ?)"));
                for (const auto& item : items)
                {
                    std::string productName = item["productName"].s();
                    int quantity = item["productAmount"].i();
                    int price = item["productPrice"].i();

                    pstmt_details->setInt64(1, orderId);
                    pstmt_details->setString(2, productName);
                    pstmt_details->setString(3, orderTime);
                    pstmt_details->setInt(4, quantity);
                    pstmt_details->setDouble(5, price);
                    pstmt_details->execute();
                }

                conn->commit();

                return crow::response(200, "Order upload successfully");
            });

            CROW_BP_ROUTE(bp, "/order").methods(crow::HTTPMethod::GET)([this](const crow::request& req)
            {
                auto data = req.url_params.get("date");
                if (!data) { return crow::response(400, "Invalid param"); }

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT order_id, order_time FROM orders WHERE order_time = ? ORDER BY order_id"));
                pstmt->setString(1, data);
                
                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
                json orders = json::array();
                while(res->next())
                {
                    json order;
                    order["order_id"] = res->getInt("order_id");
                    order["order_time"] = res->getString("order_time");
                    orders.push_back(order);
                }

                json response = orders;

                return crow::response(response.dump());
            });

            CROW_BP_ROUTE(bp, "/order_detail").methods(crow::HTTPMethod::GET)([this](const crow::request& req)
            {
                auto data = req.url_params.get("order_id");
                if (!data) { return crow::response(400, "Invalid param"); }

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT d.product_name, d.quantity FROM orders o JOIN order_details d ON o.order_id = d.order_id WHERE o.order_id = ? ORDER BY o.order_time"));
                pstmt->setString(1, std::string(data));

                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
                json orderDetails = json::array();
                while (res->next()) 
                {
                    json detail;
                    detail["product_name"] = res->getString("product_name");
                    detail["quantity"] = res->getInt("quantity");
                    orderDetails.push_back(detail);
                }

                return crow::response(orderDetails.dump());
            });

            CROW_BP_ROUTE(bp, "/image").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid param"); }
                auto conn = Connection();

                auto product = data["product"];

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT image_path FROM product_image_path WHERE product_name = ?"));
                pstmt->setString(1, std::string(product));

                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
                res->next();

                json imageInfo;
                imageInfo["image_path"] = res->getString("image_path");

                return crow::response(imageInfo.dump());
            });
        }

        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
};