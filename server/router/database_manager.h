#pragma once

#include <crow.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include "../connection/database_connection.h"

class DatabaseManager : public DatabaseConnector 
{
public:
    DatabaseManager()
    : bp("database")
    {
        CROW_BP_ROUTE(bp, "/customer_order").methods("POST"_method)([this](const crow::request& req)
        {
            auto data = crow::json::load(req.body);
            if (!data) { return crow::response(400, "Invalid JSON"); }

            std::string productName = data["productName"].s();
            int productAmount = data["productAmount"].i();

            conn = connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("INSERT INTO orders (productName, productAmount) VALUES (?, ?)"));
            pstmt->setString(1, productName);
            pstmt->setInt(2, productAmount);
            pstmt->execute();

            return crow::response(200, "Data upload successfully");
        });
    }

    crow::Blueprint& getBlueprint()
    {
        return bp;
    }

private:
    crow::Blueprint bp;
};