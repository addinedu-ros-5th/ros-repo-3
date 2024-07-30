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
            CROW_BP_ROUTE(bp, "picking").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }
                std::cout << data << std::endl;

                auto status = data["tag_data"];

                if (status == "Success")
                {
                    // TODO
                }

                return crow::response(200, "Publish successfully");
            });
        }
        
        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
};