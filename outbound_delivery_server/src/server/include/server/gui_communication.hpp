#pragma once

#include <crow.h>

#include <memory>

#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>

#include <nlohmann/json.hpp>

#include "database_connection.hpp"

using json = nlohmann::json;

class GuiCommunication : public DatabaseConnection
{
    public:
        GuiCommunication(const std::string configFile) : DatabaseConnection(configFile), bp("gui")
        {
            CROW_BP_ROUTE(bp, "/info").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                std::string request = data["request"].s();

                if (request == "call")
                {
                    auto conn = Connection();

                    std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT robot_status, robot_task, robot_goal from robot_info"));
                    std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());

                    json robotInfoArray = json::array();
                    while(res->next())
                    {
                        json robotInfo;
                        robotInfo["robot_status"] = res->getInt("robot_status");
                        robotInfo["robot_task"] = res->getString("robot_task");
                        robotInfo["robot_goal"] = res->getString("robot_goal");

                        robotInfoArray.push_back(robotInfo);
                    }

                    return crow::response(robotInfoArray.dump());
                }
                else
                {
                    return crow::response(400, "Invalid action");
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