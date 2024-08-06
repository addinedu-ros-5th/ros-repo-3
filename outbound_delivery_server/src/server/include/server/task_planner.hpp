#pragma once

#include <crow.h>

#include <queue>
#include <vector>
#include <string>
#include <memory.h>

#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include "database_connection.hpp"
#include "address.hpp"

using json = nlohmann::json;

class TaskPlanner : public DatabaseConnection
{
    public:
        TaskPlanner(const std::string configFile) : DatabaseConnection(configFile) , bp("task"), isNext(4, true), isEnd(4, false)
        {
            CROW_BP_ROUTE(bp, "/manage").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                std::cout << data << std::endl;

                auto orderId = data["order_id"];
                auto date = data["date"];

                auto conn = Connection();

                std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT d.product_name FROM order_info o JOIN order_details d ON o.order_id = d.order_id WHERE o.order_id = ? AND o.order_time = ?"));
                pstmt->setString(1, std::string(orderId));
                pstmt->setString(2, std::string(date));

                std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());

                std::queue<std::string> tasks;

                while (res->next())
                {
                    tasks.push(std::string(res->getString("product_name")));
                }

                std::cout << "Assigned workload: " << tasks.size() << std::endl;

                int robotId = availableRobotId();
                std::cout << "Available robot: " << robotId << std::endl;;
                
                if (robotId == -1)
                {
                    std::cout << "All robots are assigned" << std::endl;
                }
                else
                {
                    assignTasksToRobot(robotId, std::move(tasks));

                    std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("UPDATE robot_info SET current_task_order_id = ?, current_task_order_time = ? WHERE robot_id = ?"));
                    pstmt->setString(1, std::string(orderId));
                    pstmt->setString(2, std::string(date));
                    pstmt->setInt(3, robotId);
                    pstmt->execute();
                }

                json response;
                response["robot_id"] = robotId;

                return crow::response(response.dump());
            });

            CROW_BP_ROUTE(bp, "/process").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                int robotId = data["robot_id"].i();
                std::string process = data["process"].s();

                if (process == "finish")
                {
                    setIsNext(robotId, true);
                    cv.notify_all();
                }

                return crow::response(200, "Success\n");
            });

            CROW_BP_ROUTE(bp, "/end").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                std::string process = data["process"].s();
                std::vector<int>robotId;

                if (process == "end")
                {
                    robotId = getRobotId("UNLOADING");
                    for(const auto& id : robotId)
                    {
                        setIsEnd(id, true);
                    }
                }

                return crow::response(200, "Success\n");
            });

            CROW_BP_ROUTE(bp, "/update").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }
                std::cout << data << std::endl;

                auto robotId = data["robot_id"];
                auto status = data["status"];

                setRobotStatus(int(robotId), std::string(status));

                return crow::response(200, "Success\n");
            });
        }

        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

        void setIsNext(int robotId, bool status)
        {
            isNext[robotId - 1] = status;
        }

        bool getIsNext(int robotId)
        {
            return isNext[robotId - 1];
        }

        void setIsEnd(int robotId, bool status)
        {
            isEnd[robotId - 1] = status;
        }

        bool getIsEnd(int robotId)
        {
            return isEnd[robotId - 1];
        }

    private:
        crow::Blueprint bp;
        std::vector<bool> isNext;
        std::vector<bool> isEnd;
        std::mutex mtx;
        std::condition_variable cv;
        IpAddress ipAddress;
        
        int availableRobotId()
        {
            auto conn = Connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT * FROM robot_info WHERE robot_status = ? AND robot_battery >= ? ORDER BY robot_battery DESC"));
            pstmt->setString(1, "CHARGING");
            pstmt->setInt(2, 30);
            
            std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());

            int robotId = -1;

            if (res->next())
            {
                robotId = res->getInt("robot_id");
            }
            
            return robotId;
        }

        void setRobotStatus(int robotId, std::string status)
        {
            auto conn = Connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("UPDATE robot_info SET robot_status = ? WHERE robot_id = ?"));
            pstmt->setString(1, status);
            pstmt->setInt(2, robotId);
            pstmt->execute();
            std::cout << "Update status successfully : " << status << std::endl;
        }

        std::string getRobotStatus(int robotId)
        {
            auto conn = Connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT robot_status FROM robot_status WHERE robot_id = ?"));
            pstmt->setInt(1, robotId);

            std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());
            res->next();

            std::string robotStatus = res->getString("robot_status");
            
            return robotStatus;
        }

        std::vector<int> getRobotId(std::string robotStatus)
        {
            auto conn = Connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT robot_id FROM robot_status WHERE robot_status = ?"));
            pstmt->setString(1, robotStatus);

            std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());

            std::vector<int> robotId;

            while(res->next())
            {
                robotId.push_back(res->getInt("robot_id"));
            }
            
            return robotId;
        }

        void assignTasksToRobot(int robotId, std::queue<std::string> tasks)
        {
            std::cout << "Creating thread for robot " << robotId << std::endl;
            std::thread([this, robotId, tasks = std::move(tasks)]() mutable
            {

                while (!tasks.empty())
                {
                    if (getIsNext(robotId))
                    {
                        std::string task = tasks.front();
                        std::cout << "Current task: " << task << std::endl;
                        tasks.pop();
                        std::cout << "Task pop Successfully, task size: " << tasks.size() << std::endl;
                        sendTask(robotId, task);
                        setRobotStatus(robotId, "MOVETOSECTION");
                        setIsNext(robotId, false);
                        std::cout << "robotId[" << robotId << "]: " << getIsNext(robotId) << std::endl;
                    }
                }

                {
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [this, robotId] { return getIsNext(robotId); });
                }

                if (tasks.empty())
                {
                    sendTask(robotId, "포장소");
                    setRobotStatus(robotId, "MOVETOPACKING");
                }

                {
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [this, robotId] { return getIsEnd(robotId); });
                }

                sendTask(robotId, "충전소");
                setRobotStatus(robotId, "MOVETOSTATION");
                std::cout << "Thread ending for robot " << robotId << std::endl;
            }).detach();
        }

        void sendTask(int robotId, std::string task)
        {
            json data;
            data["robot_id"] = std::to_string(robotId);
            data["task"] = task;
            std::string jsonData = data.dump();

            std::string ip = ipAddress.getLocalIP();
            std::string url = "http://" + ip + ":5000/destination/send";

            CURL* curl = curl_easy_init();
            if (!curl) { return; }

            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, jsonData.size());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK)
            {
                std::cout << "Curl failed: " << curl_easy_strerror(res) << std::endl;
            }
            else
            {
                std::cout << "Curl succeeded" << std::endl;
            }

            curl_easy_cleanup(curl);
            curl_slist_free_all(headers);
        }
};