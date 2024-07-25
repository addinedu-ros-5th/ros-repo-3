#include <queue>
#include <crow.h>
#include <string>
#include <memory.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <nlohmann/json.hpp>

#include "robot_status.hpp"
#include "database_connection.hpp"

class TaskPlanner : public DatabaseConnection
{
    public:
        TaskPlanner(const std::string configFile) : DatabaseConnection(configFile) , bp("task")
        {
            CROW_BP_ROUTE(bp, "/manage").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) {return crow::response(400, "Invalid JSON"); }

                std::cout << "json data: " << data << std::endl;

                auto items = data["items"];

                for (const auto& item : items)
                {
                    task.push(std::string(item));
                }

                assignTask();

                return crow::response(200, "success");
            });

            CROW_BP_ROUTE(bp, "task_process").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = req.url_params.get("finish");

                return crow::response(200, "success");
            });
        }

        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
        std::queue<std::string> task;

        void assignTask()
        {
            auto conn = Connection();

            std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("SELECT robot_id FROM robot_status WHERE robot_status = ? ORDER BY robot_id"));
            pstmt->setString(1, "CHARGING");
            
            std::unique_ptr<sql::ResultSet> res(pstmt->executeQuery());

            if (!res->next())
            {
                // TODO
                // 모든 로봇이 배정된 경우 작업자 GUI에 모든 로봇이 배정됐다는 문구 전달
            }

            int64_t robotId = res->getInt64(1);
            while(!task.empty())
            {
                
            }
            conn->close();
        }

        void sendTask(int64_t robotId, std::string& task)
        {
            std::string url = "http://192.168.0.79/destination/send?robot_id=" + std::to_string(robotId) + "&task=" + task;

            CURL* curl = curl_easy_init();
            if (!curl) { return; }

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK)
            {
                std::cout << "Curl failed" << std::endl;
            }

            curl_easy_cleanup(curl);
        }
};