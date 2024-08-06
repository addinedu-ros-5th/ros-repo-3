#pragma once

#include <crow.h>
#include <memory.h>
#include <curl/curl.h>
#include <string>

#include "address.hpp"

class LedControl
{
    public:
        LedControl() : bp("led")
        {
            CROW_BP_ROUTE(bp, "/status").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }
                std::cout << data << std::endl;

                auto robotId = data["robot_id"];
                auto status = data["status"];
                if (status == "Packing")
                {
                    request("green");
                    sendDestination(int(robotId), "출고소");
                }
                else
                {
                    request("red");
                }


                return crow::response(200, "Led on/off successfully");
            });
        }

        crow::Blueprint& getBlueprint()
        {
            return bp;
        }

    private:
        crow::Blueprint bp;
        IpAddress ipAddress;

        void request(const std::string& color)
        {
            std::string url = "http://192.168.1.104:80/led_control?color=" + color;

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

        void sendDestination(int robotId, std::string task)
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