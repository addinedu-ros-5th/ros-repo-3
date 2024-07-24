#pragma once

#include <crow.h>
#include <memory.h>
#include <curl/curl.h>
#include <string>

class LedControl
{
    public:
        LedControl() : bp("led")
        {
            CROW_BP_ROUTE(bp, "/status").methods(crow::HTTPMethod::POST)([this](const crow::request& req)
            {
                auto data = crow::json::load(req.body);
                if (!data) { return crow::response(400, "Invalid JSON"); }

                auto status = data["status"];
                if (status == "Packing")
                {
                    request("green");
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

        void request(const std::string& color)
        {
            std::string url = "http://192.168.0.12:80/led_control?color=" + color;

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