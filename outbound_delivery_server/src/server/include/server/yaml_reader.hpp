#pragma once

#include <yaml-cpp/yaml.h>

class YamlReader
{
    public:
        YamlReader(const std::string& configFile)
        {
            config = YAML::LoadFile(configFile);
            host = config["database"]["host"].as<std::string>();
            port = config["database"]["port"].as<std::string>();
            username = config["database"]["username"].as<std::string>();
            password = config["database"]["password"].as<std::string>();
            name = config["database"]["name"].as<std::string>();
        }
    protected:
        YAML::Node config;
        std::string host;
        std::string port;
        std::string username;
        std::string password;
        std::string name;
};