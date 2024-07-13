#pragma once

#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class DatabaseConnector
{
public:
    DatabaseConnector() : driver(nullptr), conn(nullptr)
    {
        driver = sql::mysql::get_mysql_driver_instance();
    }

    ~DatabaseConnector()
    {
        if (conn) { conn->close(); }
    }

    sql::Connection* connection()
    {
        std::fstream config_file("connection/config.json");
        json config;
        config_file >> config;


        std::string host = config["host"];
        std::string user = config["user"];
        std::string password = config["password"];
        std::string name = config["name"];

        conn.reset(driver->connect(host, user, password));
        conn->setSchema(name);

        return conn.get();
    }

protected:
    sql::mysql::MySQL_Driver* driver;
    std::unique_ptr<sql::Connection> conn;
};