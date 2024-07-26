#pragma once

#include <mysql_driver.h>
#include <mysql_connection.h>

#include "yaml_reader.hpp"

class DatabaseConnection : public YamlReader
{
    public:
        DatabaseConnection(const std::string& configFile) : YamlReader(configFile)
        {
            driver = sql::mysql::get_mysql_driver_instance();
        }

        std::unique_ptr<sql::Connection> Connection()
        {

            auto conn = std::unique_ptr<sql::Connection>(driver->connect("tcp://" + host + ":" + port, username, password));
            conn->setSchema(name);

            return conn;
        }

    private:
        sql::mysql::MySQL_Driver* driver;
};