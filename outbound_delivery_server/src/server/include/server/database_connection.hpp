#pragma once

#include <mysql_driver.h>
#include <mysql_connection.h>

#include "yaml_reader.hpp"

class DatabaseConnection : public YamlReader
{
    public:
        DatabaseConnection(const std::string& configFile) : YamlReader(configFile), conn(nullptr)
        {
            driver = sql::mysql::get_mysql_driver_instance();
        }

        ~DatabaseConnection()
        {
            if (conn) 
            {
                conn->close();
                delete conn; 
            }
        }

        sql::Connection* Connection()
        {

            conn = driver->connect("tcp://" + host + ":" + port, username, password);
            conn->setSchema(name);

            return conn;
        }

    private:
        sql::mysql::MySQL_Driver* driver;
        sql::Connection* conn;
};