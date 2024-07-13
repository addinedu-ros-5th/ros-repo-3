#pragma once

#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/driver.h>

class DatabaseConnector
{
public:
    DatabaseConnector() : conn(nullptr)
    {
        driver = sql::mysql::get_mysql_driver_instance();
    }

    ~DatabaseConnector()
    {
        delete conn;
    }

    sql::Connection* connection()
    {
        conn = driver->connect(
            "",
            "",
            ""
        );
        return conn;
    }

protected:
    sql::mysql::MySQL_Driver* driver;
    sql::Connection* conn;
};