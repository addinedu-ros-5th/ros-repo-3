#include <crow.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/prepared_statement.h>
#include "../connection/database_connection.h"

class DatabaseManager : public DatabaseConnector 
{
public:
    DatabaseManager()
    : bp("data")
    {
        CROW_BP_ROUTE(bp, "/customer_order").methods("POST"_method)([this](const crow::request& req)
        {
            auto data = crow::json::load(req.body);
            if (!data) { return crow::response(400, "Invalid JSON"); }
            //std::cout << data << std::endl;

            auto conn = connection();
            
            if (data.has("items"))
            {
                auto itemsArray = data["items"];
                for (const auto& item : itemsArray)
                {
                    //std::cout << item << std::endl;
                    std::string productName = item["productName"].s();
                    int productAmount = item["productAmount"].i();

                    std::unique_ptr<sql::PreparedStatement> pstmt(conn->prepareStatement("INSERT INTO customer_order (productName, productAmount) VALUES (?, ?)"));
                    pstmt->setString(1, productName);
                    pstmt->setInt(2, productAmount);
                    pstmt->execute();
                }
            }

            return crow::response(200, "Data upload successfully");
        });
    }

    crow::Blueprint& getBlueprint()
    {
        return bp;
    }

private:
    crow::Blueprint bp;
};
