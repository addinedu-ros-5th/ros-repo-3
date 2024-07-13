#include "crow.h"
#include "router/database_manager.h"

int main()
{
    crow::SimpleApp app;

    DatabaseManager databaseManager;

    app.register_blueprint(databaseManager.getBlueprint());

    app.port(8080).multithreaded().run();
}