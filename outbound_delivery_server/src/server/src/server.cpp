#include "rclcpp/rclcpp.hpp"

#include <crow.h>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "server/order_manager.hpp"
#include "server/destination.hpp"
#include "server/led_control.hpp"
#include "server/camera_detection.hpp"
#include "server/task_planner.hpp"
#include "server/gui_communication.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("server");
    std::string configFile = (std::filesystem::path(package_share_directory) / "config" / "database.yaml").string();
    
    OrderManager orderManager(configFile);
    Destination destination(configFile);
    LedControl ledControl;
    CameraDetection cameraDetection;
    TaskPlanner taskPlanner(configFile);
    GuiCommunication guiCommunication(configFile);

    crow::SimpleApp app;

    app.register_blueprint(orderManager.getBlueprint());
    app.register_blueprint(destination.getBlueprint());
    app.register_blueprint(ledControl.getBlueprint());
    app.register_blueprint(cameraDetection.getBlueprint());
    app.register_blueprint(taskPlanner.getBlueprint());
    app.register_blueprint(guiCommunication.getBlueprint());

    app.port(5000).multithreaded().run();

    rclcpp::shutdown();

    return 0;
}