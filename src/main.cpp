#include <ros/ros.h>
#include <ur10/operations_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur10");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Duration(5).sleep();

    std::string group_name = "manipulator";
    std::string robot_name = "robot";
    std::string model_name = "coke_can";
    
    OperationsManager* manager = new OperationsManager(node_handle, group_name, robot_name);
    manager->addAndMoveModel(model_name);
    
    while(ros::ok)
    {}
    
    return 0;
}