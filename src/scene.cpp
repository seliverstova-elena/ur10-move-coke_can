#include <ur10/scene.h>

Scene::Scene(ros::NodeHandle& nh, std::string robot_name)
{
    ros::service::waitForService("gazebo/spawn_sdf_model");
    spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    
    ros::service::waitForService("gazebo/get_model_state");
    model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    ros::service::waitForService("gazebo_plagin/get_model_collision_bounding_box");
    model_collision_bounding_box_client = nh.serviceClient<ur10::model_collision_bounding_box>("/gazebo_plagin/get_model_collision_bounding_box");
}

void Scene::addModel(std::string model_name, std::string model_path, geometry_msgs::Pose model_pose)
{
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = model_name;
    srv.request.model_xml = createModel(model_path);
    srv.request.initial_pose = model_pose;

    spawn_model_client.call(srv); 
    ros::Duration(1.0).sleep();
}

std::string Scene::createModel(std::string model_path)
{
    std::string model, buf;
    std::ifstream fin(model_path);
    while(!fin.eof())
    {
        getline(fin, buf);
        model += buf + "\n";
    }
    return model;
}


geometry_msgs::Pose Scene::getModelPose(std::string model_name)
{
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;
    model_state_client.call(srv);
    
    geometry_msgs::Pose model_pose = srv.response.pose;
    return model_pose;
}

void Scene::getModelAABBCenterAndDimensions(std::string model_name, geometry_msgs::Point& center, geometry_msgs::Vector3& dimensions)
{
    ur10::model_collision_bounding_box srv; 
    srv.request.model_name.data = model_name;
    model_collision_bounding_box_client.call(srv);
    
    center = srv.response.center;    
    dimensions = srv.response.dimensions;
}
