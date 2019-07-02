#include <plugins/model_collision_plugin.hh>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(ModelCollisionPlugin)

void ModelCollisionPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{       
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_plagin");
    }        
    ros::NodeHandle* rosNode = new ros::NodeHandle("gazebo_plagin");
    model_collision_srv = rosNode->advertiseService("get_model_collision_bounding_box", &ModelCollisionPlugin::getModelCollisionBoundingBox, this);
    
    world = _world;
} 

bool ModelCollisionPlugin::getModelCollisionBoundingBox(ur10::model_collision_bounding_box::Request& req, ur10::model_collision_bounding_box::Response& res)
{
    std::string model_name = req.model_name.data.c_str();
    if(checkExistence(model_name))
    {
        physics::ModelPtr model = this->world->GetModel(model_name);
        math::Box bounding_box = model->GetCollisionBoundingBox();

        res.dimensions.x = bounding_box.GetSize().x;
        res.dimensions.y = bounding_box.GetSize().y;
        res.dimensions.z = bounding_box.GetSize().z;
        
        res.center.x = bounding_box.GetCenter().x;
        res.center.y = bounding_box.GetCenter().y;
        res.center.z = bounding_box.GetCenter().z;
        
        return true;
    }
    return false; 
}

bool ModelCollisionPlugin::checkExistence(std::string model_name)
{
    std::vector<std::string> model_names = getExistingModels();
    if (std::find(model_names.begin(), model_names.end(), model_name) != model_names.end())
        return true;
    ROS_ERROR("getModelCollisionBoundingBox [%s] does not exist", model_name.c_str());
    return false;
}

std::vector<std::string> ModelCollisionPlugin::getExistingModels()
{
    std::vector<std::string> model_names;
    physics::Model_V models = this->world->GetModels();
    for(auto i = 0; i < models.size(); i++)
        model_names.push_back(models[i]->GetName());
    return model_names;
}