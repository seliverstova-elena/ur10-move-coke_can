#include <ur10/operations_manager.h>

OperationsManager::OperationsManager(ros::NodeHandle& nh, std::string group_name_, std::string robot_name_)
{
    motion_control = new MotionControl(nh, group_name_);
    
    scene = new Scene(nh, robot_name_);
    
    std::string robot_root_frame = motion_control->getPlanningFrame();
    collision = new Collision(nh, robot_root_frame);
    
    geometry_msgs::Pose robot_pose_in_world_frame = scene->getModelPose(robot_name_); 
    transform = new Transform(robot_pose_in_world_frame);
}

void OperationsManager::addAndMoveModel(std::string model_name_)
{
    model_name = model_name_;
    // Добавление объекта в мир, загруженный в Gazebo
    addModelToScene();
    // Добавление земли в коллизиционную сцену MoveIt
    addObjectToCollision("ground");
    // Добавление ступеньки в коллизиционную сцену MoveIt
    addObjectToCollision("stair");
    // Добавление объекта в коллизиционную сцену MoveIt
    addObjectToCollision(model_name);
    // Вывод манипулятора в положение перед началом перемещение объекта
    goToStartPose();
    // Удаление объекта с коллизиционной сцены MoveIt
    collision->removeCollisionObject(model_name);
    // Прижатие объекта к ступеньке манипулятором
    moveModel();
}

void OperationsManager::addModelToScene()
{
    std::string model_path = getModelPath();
    geometry_msgs::Pose model_pose;
    model_pose.position.x = 0.75;
    model_pose.position.y = 0.35;
    model_pose.position.z = 0;
    model_pose.orientation.w = 1;
    
    scene->addModel(model_name, model_path, model_pose);
}

std::string OperationsManager::getModelPath()
{
    passwd* pw = getpwuid(getuid());
    std::string model_path(pw->pw_dir);
    model_path += "/.gazebo/models/" + model_name + "/model.sdf";
    return model_path;
}

void OperationsManager::addObjectToCollision(std::string obj_name)
{
    geometry_msgs::Point obj_center;
    geometry_msgs::Vector3 obj_dimensions;    
    if(obj_name == "ground")
    {
        obj_dimensions.x = 2.5;
        obj_dimensions.y = 2.5;
    }
    else
        scene->getModelAABBCenterAndDimensions(obj_name, obj_center, obj_dimensions);
    
    geometry_msgs::Pose obj_pose_in_world_frame;
    obj_pose_in_world_frame.position = obj_center;
    obj_pose_in_world_frame.orientation.w = 1.0;
    geometry_msgs::Pose obj_pose_in_robot_frame = transform->getPoseInRobotFrame(obj_pose_in_world_frame);
    collision->addCollisionObject(obj_name, obj_pose_in_robot_frame, obj_dimensions);
}

void OperationsManager::goToStartPose()
{
    geometry_msgs::Point model_center; 
    geometry_msgs::Vector3 model_dimensions;
    scene->getModelAABBCenterAndDimensions(model_name, model_center, model_dimensions);
    
    geometry_msgs::Pose target_robot_pose_in_world_frame;
    target_robot_pose_in_world_frame.position = model_center;
    // Отступаем от коллизиционной модели объекта на 0,01 м
    target_robot_pose_in_world_frame.position.y += model_dimensions.y / 2.0 + 0.01;
    target_robot_pose_in_world_frame.orientation.w = 1.0;
    
    // Ориентируем тул манипулятора по направлению к объекту
    Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
    float angle = -M_PI_2;
    target_robot_pose_in_world_frame.orientation = transform->turnOrientation(target_robot_pose_in_world_frame.orientation, axis, angle);
    
    geometry_msgs::Pose target_robot_pose_in_robot_frame = transform->getPoseInRobotFrame(target_robot_pose_in_world_frame);
    motion_control->moveToGoalPose(target_robot_pose_in_robot_frame);
}

void OperationsManager::moveModel()
{
    Eigen::Vector3f direction_in_world_frame = -Eigen::Vector3f::UnitY();
    geometry_msgs::Vector3 direction_in_robot_frame = transform->getVectorInRobotFrame(direction_in_world_frame);
    motion_control->moveAlongVector(direction_in_robot_frame);
}