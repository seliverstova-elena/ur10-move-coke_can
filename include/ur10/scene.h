#ifndef SCENE_H
#define SCENE_H

#include <fstream>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <ur10/model_collision_bounding_box.h>

// Класс взаимодействовия с миром, загруженным в Gazebo, позволяющий добавлять новые модели в мир
// и получать информацию о положении и ориентации моделей, а так же о габаритах и положении центров 
// коллизиционных ограничивающих параллелепипедов моделей, находящихся в загруженном в Gezebo мире
class Scene 
{
public:
    // Конструктор
    // [in] nh          Ссылка на ноду
    // [in] robot_name_ Имя манипулятора в загруженном в Gazebo мире
    Scene(ros::NodeHandle& nh, std::string robot_name);
    // Метод добавления модели в мир, загруженном в Gazebo
    // [in] model_name Имя модели
    // [in] model_path Абсолютный путь к SDF-модели
    void addModel(std::string model_name, std::string model_path, geometry_msgs::Pose model_pose);
    // Метод, возвращающий положение и ориентацию модели
    // [in] model_name  Имя модели
    // [out] model_pose Положение и ориентацию модели
    geometry_msgs::Pose getModelPose(std::string model_name);
    // Метод, возвращающий габариты и положение центра коллизиционного ограничивающего параллелепипеда модели
    // [in] model_name   Имя модели
    // [out] center      Положение центра коллизиционного ограничивающего параллелепипеда модели
    // [out] dimensions  Габариты коллизиционного ограничивающего параллелепипеда модели
    void getModelAABBCenterAndDimensions(std::string model_name, geometry_msgs::Point& center, geometry_msgs::Vector3& dimensions);
    
private:
    // Клиент, вызывающий сервис, который добавляет модель в мир, загружнный в Gazebo
    ros::ServiceClient spawn_model_client;
    // Клиент, вызывающий сервис, который возвращает положение и ориентацию модели в мире, загружнном в Gazebo
    ros::ServiceClient model_state_client;
    // Клиент, вызывающий сервис, который возвращает габариты 
    // и положении центра коллизиционного ограничивающего параллелепипеда модели
    ros::ServiceClient model_collision_bounding_box_client;
    // Метод создания модели
    // [in] model_path  Абсолютный путь к SDF-модели
    // [out] model      Модель объекта
    std::string createModel(std::string model_path);
};

#endif // SCENE_H