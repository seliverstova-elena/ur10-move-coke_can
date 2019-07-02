#ifndef _GAZEBO_MODEL_COLLISION_PLUGIN_HH_
#define _GAZEBO_MODEL_COLLISION_PLUGIN_HH_

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ur10/model_collision_bounding_box.h>

namespace gazebo
{
  // Плагин для получения габаритов и положения центров ограничивающих коллизиционных параллелепипедов
  // типа AABB моделей, находящихся в загруженном в Gezebo мире
  class ModelCollisionPlugin : public WorldPlugin
  {
    public: 
        // Конструктор
        ModelCollisionPlugin() : WorldPlugin() {};
        // Метод загрузки плагина
        // [in] _world  Указатель на мир, загруженный в Gezebo
        // [in] _sdf    Элемент SDF, который описывает плагин
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    
    private: 
        // Указатель на мир, загруженный в Gezebo
        physics::WorldPtr world;
        // Сервис, возвращающий габариты и положение центра ограничивающего параллелепипеда модели
        ros::ServiceServer model_collision_srv;
        // Метод, вызываемый при вызове сервиса model_collision_srv
        bool getModelCollisionBoundingBox(ur10::model_collision_bounding_box::Request& req, ur10::model_collision_bounding_box::Response& res); 
        // Метод проверки существования модели с именем model_name в загруженном в Gezebo мире
        // [out] true, если модель с именем model_name существует, и false, если нет
        // [in]  model_name Имя модели
        bool checkExistence(std::string model_name);
        // Метод, возвращаюй список имен моделей, находящихся в загруженном в Gezebo мире
        // [out] model_names Список имен моделей, находящихся в загруженном в Gezebo мире
        std::vector<std::string> getExistingModels();
  };
}

#endif // _GAZEBO_MODEL_COLLISION_PLUGIN_HH_