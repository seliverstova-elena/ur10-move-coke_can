#ifndef COLLISION_H
#define COLLISION_H

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Класс добавления ограничивающих коллизиционных параллелепипедов типа AABB
// на коллизицинную сцену MoveIt и их удаления 
class Collision 
{
public:
    // Конструктор
    // [in] nh              Ссылка на ноду
    // [in] frame_attach_   Имя системы координат, относительно которой задаются положения коллизиционных объектов
    Collision(ros::NodeHandle& nh, std::string frame_attach_);
    // Метод добаления объекта на коллизицинную сцену MoveIt
    // [in] id_         Идентификатор коллизиционного объекта
    // [in] pose_       Положение коллизиционного объекта
    // [in] dimensions_ Габариты коллизиционного объекта
    void addCollisionObject(std::string id_, geometry_msgs::Pose pose_, geometry_msgs::Vector3 dimensions_);
    // Метод удаления объекта с коллизицинной сцены MoveIt
    // [in] id_ Идентификатор коллизиционного объекта
    void removeCollisionObject(std::string id_);
    
private:
    // Указатель на объект, позволяющий получить доступ к коллизицинной сцене MoveIt
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
    // Паблишер, публикующий сообщение добавления или удалания коллизиционного объекта
    ros::Publisher planning_scene_diff_publisher;
    // Сообщение, которое публикует planning_scene_diff_publisher 
    moveit_msgs::PlanningScene planning_scene;
    // Имя системы координат, относительно которой задаются положения коллизиционных объектов
    std::string frame_attach;
    // Идентификатор коллизиционного объекта
    std::string id;
    // Положение коллизиционного объекта
    geometry_msgs::Pose pose;
    // Габариты коллизиционного объекта
    geometry_msgs::Vector3 dimensions;
    // Идентификатор выполняемой операции
    // true, если объект удаляется с колизициооной сцены, и false, если добавляется
    bool remove_object;    
    // Метод изменения коллизиционной сцены
    void changeCollisionScene();
    // Метод создания коллизиционного объекта
    // [out] collision_object Коллизиционный объект
    moveit_msgs::CollisionObject createCollisionObject();
    // Метод создания примитива типа "параллелепипед"
    // [out] primitive Примитив типа "параллелепипед" 
    shape_msgs::SolidPrimitive createPrimitive();
    // Метод ожидания добавления объекта на коллизиционную сцену MoveIt
    void waitForCollisionObjectAdded();
    // Метод ожидания удаляния объекта с коллизиционной сцены MoveIt
    void waitForCollisionObjectRemoved();
};

#endif // COLLISION_H