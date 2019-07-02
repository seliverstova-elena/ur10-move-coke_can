#ifndef OPERATIONS_MANAGER_H
#define OPERATIONS_MANAGER_H

#include <pwd.h>
#include <ur10/motion_control.h>
#include <ur10/collision.h>
#include <ur10/scene.h>
#include <ur10/transform.h>

// Класс, управляющей выполнением операции по прижатию объекта к ступеньке
class OperationsManager 
{
public:
    // Конструктор
    // [in] nh          Ссылка на ноду
    // [in] group_name_ Имя группы движения манипулятора в MoveIt
    // [in] robot_name_ Имя манипулятора в загруженном в Gazebo мире
    OperationsManager(ros::NodeHandle& nh, std::string group_name_, std::string robot_name_);
    // Метод добавления объекта в мир, загруженный в Gazebo, и его прижатия к ступеньке манипулятором
    void addAndMoveModel(std::string model_name_);
    
private:  
    // Имя объекта, с которым выполняется операция
    std::string model_name;
    // Указатель на объект класса планирования и выполнения тракторий манипулятором
    MotionControl* motion_control;
    // Указатель на объект класса взаимодействовия с миром, загруженным в Gazebo
    Scene* scene;
    // Указатель на объект класса взаимодействовия с коллизиционной сценой MoveIt
    Collision* collision;
    // Указатель на объект класса геометрических преобразований
    Transform* transform;
    // Метод добавления объекта в мир, загруженный в Gazebo
    void addModelToScene();
    // Метод, формирующий абсолютный путь к SDF-модели добавляемого объекта, которая расположена в .gazebo
    // [out] model_path Абсолютный путь к SDF-модели добавляемого объекта
    std::string getModelPath();
    // Метод добавления объекта на коллизиционную сцену MoveIt
    // [in] obj_name Имя объекта
    void addObjectToCollision(std::string obj_name);
    // Метод, переводящий манипулятор в положение перед началом перемещения объекта
    void goToStartPose();
    // Метод, прижимающий объект к ступеньке манипулятором
    void moveModel();
};

#endif // OPERATIONS_MANAGER_H