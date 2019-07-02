#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <moveit/move_group_interface/move_group.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <geometry_msgs/WrenchStamped.h>

// Класс планирования и выполнения тракторий манипулятором
class MotionControl
{
public:
    // Конструктор
    // [in] nh          Ссылка на ноду
    // [in] group_name_ Имя группы движения манипулятора
    MotionControl(ros::NodeHandle& nh, std::string group_name_);
    // Метод, возвращающий имя базового звена манипулятора
    // [out] Имя базового звена манипулятора
    std::string getPlanningFrame();    
    // Метод, перемещающий тул манипулятора в заданныое положение с заданной ориентацией
    // [in] goal_pose Желаемые положение и ориентация тула манипулятора
    void moveToGoalPose(geometry_msgs::Pose goal_pose);
    // Метод, перемещающий тул манипулятора из текущего положения вдоль вектора
    // на расстояние, равное длинне этого вектора. Ориентация тула при этом сохраняется
    // [in] vector Вектор, вдоль которого перемещается манипулятор
    void moveAlongVector(geometry_msgs::Vector3 vector);
    
private:  
    // Группа движения манипулятора
    moveit::planning_interface::MoveGroup* move_group;
    // Траектория перемещения манипулятора
    moveit::planning_interface::MoveGroup::Plan plan;
    // Идентификатор завершения выполнения траектории
    // true, если трактория выполняется, и false, если траектория выполнена
    bool plan_is_not_executed;
    // Подписчик на тему, в которую публикуется результат выполнения траектории
    ros::Subscriber follow_trajectory_result_sub;
    // Подписчик на тему, в которую публикуется информация от силомоментного датчика,
    // расположенного в шарнире wrist_3_joint манипулятора
    ros::Subscriber ft_sensor_sub;   
    // Метод, выполняющийся при получении сообщения подписчиком follow_trajectory_result_sub
    void followTrajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg);
    // Метод, выполняющийся при получении сообщения подписчиком ft_sensor_sub
    void ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    // Метод задания параметров группы движения манипулятора
    void specifyMoveGroup();
    // Метод выполнения спланированной траектории
    void execute();
    // Метод ожидания выполнения спланированной траектории
    void waitForExecute();
    // Метод выполнения движения манипулятора в декартовой системе координат
    void cartesianPaths(std::vector<geometry_msgs::Pose> waypoints);
};

#endif // MOTION_CONTROL_H