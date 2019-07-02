#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/Core>

// Класс геометрических преобразований
class Transform 
{
public:
    // Конструктор
    // [in] robot_pose_in_world_frame Положение и ориентация базового звена манипулятора в мировой системе координат (СК)
    Transform(geometry_msgs::Pose robot_pose_in_world_frame);
    // Метод перевода положения и ориентации СК, заданной в мировой СК, в СК манипулятора
    // [in] pose_in_world_frame     Положение и ориентации СК, заданной в мировой СК
    // [out] pose_in_robot_frame    Положение и ориентации СК, заданной в СК манипулятора
    geometry_msgs::Pose getPoseInRobotFrame(geometry_msgs::Pose pose_in_world_frame);
    // Метод перевода вектора, заданного в мировой СК, в СК манипулятора
    // [in] vec_in_world_frame     Вектор, заданный в мировой СК
    // [out] vec_in_robot_frame    Вектор, заданный в СК манипулятора
    geometry_msgs::Vector3 getVectorInRobotFrame(Eigen::Vector3f vec_in_world_frame);
    // Метод поворота произвольной СК вокруг заданной оси на заданный угол
    // [in] initial_orientation Начальная ориентация СК
    // [in] axis                Ось, вокруг которой осуществляется поворот
    // [in] angle               Угол, на которой осуществляется поворот
    // [out] final_orientation  Итоговая ориентация СК
    geometry_msgs::Quaternion turnOrientation(geometry_msgs::Quaternion initial_orientation, Eigen::Vector3f axis, float angle);

private:
    // Вектор, описывающий смещение СК манипулятора относительно мировой СК, заданный в СК мира
    Eigen::Vector3f l_robot_world_world;
    // Матрица, описывающая поворот СК манипулятора относительно мировой СК
    Eigen::Matrix3f tau_world_robot;
    // Метод, преобразующий geometry_msgs::Point в Eigen::Vector3f
    Eigen::Vector3f getVectorFromPosition(geometry_msgs::Point position);
    // Метод, преобразующий geometry_msgs::Quaternion в Eigen::Matrix3f
    Eigen::Matrix3f getMatrixFromOrientation(geometry_msgs::Quaternion orientation);
    // Метод, преобразующий geometry_msgs::Quaternion в Eigen::Quaternionf
    Eigen::Quaternionf getQuaternionFromOrientation(geometry_msgs::Quaternion orientation);
    // Метод, преобразующий Eigen::Vector3f и Eigen::Matrix3f в geometry_msgs::Pose
    geometry_msgs::Pose getPoseFromVectorAndMatrix(Eigen::Vector3f l, Eigen::Matrix3f tau);
    // Метод, преобразующий Eigen::Vector3f в geometry_msgs::Point   
    geometry_msgs::Point getPositionFromVector(Eigen::Vector3f l);
    // Метод, преобразующий Eigen::Matrix3f в geometry_msgs::Quaternion
    geometry_msgs::Quaternion getOrientationFromMatrix(Eigen::Matrix3f tau);
    // Метод, преобразующий Eigen::Quaternionf в geometry_msgs::Quaternion
    geometry_msgs::Quaternion getOrientationFromQuaternion(Eigen::Quaternionf q);
    // Метод, преобразующий Eigen::Vector3f в geometry_msgs::Vector3
    geometry_msgs::Vector3 getGeomMsgVectorFromEigenVector(Eigen::Vector3f eigen_vec);
};

#endif // TRANSFORM_H