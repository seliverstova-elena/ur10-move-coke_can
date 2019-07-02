#include <ur10/transform.h>

Transform::Transform(geometry_msgs::Pose robot_pose_in_world_frame)
{
    l_robot_world_world = getVectorFromPosition(robot_pose_in_world_frame.position);
    tau_world_robot = getMatrixFromOrientation(robot_pose_in_world_frame.orientation);
}

Eigen::Vector3f Transform::getVectorFromPosition(geometry_msgs::Point position)
{
    Eigen::Vector3f l;
    l.x() = position.x;
    l.y() = position.y;
    l.z() = position.z;
    return l;
}

Eigen::Matrix3f Transform::getMatrixFromOrientation(geometry_msgs::Quaternion orientation)
{
    Eigen::Quaternionf q = getQuaternionFromOrientation(orientation);    
    Eigen::Matrix3f tau = Eigen::Matrix3f(q);
    return tau;
}

Eigen::Quaternionf Transform::getQuaternionFromOrientation(geometry_msgs::Quaternion orientation)
{
    Eigen::Quaternionf q;
    q.x() = orientation.x;
    q.y() = orientation.y;
    q.z() = orientation.z;
    q.w() = orientation.w;
    return q;
}

geometry_msgs::Pose Transform::getPoseInRobotFrame(geometry_msgs::Pose model_pose_in_world_frame)
{    
    Eigen::Vector3f l_model_world_world = getVectorFromPosition(model_pose_in_world_frame.position);
    Eigen::Matrix3f tau_world_model = getMatrixFromOrientation(model_pose_in_world_frame.orientation);
    
    Eigen::Vector3f l_model_robot_robot = tau_world_robot.transpose() * (l_model_world_world - l_robot_world_world);
    Eigen::Matrix3f tau_robot_model = tau_world_robot.transpose() * tau_world_model;

    geometry_msgs::Pose model_pose_in_robot_frame = getPoseFromVectorAndMatrix(l_model_robot_robot, tau_robot_model);
    return model_pose_in_robot_frame;
}

geometry_msgs::Pose Transform::getPoseFromVectorAndMatrix(Eigen::Vector3f l, Eigen::Matrix3f tau)
{    
    geometry_msgs::Pose pose;
    pose.position = getPositionFromVector(l);
    pose.orientation = getOrientationFromMatrix(tau);
    return pose;
}

geometry_msgs::Point Transform::getPositionFromVector(Eigen::Vector3f l)
{
    geometry_msgs::Point position;
    position.x = l.x();
    position.y = l.y();
    position.z = l.z();  
    return position;
}

geometry_msgs::Quaternion Transform::getOrientationFromMatrix(Eigen::Matrix3f tau)
{
    Eigen::Quaternionf q = Eigen::Quaternionf(tau);    
    geometry_msgs::Quaternion orientation = getOrientationFromQuaternion(q);
    return orientation;
}

geometry_msgs::Quaternion Transform::getOrientationFromQuaternion(Eigen::Quaternionf q)
{
    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
}

geometry_msgs::Vector3 Transform::getVectorInRobotFrame(Eigen::Vector3f vec_in_world_frame)
{
    Eigen::Vector3f vec_in_robot_frame = tau_world_robot.transpose() * vec_in_world_frame;
    return getGeomMsgVectorFromEigenVector(vec_in_robot_frame);
}

geometry_msgs::Vector3 Transform::getGeomMsgVectorFromEigenVector(Eigen::Vector3f eigen_vec)
{
    geometry_msgs::Vector3 geom_msg_vec;
    geom_msg_vec.x = eigen_vec.x();
    geom_msg_vec.y = eigen_vec.y();
    geom_msg_vec.z = eigen_vec.z();
    return geom_msg_vec;
}

geometry_msgs::Quaternion Transform::turnOrientation(geometry_msgs::Quaternion initial_orientation, Eigen::Vector3f axis, float angle)
{
    Eigen::Quaternionf initial_q = getQuaternionFromOrientation(initial_orientation);
    Eigen::AngleAxisf angle_axis = Eigen::AngleAxisf(angle, axis);
    Eigen::Quaternionf rotation_q = Eigen::Quaternionf(angle_axis);
    Eigen::Quaternionf final_q = initial_q * rotation_q;
    geometry_msgs::Quaternion final_orientation = getOrientationFromQuaternion(final_q);
    return final_orientation;
}