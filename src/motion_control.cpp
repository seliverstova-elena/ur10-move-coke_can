#include <ur10/motion_control.h>

MotionControl::MotionControl(ros::NodeHandle& nh, std::string group_name_)
{       
    move_group = new moveit::planning_interface::MoveGroup(group_name_);
    specifyMoveGroup();
    
    follow_trajectory_result_sub = nh.subscribe("/arm_controller/follow_joint_trajectory/result", 1, &MotionControl::followTrajectoryResultCallback, this);
    ft_sensor_sub = nh.subscribe("/wrist_3_joint/ft_sensor", 1, &MotionControl::ftSensorCallback, this);
}

void MotionControl::followTrajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
{
    plan_is_not_executed = false;
}

void MotionControl::ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    if(msg->wrench.force.y < -30)
    {
        plan_is_not_executed = false;
        move_group->stop();
        ros::spinOnce();
    }
}

void MotionControl::specifyMoveGroup()
{
    move_group->allowReplanning(true);
    move_group->setPlanningTime(10.0);
    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setMaxVelocityScalingFactor(0.5);
    move_group->setMaxAccelerationScalingFactor(0.5);
}

void MotionControl::moveToGoalPose(geometry_msgs::Pose target_pose)
{
    move_group->setPoseTarget(target_pose);
    bool plan_is_success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(plan_is_success)
        execute();
}

void MotionControl::execute()
{
    move_group->asyncExecute(plan);
    waitForExecute();
}

void MotionControl::waitForExecute()
{
    plan_is_not_executed = true;
    while(plan_is_not_executed)
        sleep(0.5);
    sleep(1.0);
}

void MotionControl::moveAlongVector(geometry_msgs::Vector3 vector)
{
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    target_pose.position.x += vector.x;
    target_pose.position.y += vector.y;
    target_pose.position.z += vector.z;
    
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    cartesianPaths(waypoints);
}

void MotionControl::cartesianPaths(std::vector<geometry_msgs::Pose> waypoints)
{   
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if(fraction > 0.0)
    {
        plan.trajectory_= trajectory;
        execute();
    }
}

std::string MotionControl::getPlanningFrame()
{
    return move_group->getPlanningFrame().c_str();
}