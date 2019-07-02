#include <ur10/collision.h>

Collision::Collision(ros::NodeHandle& nh, std::string frame_attach_)
{
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    sleep(2.0); 
    
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);      
    frame_attach = frame_attach_;
}

void Collision::addCollisionObject(std::string id_, geometry_msgs::Pose pose_, geometry_msgs::Vector3 dimensions_)
{
    id = id_;
    pose = pose_;
    dimensions = dimensions_;
    remove_object = false;
    
    changeCollisionScene();
    waitForCollisionObjectAdded();
}

void Collision::changeCollisionScene()
{
    planning_scene.world.collision_objects.push_back(createCollisionObject());
    planning_scene.is_diff = true;    
    planning_scene_diff_publisher.publish(planning_scene);
}

moveit_msgs::CollisionObject Collision::createCollisionObject()
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_attach; 
    collision_object.id = id;
    if(remove_object)
    {
        collision_object.operation = collision_object.REMOVE;
    }
    else
    {
        collision_object.operation = collision_object.ADD;
        collision_object.primitive_poses.push_back(pose);  
        collision_object.primitives.push_back(createPrimitive());
    }
    
    return collision_object;
}

shape_msgs::SolidPrimitive Collision::createPrimitive()
{
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimensions.x;
    primitive.dimensions[1] = dimensions.y;
    primitive.dimensions[2] = dimensions.z;
    
    return primitive;
}

void Collision::waitForCollisionObjectAdded()
{
    bool collision_object_not_added = true;
    while(collision_object_not_added)
    {
        std::vector<std::string> known_collision_objects = planning_scene_interface->getKnownObjectNames();
        if (std::find(known_collision_objects.begin(), known_collision_objects.end(), id) != known_collision_objects.end())
            collision_object_not_added = false;
        sleep(0.5);
    }
    sleep(1.0);
}

void Collision::removeCollisionObject(std::string id_)
{
    id = id_;
    remove_object = true;
    changeCollisionScene();
    waitForCollisionObjectRemoved();
}

void Collision::waitForCollisionObjectRemoved()
{
    bool collision_scene_not_removed = true;
    while(collision_scene_not_removed)
    {
        collision_scene_not_removed = false;
        std::vector<std::string> known_collision_objects = planning_scene_interface->getKnownObjectNames();
        if (std::find(known_collision_objects.begin(), known_collision_objects.end(), id) != known_collision_objects.end())
            collision_scene_not_removed = true;
        sleep(0.5);
    }
    sleep(1.0);
}