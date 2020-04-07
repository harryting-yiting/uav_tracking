#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

std::string iris_name = "ar_model";

// callback gazeo pose -> transform to geo_msg transform -> boradcast
void ModelPoseCB(const gazebo_msgs::ModelStates& new_states)
{
    // debug
    std::cout << "CB" << std::endl;
    geometry_msgs::Pose md_pose;

    // find model state 
    ros::V_string::const_iterator it=std::find(new_states.name.begin(), new_states.name.end(), iris_name);
    int index = std::distance(new_states.name.begin(), it);
    //std::cout << "index:" << index << std::endl;
    md_pose = new_states.pose[index];

    // convert pose to transform
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = iris_name;
    transform_stamped.transform.translation.x = md_pose.position.x;
    transform_stamped.transform.translation.y = md_pose.position.y;
    transform_stamped.transform.translation.z = md_pose.position.z;

    transform_stamped.transform.rotation = md_pose.orientation;
    // broadcast transform

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transform_stamped);

}

int main(int argc, char **argv)
{
    // ros setup
    ros::init(argc, argv, "tag_tf_broadcaster");
    ros::NodeHandle nh("~");
    // sub
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states",1, &ModelPoseCB);
    // spin
    ros::spin();
    return 0;
}   

