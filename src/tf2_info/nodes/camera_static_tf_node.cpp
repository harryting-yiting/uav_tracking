#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string camera_name = "fpv_cam";

int main(int argc, char **argv)
{
    //ros init
    ros::init(argc, argv, "camera_static_tf_broadcaster");
    ros::NodeHandle nh("~");
    static tf2_ros::StaticTransformBroadcaster static_braodcaster;

    // transfrom 
    geometry_msgs::TransformStamped camera_trans;
    camera_trans.header.stamp = ros::Time::now();
    camera_trans.header.frame_id = "iris_fpv_cam";
    camera_trans.child_frame_id = camera_name;
    camera_trans.transform.translation.x = 0;
    camera_trans.transform.translation.y = 0;
    camera_trans.transform.translation.z = 0;

    // quaternion
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);
    camera_trans.transform.rotation.x = quat.x();
    camera_trans.transform.rotation.y = quat.y();
    camera_trans.transform.rotation.z = quat.z();
    camera_trans.transform.rotation.w = quat.w();

    // pub static
    static_braodcaster.sendTransform(camera_trans);
    ros::spin();
    return 0;

}