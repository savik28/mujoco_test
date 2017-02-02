#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  int counter = 0;
  while(ros::ok())
  {
    while(counter < 100)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();

      marker.ns = "free_objects";
      marker.id = counter;

       marker.type = visualization_msgs::Marker::CUBE;
       marker.action = visualization_msgs::Marker::ADD;


         marker.pose.position.x = -0.5+0.01;
         marker.pose.position.y = 0.15+0.01;
         marker.pose.position.z = 0.2;


       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = 1.0;

       marker.scale.x = 0.1;
       marker.scale.y = 0.1;
       marker.scale.z = 0.4;

       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 0.0;
       marker.color.g = 0.9;
       marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
       //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
       marker_pub.publish( marker );
        ros::spinOnce();
       r.sleep();
       counter++;
     }
  }


}
