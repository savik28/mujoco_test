#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 100);

  while (ros::ok())
  {
      visualization_msgs::Marker mark;

      mark.header.frame_id = "world";
      mark.header.stamp = ros::Time();
      mark.ns = "free_objects";
      mark.id = 1;

      mark.type = visualization_msgs::Marker::CYLINDER;
      mark.action = visualization_msgs::Marker::ADD;

      mark.scale.x = 0.1;
      mark.scale.y = 0.15;
      mark.scale.z = 0.4;

      mark.color.a = 1.0; // Don't forget to set the alpha!
      mark.color.r = 0.0;
      mark.color.g = 0.9;
      mark.color.b = 0.0;


         mark.pose.position.x = -0.5;
         mark.pose.position.y = 0.15;
         mark.pose.position.z = 0.2;

         mark.pose.orientation.x = 0.0;
         mark.pose.orientation.y = 0.0;
         mark.pose.orientation.z = 0.0;
         mark.pose.orientation.w = 1.0;

       //only if using a MESH_RESOURCE marker type:
       //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
       //mark.markers.push_back(mark);
       marker_pub.publish(mark);
       r.sleep();
  }

}
