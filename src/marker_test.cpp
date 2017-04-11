#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  int counter = 0;

  visualization_msgs::MarkerArray mark;
  mark.markers.resize(2);

  for(int i=1; i<=2; i++)
  {
    mark.markers[i].header.frame_id = "world";
    mark.markers[i].header.stamp = ros::Time();
    mark.markers[i].ns = "free_objects";
    mark.markers[i].id = i;

    mark.markers[i].type = visualization_msgs::Marker::CUBE;
    mark.markers[i].action = visualization_msgs::Marker::ADD;

    mark.markers[i].scale.x = 0.1;
    mark.markers[i].scale.y = 0.1;
    mark.markers[i].scale.z = 0.4;

    mark.markers[i].color.a = 1.0; // Don't forget to set the alpha!
    mark.markers[i].color.r = 0.0;
    mark.markers[i].color.g = 0.9;
    mark.markers[i].color.b = 0.0;
  }

  while(ros::ok())
  {
    for(int i=1; i<=2; i++)
    {
       mark.markers[i].pose.position.x = -0.5+(i*0.1)+counter*0.01;
       mark.markers[i].pose.position.y = 0.15+counter*0.01;
       mark.markers[i].pose.position.z = 0.2;

       mark.markers[i].pose.orientation.x = 0.0;
       mark.markers[i].pose.orientation.y = 0.0;
       mark.markers[i].pose.orientation.z = 0.0;
       mark.markers[i].pose.orientation.w = 1.0;
    }

     //only if using a MESH_RESOURCE marker type:
     //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
     //mark.markers.push_back(mark);
     marker_pub.publish(mark);
     ros::spinOnce();
     r.sleep();
     counter++;
  }
}
