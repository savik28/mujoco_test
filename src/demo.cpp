#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/ColorRGBA.h"

using namespace std;

char error[1000];
mjModel* m;
mjData* d;

int main(int argc, char **argv)
{

    ros::init(argc, argv,"demo_publisher");
    ros::NodeHandle node_obj;
    ros::Publisher height_publisher = node_obj.advertise<geometry_msgs::TransformStamped>("/cartesian_pos",100);
    ros::Publisher rgba_publisher = node_obj.advertise<std_msgs::ColorRGBA>("/color",100);
    ros::Publisher force_publisher = node_obj.advertise<geometry_msgs::WrenchStamped>("/force_sensor",100);
    ros::Duration(0.1).sleep();
    //ros::Rate loop_rate(10);

    mj_activate("/home/student/Downloads/mjkey.txt");
    cout<<"success"<<endl;

    m = mj_loadXML("/home/student/mjpro140/model/obj.xml", NULL, error, 1000);

    if( !m )
    {
       printf("%s\n", error);
       return 1;
    }

    std_msgs::ColorRGBA col;

    col.r = m->geom_rgba[4];
    col.g = m->geom_rgba[5];
    col.b = m->geom_rgba[6];
    col.a = m->geom_rgba[7];
    ROS_INFO("%f, %f,%f,%f",col.r,col.g,col.b,col.a);

    d = mj_makeData(m);


    while ( ros::ok() )
    {
        while( d->time<1)
        {
            geometry_msgs::TransformStamped t_msg;
            geometry_msgs::WrenchStamped w_msg;

            mj_step(m, d);

            w_msg.header.stamp = ros::Time(d->time);
            w_msg.header.frame_id = "sensor_body";
            w_msg.wrench.force.x = d->sensordata[0];
            w_msg.wrench.force.y = d->sensordata[1];
            w_msg.wrench.force.z = d->sensordata[2];
            force_publisher.publish(w_msg);

            t_msg.header.stamp = ros::Time(d->time);
            t_msg.header.frame_id = "object";
            //t_msg.header.child_frame_id = "sensor_body";
            t_msg.transform.translation.x = d->xpos[3];
            t_msg.transform.translation.y = d->xpos[4];
            t_msg.transform.translation.z = d->xpos[5];
            height_publisher.publish(t_msg);

            // ROS_INFO("%f,%f",msg.data,force.data);
            ros::spinOnce();
            //loop_rate.sleep();
        }

    }

    return 0;
}
