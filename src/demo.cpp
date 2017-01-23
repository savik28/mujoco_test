#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "iostream"
#include "ros/ros.h"
//#include "geometry_msgs/WrenchStamped.h"
//#include "tf2_msgs/TFMessage.h"
//#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/JointState.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>

using namespace std;

mjModel* m;
mjData* d;

KDL::Chain my_chain;
KDL::Tree my_tree;

unsigned int mujoco_init()
{
    char error[1000];

    mj_activate("/home/student/mjpro140/bin/mjkey.txt");

    //std::string file_content = read_file("obj.xml");
    //m = mj_loadXML(NULL, file_content, error, 1000);

    m = mj_loadXML("/home/student/mjpro140/model/ur5/ur5.xml", NULL, error, 1000);
    cout<<"success"<<endl;

    if( !m )
    {
       printf("%s\n", error);
       return 1;
    }

    d = mj_makeData(m);

    // setting the set points for Joint angles in radians
    double start_pose[8]    = {0,0,0,0,0,0,0,0};//{0.1,-1,1,-2.5,-1,0,-0.05,0.05};{0,0,0,0,0,0,0,0}
    double pos_set_point[8] = {-1.57,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};//{0.3,-1.45,1.45,-3.14,-1.46,0.9,-0.04,0.04};
    double vel_set_point[8] = {0,0,0,0,0,0,0,0};
    for(int c=0; c < m->nv; c++)
    {
        // first 8 in d->ctrl is used as motor actuator (online gravity compensation)
        d->ctrl[c+8] = pos_set_point[c];  // next 8 in d->ctrl is used as position actuator
        d->ctrl[c+16] = vel_set_point[c]; // next 8 in d->ctrl is used as velocity actuator
        d->qpos[c] = start_pose[c];
    }

    return sizeof(*d);

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo");
    ros::NodeHandle node_obj;

    ros::Publisher js_publisher = node_obj.advertise<sensor_msgs::JointState>("/joint_states",1000);

    ros::Duration(0.1).sleep();
    //ros::Rate loop_rate(10);

    unsigned int mi;

    mi = mujoco_init();  //load

    if ( mi==1 )
        cout<<"Error in Initialisation"<<endl;
    else
       cout<<"Data Block of "<<mi <<" Bytes is reserved for Simulation" <<endl;


    //setting up header joint state msgs
    sensor_msgs::JointState js_msg;
    js_msg.name.resize(8);
    js_msg.position.resize(8);
    js_msg.velocity.resize(8);
    js_msg.name[0] = "shoulder_pan_joint";
    js_msg.name[1] = "shoulder_lift_joint";
    js_msg.name[2] = "elbow_joint";
    js_msg.name[3] = "wrist_1_joint";
    js_msg.name[4] = "wrist_2_joint";
    js_msg.name[5] = "wrist_3_joint";
    js_msg.name[6] = "gripper_base_left_finger_joint";
    js_msg.name[7] = "gripper_base_right_finger_joint";


  // while
    if ( ros::ok() )
    {
        while( d->time < 20 )
        {
            for(int e=0; e<m->nv; e++)
            {
              d->ctrl[e] = d->qfrc_bias[e];
            }

            mj_step(m,d); //simulation

            ros::Time now = ros::Time::now();

            js_msg.header.stamp = now;
            for(int i=0; i < m->nv; i++)
            {
               js_msg.position[i] = d->qpos[i];
               js_msg.velocity[i] = d->qvel[i];
            }

            //js_msg.effort = [];

            js_publisher.publish(js_msg);

            ros::spinOnce();
            ros::Duration(0.01).sleep();
            //loop_rate.sleep();
        }

        ROS_INFO("All Messages are Published");
        ROS_INFO("Simulation done");

        for (int z=0; z < m->nv; z++)
        {
            cout <<"Joint-"<< z << endl
                 <<"Goal::Cu.State::Error => ";
            cout << d->ctrl[z+8] <<"::"
                 << d->qpos[z] <<"::"
                 << (d->ctrl[z+8] - d->qpos[z])<< endl<< endl;
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 0;
}

