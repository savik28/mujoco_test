#include "ros/ros.h"
#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "iostream"
#include "fstream"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
//#include "geometry_msgs/WrenchStamped.h"
//#include "tf2_msgs/TFMessage.h"
//#include "std_msgs/ColorRGBA.h"
//#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chain.hpp>
//#include <kdl/jntarray.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/chaindynparam.hpp>

using namespace std;

mjModel* m;
mjData* d;

ofstream simulation_data;

int objects_in_scene = 1; //No. of Objects [with FREE JOINT not FIXED to the PLANE] in ur5.xml

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
       simulation_data << error <<endl;
       return 1;
    }
    else
    {
       /*simulation_data <<"Model loaded, parsed & converted sucessfully\n"
                       <<endl<<"MODEL_PARAMETERS"<< endl
                       <<"Gen.Coordinates :"<< m->nq <<endl
                       <<"DOF's :"<< m->nv <<endl
                       <<"Bodys :"<< m->nbody <<endl
                       <<"Joints:"<< m->njnt <<endl
                       <<"Ctrl.IP:"<< m->nu <<endl
                       <<"No.of Free Objects:"<< objects_in_scene <<endl<<endl; */

       cout<<endl<<"MODEL_PARAMETERS"<< endl
                 <<"Gen.Coordinates :"<< m->nq <<endl
                 <<"DOF's :"<< m->nv <<endl
                 <<"Bodys :"<< m->nbody <<endl
                 <<"Joints:"<< m->njnt <<endl
                 <<"Ctrl.IP:"<< m->nu <<endl
                 <<"No.of Free Objects:"<< objects_in_scene <<endl<<endl;

     }

    d = mj_makeData(m);

    // setting the set points for Joint angles in radians
    double start_pose[8]    = {0.5,-0.1,0.2,-2.5,-1,0,-0.05,0.05};//{0.1,-1,1,-2.5,-1,0,-0.05,0.05};{0,0,0,0,0,0,0,0}
    double pos_set_point[8] = {-1,-0.8,0.9,-3.14,-1.57,0.8,-0.025,0.025};//{0.3,-1.45,1.45,-3.14,-1.46,0.9,-0.04,0.04}{-1.57,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};
    double vel_set_point[8] = {0,0,0,0,0,0,0,0};

    for(int c=0; c < m->njnt-objects_in_scene; c++)
    {
        // first 8 in d->ctrl is used as motor actuator (online gravity compensation)
        d->ctrl[c+8] = pos_set_point[c];  // next 8 in d->ctrl is used as position actuator
        d->ctrl[c+16] = vel_set_point[c]; // next 8 in d->ctrl is used as velocity actuator
        d->qpos[c+(objects_in_scene*7)] = start_pose[c];
    }

    return sizeof(*d);

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo");
    ros::NodeHandle node_obj;

    simulation_data.open("contact.txt");
    if(simulation_data.is_open())
        cout << "File is open";
    else
        cout<< "Unable to open a file";

    unsigned int mi, steps = 0;

    mi = mujoco_init();  //load

    if( mi==1 )
       cout<<"Error in Initialisation"<<endl;
    else
       cout<<"Data Block of "<<mi <<" Bytes is reserved for Simulation" <<endl;


    ros::Publisher js_publisher = node_obj.advertise<sensor_msgs::JointState>("/joint_states",1000);

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

    ros::Publisher box_publisher = node_obj.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

    visualization_msgs::Marker box;
    box.header.frame_id = "/world";
    box.ns = "free_objects";
    box.type = visualization_msgs::Marker::CUBE;
    box.action = visualization_msgs::Marker::ADD;
    // id: world=0; if there are 2 objects in scene object1_id=1;object2_id=2;then robots's base_link id = 3
    for(int i=1; i <= objects_in_scene; i++)
    {
      box.id = i; //geom_id/body_id
      box.scale.x = m->geom_size[(i*3)+0]*2;//0.1;
      box.scale.y = m->geom_size[(i*3)+1]*2;//0.1;
      box.scale.z = m->geom_size[(i*3)+2]*2;//0.4;

      box.color.r = m->geom_rgba[(i*4)+0];
      box.color.g = m->geom_rgba[(i*4)+1];
      box.color.b = m->geom_rgba[(i*4)+2];
      box.color.a = m->geom_rgba[(i*4)+3]; // Don't forget to set the alpha!
    }

    ros::Duration(0.1).sleep();
    //ros::Rate loop_rate(10);


  // while
    if ( ros::ok() )
    {
        while( d->time < 20 )
        {
            for(int e=0; e< m->njnt-objects_in_scene; e++)
            {
              d->ctrl[e] = d->qfrc_bias[e+(objects_in_scene*6)];// 1 free joint adds 6 DOF's
            }

            mj_step(m,d); //simulation

            steps++;
            simulation_data <<endl <<"Step "<<steps <<": "<<endl;
            for(int cf=0; cf< d->ncon; cf++)
            {
                //mj_contactForce(m, d, cf,con_force);
                simulation_data <<"Contact "<<cf <<": "<<endl
                                <<"    Contact between geoms "<<d->contact[cf].geom1<<" & "<<d->contact[cf].geom2<<endl;
                                //<<"    Force: "<<*con_force<<endl<<endl;
            }

            ros::Time now = ros::Time::now();

            js_msg.header.stamp = now;
            for(int i=0; i < m->njnt-objects_in_scene; i++)
            {
               js_msg.position[i] = d->qpos[i+(objects_in_scene*7)];
               js_msg.velocity[i] = d->qvel[i+(objects_in_scene*6)];
            }
            //js_msg.effort = [];

            box.header.stamp = now;
            for(int i=1; i <= objects_in_scene; i++)
            {
              box.pose.position.x = d->xpos[(i*3)+0];
              box.pose.position.y = d->xpos[(i*3)+1];
              box.pose.position.z = d->xpos[(i*3)+2];
              box.pose.orientation.x = d->xquat[(i*3)+0];
              box.pose.orientation.y = d->xquat[(i*3)+1];
              box.pose.orientation.z = d->xquat[(i*3)+2];
              box.pose.orientation.w = d->xquat[(i*3)+3];

              //cout<<box.pose.position.x<<":"<< box.pose.position.y<<endl;
              //cout<<d->xpos[3]<<":"<<d->xpos[4]<<endl;
            }

            js_publisher.publish(js_msg);
            box_publisher.publish(box);

            ros::spinOnce();
            ros::Duration(0.01).sleep();
            //loop_rate.sleep();
        }

        ROS_INFO("All Messages are Published");
        ROS_INFO("Simulation done");

        for (int z=0; z < m->njnt-objects_in_scene; z++)
        {
            simulation_data << endl<<"Joint-"<< z << endl
                            <<"Goal::Cu.State::Error => ";
            simulation_data << d->ctrl[z+8] <<"::"
                            << d->qpos[z+(objects_in_scene*7)] <<"::" // 1 free joint adds 7 nq's
                            << (d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)])<< endl;
            cout << endl<<"Joint-"<< z << endl
                 <<"Goal::Cu.State::Error => ";
            cout << d->ctrl[z+8] <<"::"
                 << d->qpos[z+(objects_in_scene*7)] <<"::" // 1 free joint adds 7 nq's
                 << (d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)])<< endl;
        }
    }

    simulation_data.close();
    mj_deleteData(d);
    mj_deleteModel(m);  
    mj_deactivate();

    return 0;
}

