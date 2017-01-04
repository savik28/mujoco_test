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

    mj_activate("/home/student/Downloads/mjkey.txt");

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
    double set_point[8] = {0,-1.42,1.45,-3.14,-1.46,0,-0.025,0.025};
    for(int c=0; c < m->nu; c++)
    {
      d->ctrl[c] = set_point[c];
    }

    return sizeof(d);

}

unsigned int chain_creator()
{

    if (!kdl_parser::treeFromFile("/home/student/ros_mujoco_ws/src/mujoco_test/urdf/ur5_with_gripper.urdf", my_tree))
    {
         ROS_ERROR("Failed to construct kdl tree");
         return 1;
    }
    unsigned int tj = my_tree.getNrOfJoints();
    unsigned int ts = my_tree.getNrOfSegments();
    cout << endl << "A Tree is created with "<< ts << " segments and " << tj << " joints" << endl;

    bool chain_done;
    chain_done = my_tree.getChain("base_link","ee_link",my_chain);

    unsigned int cj = my_chain.getNrOfJoints();
    unsigned int cs = my_chain.getNrOfSegments();

    if(chain_done)
        cout <<"A Chain is created with " << cs << " segments and " << cj << " joints" << endl;
    else
        cout <<"Chain Creation Failed" << endl;

    return cj;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo");
    ros::NodeHandle node_obj;

    ros::Publisher js_publisher = node_obj.advertise<sensor_msgs::JointState>("/joint_states",1000);

    ros::Duration(0.1).sleep();
    //ros::Rate loop_rate(10);

    unsigned int mi, cjn, gb;

    mi = mujoco_init();  //load

    if ( mi==1 )
        cout<<"Error in Initialisation"<<endl;
    else
       cout<<"Data Block of "<<mi <<"Bytes is reserved for Simulation" <<endl;


    cjn = chain_creator(); // load

    KDL::Vector my_gravity(0.0, 0.0,-9.81);
    KDL::JntArray jnt_q(cjn),gravity_mat(cjn);    //jnt_qd(cjn),coriolis_mat(cjn);
    //KDL::JntSpaceInertiaMatrix inertia_mat(cjn);

    KDL::ChainDynParam forces_calc = KDL::ChainDynParam( my_chain, my_gravity );

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
            for(int j=0; j < m->nu; j++)
            {
                 jnt_q(j) = d->qpos[j];
            }

            gb = forces_calc.JntToGravity(jnt_q, gravity_mat); //gb=0 calculation is done

            if (gb >= 0)
            {
                for (unsigned int k=0; k < cjn; k++)
                {
                    d->qfrc_applied[k] = gravity_mat(k);
                }

            }


            mj_step(m,d); //simulation


            ros::Time now = ros::Time::now();

            js_msg.header.stamp = now;
            for(int i=0; i < m->nu; i++)
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

        ROS_INFO("Simulation done");
        ROS_INFO("All Messages are Published");

    }

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 0;
}

/*
    ros::Publisher force_publisher = node_obj.advertise<geometry_msgs::WrenchStamped>("/force_sensor",100);
    w_msg.header.stamp = now;
    w_msg.header.frame_id = "sensor_body";
    w_msg.wrench.force.x = d->sensordata[0];
    w_msg.wrench.force.y = d->sensordata[1];
    w_msg.wrench.force.z = d->sensordata[2];
    force_publisher.publish(w_msg);
*/
/*
    ros::Publisher tf_publisher = node_obj.advertise<tf2_msgs::TFMessage>("/tf",100);
    geometry_msgs::TransformStamped ee_msg;
    tf2_msgs::TFMessage tf_msg;
    ee_msg.header.stamp = now;
    ee_msg.header.frame_id = "r_gripper";
    ee_msg.child_frame_id = "free";
    ee_msg.transform.translation.x = d->xpos[39];
    ee_msg.transform.translation.y = d->xpos[40];
    ee_msg.transform.translation.z = d->xpos[41];
    ee_msg.transform.rotation.x = d->xquat[52];
    ee_msg.transform.rotation.y = d->xquat[53];
    ee_msg.transform.rotation.z = d->xquat[54];
    ee_msg.transform.rotation.w = d->xquat[55];

    tf_msg.transforms.push_back(ee_msg);
    tf_publisher.publish(tf_msg);
*/
/*
    ros::Publisher rgba_publisher = node_obj.advertise<std_msgs::ColorRGBA>("/color",100);
    std_msgs::ColorRGBA col;
    col.r = m->mat_rgba[4];
    col.g = m->mat_rgba[5];
    col.b = m->mat_rgba[6];
    col.a = m->mat_rgba[7];
    ROS_INFO("%f, %f,%f,%f",col.r,col.g,col.b,col.a);
    rgba_publisher.publish(col);
*/
