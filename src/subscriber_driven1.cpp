#include "ros/ros.h"
#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "iostream"
#include "fstream"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"

using namespace std;

class JointStateInterpreter
{
public:
  JointStateInterpreter(const ros::NodeHandle& nh): nh_(nh)
  {
    // here is the place to init any variables
  }
  ~JointStateInterpreter()
  {
    // here is the place to delete any variable, like memory that you got
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();
  }

  void start()
  {
    mj_activate("/home/student/mjpro140/bin/mjkey.txt");

    objects_in_scene = 1;

    m = mj_loadXML("/home/student/mjpro140/model/ur5/ur5.xml", NULL, error, 1000);
    cout<<"success"<<endl;

    if( !m )
       printf("%s\n", error);

    else
    {
       cout<<endl<<"MODEL_PARAMETERS"<< endl
                 <<"Gen.Coordinates :"<< m->nq <<endl
                 <<"DOF's :"<< m->nv <<endl
                 <<"Bodys :"<< m->nbody <<endl
                 <<"Joints:"<< m->njnt <<endl
                 <<"Ctrl.IP:"<< m->nu <<endl
                 <<"No.of Free Objects:"<< objects_in_scene <<endl<<endl;
    }

    d = mj_makeData(m);

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

    for(int i=1; i <= objects_in_scene; i++)
    {
      box.header.frame_id = "/world";
      box.ns = "free_objects";
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;

      box.id = i; //geom_id/body_id
      box.scale.x = m->geom_size[(i*3)+0]*2;//0.1;
      box.scale.y = m->geom_size[(i*3)+1]*2;//0.1;
      box.scale.z = m->geom_size[(i*3)+2]*2;//0.4;

      box.color.r = m->geom_rgba[(i*4)+0];
      box.color.g = m->geom_rgba[(i*4)+1];
      box.color.b = m->geom_rgba[(i*4)+2];
      box.color.a = m->geom_rgba[(i*4)+3]; // Don't forget to set the alpha!
    }

    //start_pose[8]  = {0.5,-0.1,0.2,-2.5,-1,0,-0.05,0.05};   //{0.1,-1,1,-2.5,-1,0,-0.05,0.05};{0,0,0,0,0,0,0,0}
    //pos_set_point[8] = {-1,-0.8,0.9,-3.14,-1.57,0.8,-0.025,0.025}; //{0.3,-1.45,1.45,-3.14,-1.46,0.9,-0.04,0.04}{-1.57,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};
    //vel_set_point[8] = {0,0,0,0,0,0,0,0};

    for(int c=0; c < m->njnt-objects_in_scene; c++)
    {
        // first 8 in d->ctrl is used as motor actuator (online gravity compensation)
        d->ctrl[c+8] = pos_set_point[c];  // next 8 in d->ctrl is used as position actuator
        d->ctrl[c+16] = vel_set_point[c]; // next 8 in d->ctrl is used as velocity actuator
        d->qpos[c+(objects_in_scene*7)] = start_pose[c];
    }

    sub_ = nh_.subscribe("joint_states_in", 1, &JointStateInterpreter::js_callback, this);
    pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states_out", 1);
    js_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states",100);
    box_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
  }

private:
  // here is the place to add variables, like mjModel* m and mjData* d;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_,js_pub,box_pub;
  mjModel* m;
  mjData* d;
  char error[1000];
  int objects_in_scene;
  //double start_pose[8], pos_set_point[8], vel_set_point[8];
  sensor_msgs::JointState js_msg;
  visualization_msgs::Marker box;
  // setting the set points for Joint angles in radians
  double start_pose[8]    = {0.5,-0.1,0.2,-2.5,-1,0,-0.05,0.05};//{0.1,-1,1,-2.5,-1,0,-0.05,0.05};{0,0,0,0,0,0,0,0}
  double pos_set_point[8] = {-1,-0.8,0.9,-3.14,-1.57,0.8,-0.025,0.025};//{0.3,-1.45,1.45,-3.14,-1.46,0.9,-0.04,0.04}{-1.57,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};
  double vel_set_point[8] = {0,0,0,0,0,0,0,0};


  void js_callback(const sensor_msgs::JointStateConstPtr& message)
  {
    // put JointState into Mujoco
    // set control set points
    // simulate one step
    // publish object poses
    // publish contacts
    // publish actual joint states from Mujoco
    if ( ros::ok() )
    {
        while( d->time < 20 )
        {
            for(int e=0; e< m->njnt-objects_in_scene; e++)
            {
              d->ctrl[e] = d->qfrc_bias[e+(objects_in_scene*6)];// 1 free joint adds 6 DOF's
            }

            mj_step(m,d); //simulation
            ros::Time now = ros::Time::now();

            js_msg.header.stamp = now;
            for(int i=0; i < m->njnt-objects_in_scene; i++)
            {
               js_msg.position[i] = d->qpos[i+(objects_in_scene*7)];
               js_msg.velocity[i] = d->qvel[i+(objects_in_scene*6)];
            }

            for(int i=1; i <= objects_in_scene; i++)
            {
              box.header.stamp = now;
              box.pose.position.x = d->xpos[(i*3)+0];
              box.pose.position.y = d->xpos[(i*3)+1];
              box.pose.position.z = d->xpos[(i*3)+2];
              box.pose.orientation.x = d->xquat[(i*3)+0];
              box.pose.orientation.y = d->xquat[(i*3)+1];
              box.pose.orientation.z = d->xquat[(i*3)+2];
              box.pose.orientation.w = d->xquat[(i*3)+3];

              //cout<<box[i].pose.position.x<<":"<< box[i].pose.position.y<<endl;
              //cout<<d->xpos[3]<<":"<<d->xpos[4]<<endl;
            }

            js_pub.publish(js_msg);
            box_pub.publish(box);
            pub_.publish(*message);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    }
    ROS_INFO("All Messages are Published");
    ROS_INFO("Simulation done");

    for (int z=0; z < m->njnt-objects_in_scene; z++)
    {
      cout << endl<<"Joint-"<< z << endl
           <<"Goal::Cu.State::Error => ";
      cout << d->ctrl[z+8] <<"::"
           << d->qpos[z+(objects_in_scene*7)] <<"::" // 1 free joint adds 7 nq's
           << (d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)])<< endl;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_interpreter");
  ros::NodeHandle nh("~");
  JointStateInterpreter my_jsi(nh);
  try
  {
    my_jsi.start();
    //ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  my_jsi.~JointStateInterpreter();
  return 0;
}
