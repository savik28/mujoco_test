#include "ros/ros.h"
#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "iostream"
#include "fstream"
#include <mujoco_test/helpers.h>

using namespace std;

class JointStateInterpreter
{
public:
  JointStateInterpreter(const ros::NodeHandle& nh): nh_(nh)
  {
    // here is the place to init any variables
    // TODO: get rid of this
    for (size_t i=0; i<8; ++i)
    {
      start_pose[i] = 0;
      vel_set_point[i] = 0;
    }
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
    std::string license_file, model_file;
    if (!nh_.getParam("license_file", license_file))
      throw std::runtime_error("Did not find parameter 'license_file'.");
    if (!nh_.getParam("model_file", model_file))
      throw std::runtime_error("Did not find parameter 'model_file'.");

    mj_activate(license_file.c_str());

    // TODO: get rid of this 1000
    m = mj_loadXML(model_file.c_str(), NULL, error, 1000);
    cout<<"success"<<endl;

    std::vector<int> body_ids = mujoco_test::get_free_body_ids(m);

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
                 <<"No.of Free Objects:"<< body_ids.size() <<endl<<endl;
    }

    d = mj_makeData(m);

    js_msg = mujoco_test::get_joint_state(m,d,ros::Time::now());
    js_new = js_msg;

    js_old = mujoco_test::set_joint_pos (m,d,start_pose);

    sub_ = nh_.subscribe("joint_states_in", 1, &JointStateInterpreter::js_callback, this);
    pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states_out", 10);

    js_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states",100);
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100);
    wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>("/ee_wrench",100);
  }

private:
  // here is the place to add variables, like mjModel* m and mjData* d;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_,js_pub,marker_pub,wrench_pub;
  mjModel* m;
  mjData* d;
  char error[1000];
  sensor_msgs::JointState js_msg,js_old,js_new;
  visualization_msgs::MarkerArray markers_msg;
  geometry_msgs::WrenchStamped ee_wrench_msg;
  // setting the set points for Joint angles in radians
  double start_pose[8];
  double vel_set_point[8];

  void js_callback(const sensor_msgs::JointStateConstPtr& message)
  {

    js_new = *message;

    d = mujoco_test::set_control_input (m,d,js_new,js_old,vel_set_point);

    mj_step(m,d); //simulation

    ros::Time now = ros::Time::now();
    js_msg = mujoco_test::get_joint_state(m, d, now);
    markers_msg.markers = mujoco_test::get_free_body_markers(m, d, now);
    ee_wrench_msg = mujoco_test::get_ee_wrench(m, d, now);

    js_pub.publish(js_msg);
    marker_pub.publish(markers_msg);
    wrench_pub.publish(ee_wrench_msg);
    pub_.publish(*message);

    js_old = js_new;
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
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  my_jsi.~JointStateInterpreter();
  return 0;
}
