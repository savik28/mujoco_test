#include <ros/ros.h>
#include "iostream"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>


using namespace std;
using namespace KDL; // to be used with Tree,Chain,Frame,JntArray

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "kdl_test");

    Tree my_tree;
    if (!kdl_parser::treeFromFile("/home/student/ros_mujoco_ws/src/mujoco_test/urdf/ur5_with_gripper.urdf", my_tree))
    {
         ROS_ERROR("Failed to construct kdl tree");
         return false;
    }

    unsigned int tj = my_tree.getNrOfJoints();
    unsigned int ts = my_tree.getNrOfSegments();
    std::map<std::string,KDL::TreeElement>::const_iterator root = my_tree.getRootSegment();
    //std::map<std::string,KDL::TreeElement>::const_iterator chain_tip = my_tree.getSegment("gripper_base_link");
    //std::map<std::string,KDL::TreeElement>& segments = my_tree.getSegments();
    cout << endl << "A Tree is created with "<< ts << " segments and " << tj << " joints" << endl;

        cout<<"ROOT of the TREE =>" << root->first <<endl;

    bool chain_done;
    Chain my_chain;
    chain_done = my_tree.getChain("base_link","ee_link",my_chain);
    //ex=my_tree.getChain("gripper_base_link","",left_chain);
    //ey=my_tree.getChain("gripper_base_link","",right_chain);
    //ez=my_chain.addChain (left_chain/right_chain);

    unsigned int cj = my_chain.getNrOfJoints();
    unsigned int cs = my_chain.getNrOfSegments();
    Segment rt_seg;
    rt_seg = my_chain.getSegment(5);
    //cout<< " "; // error in printing rt_seg
    // 3 functions mentioned below are missing
    /*bool exit_value;
    Segment root_segment;
    Segment leaf_segment;
    std::vector<Segment> segments;
    exit_value = chain1.getRootSegment(root_segment);
    exit_value = chain1.getLeafSegment(leaf_segment);
    exit_value = chain1.getSegments(segments);*/

    if(chain_done)
        cout <<"A Chain is created with " << cs << " segments and " << cj << " joints" << endl;
    else
        cout <<"Chain Creation Failed" << endl;


    //Vector my_gravity = Vector(0.0, 0.0,-9.81);
    Vector my_gravity(0.0, 0.0,-9.81);
    JntArray jnt_q(cj),jnt_qd(cj),gravity_mat(cj),coriolis_mat(cj);
    JntSpaceInertiaMatrix inertia_mat(cj);
    int ib,gb,cb;
    double pos[6] = {0,-1.42,1.45,-3.14,-1.46,0};
    double vel[6] = {0,-0.05,0.015,0.14,-0.06,0};
    for(int c=0; c < cj; c++)
    {
      jnt_q(c) = pos[c];
      jnt_qd(c) = vel[c];
    }

    ChainDynParam forces_calc = ChainDynParam(my_chain,my_gravity);
    ib = forces_calc.JntToMass(jnt_q, inertia_mat);
    gb = forces_calc.JntToGravity(jnt_q, gravity_mat);
    cb = forces_calc.JntToCoriolis(jnt_q, jnt_qd, coriolis_mat);
    if ((ib && gb && cb) >= 0)
        cout<< "Dynamic Matrices M,G & C are calculated"<< endl;

    for (unsigned int i=0; i < cj; i++)
    {
        for (unsigned int j=0; j < cj; j++ )
        {
           cout << inertia_mat(i,j)<<";";// problems with printing
        }
        cout <<"::" << gravity_mat(i)<<"::"<< coriolis_mat(i)<<endl;
    }



    Frame cartpos;
    ChainFkSolverPos_recursive my_fksolver(my_chain);
    bool kinematics_status;
    kinematics_status = my_fksolver.JntToCart(jnt_q,cartpos);

    if(kinematics_status >= 0)
        cout <<"Forward Kinematics successful----" <<endl; //dont know how to print cartpos
    else
        cout <<"Forward Kinematics failed" <<endl;


    return 0;
}
