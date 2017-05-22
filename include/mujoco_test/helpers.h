#include <mjmodel.h>
#include <mjdata.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <exception>
#include <vector>

#ifndef MUJOCO_TEST_HELPERS_H
#define MUJOCO_TEST_HELPERS_H

namespace mujoco_test
{
  inline bool joint_index_valid(const mjModel* model, size_t index)
  {
    return index > model->njnt;
  }

  inline void check_joint_index(const mjModel* model, size_t index)
  {
    if (joint_index_valid(model, index))
      throw std::runtime_error("Given joint index exceeds number of joints.");
  }

  inline bool is_slide_joint(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return model->jnt_type[index] == mjJNT_SLIDE;
  }

  inline bool is_hinge_joint(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return model->jnt_type[index] == mjJNT_HINGE;
  }

  inline bool is_free_joint(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return model->jnt_type[index] == mjJNT_FREE;
  }

  inline bool is_1dof_joint(const mjModel* model, size_t index)
  {
    return is_hinge_joint(model, index) || is_slide_joint(model, index);
  }

  inline std::string get_joint_name(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return std::string(model->names + model->name_jntadr[index]);
  }

  inline mjtNum get_qpos(const mjModel* model, const mjData* data, size_t index)
  {
    check_joint_index(model, index);
    return data->qpos[model->jnt_qposadr[index]];
  }

  inline mjtNum get_qvel(const mjModel* model, const mjData* data, size_t index)
  {
    check_joint_index(model, index);
    return data->qvel[model->jnt_dofadr[index]];
  }

  inline sensor_msgs::JointState get_joint_state(const mjModel* model, const mjData* data, const ros::Time& now)
  {
    sensor_msgs::JointState result;
    result.header.stamp = now;
    for (size_t i=0; i<model->njnt; ++i)
      if (is_1dof_joint(model, i))
      {
        result.name.push_back(get_joint_name(model, i));
        result.position.push_back(get_qpos(model,data,i));
        result.velocity.push_back(get_qvel(model,data,i));
      }

    return result;
  }

  inline bool body_index_valid(const mjModel* model, size_t index)
  {
    return index > model->nbody;
  }

  inline void check_body_index(const mjModel* model, size_t index)
  {
    if (body_index_valid(model, index))
      throw std::runtime_error("Given body index '" + std::to_string(index) + "' exceeds number of bodies.");
  }

  inline std::vector<int> get_free_body_ids(const mjModel* model)
  {
    std::vector<int> free_body_ids;
    for (size_t i=0; i<model->nbody; ++i)
    {
      check_body_index(model, i);
      if(model->body_dofnum[i] == 6) // TODO: check whether this necessary
        if(is_free_joint(model, model->body_jntadr[i]))
          free_body_ids.push_back(i);
    }

    return free_body_ids;
  }

  inline visualization_msgs::Marker get_free_body_marker(const mjModel* model, const mjData* data, const ros::Time& now, size_t body_id)
  {
    check_body_index(model, body_id);

    visualization_msgs::Marker object;

    int g_index = model->body_geomadr[body_id];
    object.header.frame_id = "/world";
    object.header.stamp = now;
    object.ns = "free_objects";
    object.id = body_id; //geom_id/body_id
    object.action = visualization_msgs::Marker::ADD;

    if(model->geom_type[g_index] == mjGEOM_BOX)
    {
      object.type = visualization_msgs::Marker::CUBE;

      object.scale.x = model->geom_size[(g_index*3)+0]*2;
      object.scale.y = model->geom_size[(g_index*3)+1]*2;
      object.scale.z = model->geom_size[(g_index*3)+2]*2;
    }

    if(model->geom_type[g_index] == mjGEOM_CYLINDER)
    {
      object.type = visualization_msgs::Marker::CYLINDER;

      object.scale.x = model->geom_size[(g_index*2)+0]*2; //diameter
      object.scale.y = model->geom_size[(g_index*2)+0]*2;
      object.scale.z = model->geom_size[(g_index*3)+1]*2; //full height
    }

    object.color.r = model->geom_rgba[(g_index*4)+0];
    object.color.g = model->geom_rgba[(g_index*4)+1];
    object.color.b = model->geom_rgba[(g_index*4)+2];
    object.color.a = model->geom_rgba[(g_index*4)+3]; // Don't forget to set the alpha!

    object.pose.position.x = data->xpos[(body_id*3)+0];
    object.pose.position.y = data->xpos[(body_id*3)+1];
    object.pose.position.z = data->xpos[(body_id*3)+2];
    object.pose.orientation.x = data->xquat[(body_id*3)+0];
    object.pose.orientation.y = data->xquat[(body_id*3)+1];
    object.pose.orientation.z = data->xquat[(body_id*3)+2];
    object.pose.orientation.w = data->xquat[(body_id*3)+3];

    return object;
  }

  inline std::vector<visualization_msgs::Marker> get_free_body_markers(const mjModel* model, const mjData* data, const ros::Time& now)
  {
    std::vector<visualization_msgs::Marker> markers;

    std::vector<int> body_ids = get_free_body_ids(model);
    for (size_t i=0; i<body_ids.size(); ++i)
      markers.push_back(get_free_body_marker(model, data, now, body_ids[i]));

    return markers;
  }

  inline mjData* set_control_input (const mjModel* model, mjData* data,sensor_msgs::JointState js_goal,sensor_msgs::JointState js_start, double vel_set_point[])
  {
    //assigning goals b4 simulation for the current time instant

    std::vector<int> body_ids = get_free_body_ids(model);
    int free_objects_in_scene  = body_ids.size();

    for (size_t i=0; i< (model->njnt-free_objects_in_scene); i++) //make a function to return num of free joints
    {
      //assigning start_pose b4 simulation
      data->qpos[i+(free_objects_in_scene*7)] = js_start.position[i]; // 1 free joint adds 7 nq's

      // first 8 in d->ctrl is used as motor actuator (online gravity compensation)
      data->ctrl[i] = data->qfrc_bias[i+(free_objects_in_scene*6)]; // 1 free joint adds 6 DOF's/nv's

      // next 8 in d->ctrl is used as position actuator
      data->ctrl[i+8] = js_goal.position[i];

      // next 8 in d->ctrl is used as velocity actuator
      data->ctrl[i+16] = vel_set_point[i];
    }

    return data;

  }

  inline sensor_msgs::JointState set_joint_pos(const mjModel* model, const mjData* data, double pose[])
  {
    std::vector<int> body_ids = get_free_body_ids(model);
    int free_objects_in_scene  = body_ids.size();

    sensor_msgs::JointState result;
    //result.header.stamp = now;
    for (size_t i=0; i<model->njnt; ++i)
      if (is_1dof_joint(model, i))
      {
        result.name.push_back(get_joint_name(model, i));
        result.position.push_back(get_qpos(model,data,i));
        result.velocity.push_back(get_qvel(model,data,i));
      }

    for (size_t j=0; j<model->njnt-free_objects_in_scene; j++)
      result.position.push_back(pose[j]);

    return result;
  }

  inline geometry_msgs::WrenchStamped get_ee_wrench(const mjModel* model, const mjData* data, const ros::Time& now)
  {
    geometry_msgs::WrenchStamped ee_wrench;
    ee_wrench.header.stamp = now;
     ee_wrench.header.frame_id = "ee_link";

    for (size_t i=0; i<model->nsensor; ++i)
    {
      if(model->sensor_type[i] == mjSENS_FORCE)
      {
          ee_wrench.wrench.force.x = data->sensordata[ model->sensor_adr[i]+0 ] ;// chance of using model->sensor_dim[i]
          ee_wrench.wrench.force.y = data->sensordata[ model->sensor_adr[i]+1 ] ;
          ee_wrench.wrench.force.z = data->sensordata[ model->sensor_adr[i]+2 ] ;
      }

      if(model->sensor_type[i] == mjSENS_TORQUE)
      {
          ee_wrench.wrench.torque.x = data->sensordata[ model->sensor_adr[i]+0 ];
          ee_wrench.wrench.torque.y = data->sensordata[ model->sensor_adr[i]+1 ];
          ee_wrench.wrench.torque.z = data->sensordata[ model->sensor_adr[i]+2 ];
      }
    }

    return ee_wrench;
  }


}


#endif
