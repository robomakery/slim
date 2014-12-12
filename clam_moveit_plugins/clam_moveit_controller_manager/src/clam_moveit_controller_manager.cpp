/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Ioan Sucan, E. Gil Jones */

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_list_macros.h>

//#include <dynamixel_hardware_interface/SwitchController.h>
#include <dynamixel_hardware_interface/LoadController.h>
#include <dynamixel_hardware_interface/UnloadController.h>
#include <dynamixel_hardware_interface/ListControllers.h>
#include <clam_msgs/ClamGripperCommandAction.h>
#include <algorithm>
#include <map>

namespace clam_moveit_controller_manager
{

static const double DEFAULT_MAX_GRIPPER_EFFORT = 10000.0;

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Controller Handle
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  template<typename T>
  class ActionBasedControllerHandle : public moveit_controller_manager::MoveItControllerHandle
  {
  public:
    ActionBasedControllerHandle(const std::string &name, const std::string &ns) :
      moveit_controller_manager::MoveItControllerHandle(name),
      namespace_(ns),
      done_(true)
    {
      controller_action_client_.reset(new actionlib::SimpleActionClient<T>(name_ +"/" + namespace_, true));
      unsigned int attempts = 0;
      while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(5.0)) && ++attempts < 3)
        ROS_INFO_STREAM("Waiting for " << name_ + "/" + namespace_ << " to come up");

      if (!controller_action_client_->isServerConnected())
      {
        ROS_ERROR_STREAM("Action client not connected: " << name_ + "/" + namespace_);
        controller_action_client_.reset();
      }

      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    }

    bool isConnected() const
    {
      return controller_action_client_;
    }

    virtual bool cancelExecution()
    {
      if (!controller_action_client_)
        return false;
      if (!done_)
      {
        ROS_INFO_STREAM("Cancelling execution for " << name_);
        controller_action_client_->cancelGoal();
        last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        done_ = true;
      }
      return true;
    }

    virtual bool waitForExecution(const ros::Duration &timeout = ros::Duration(0))
    {
      if (controller_action_client_ && !done_)
        return controller_action_client_->waitForResult(timeout);
      return true;
    }

    virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
    {
      return last_exec_;
    }

  protected:

    void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
    {
      ROS_DEBUG_STREAM("Controller " << name_ << " is done with state " << state.toString() << ": " << state.getText());
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
      else
        if (state == actionlib::SimpleClientGoalState::ABORTED)
          last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
        else
          if (state == actionlib::SimpleClientGoalState::PREEMPTED)
            last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
          else
            last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
      done_ = true;
    }

    moveit_controller_manager::ExecutionStatus last_exec_;
    std::string namespace_;
    bool done_;
    boost::shared_ptr<actionlib::SimpleActionClient<T> > controller_action_client_;
  };

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Gripper
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
class ClamGripperControllerHandle : public ActionBasedControllerHandle<clam_msgs::ClamGripperCommandAction>
{
public:
  ClamGripperControllerHandle(const std::string &name, const std::string &ns  = "gripper_action") :
    ActionBasedControllerHandle<clam_msgs::ClamGripperCommandAction>(name, ns),
    closing_(false)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The ClamArm gripper controller cannot execute multi-dof trajectories.");
      return false;
    }

    if (trajectory.joint_trajectory.points.size() > 2)
    {
      ROS_ERROR("The ClamArm gripper controller expects a joint trajectory with one point only, but %u provided)", (unsigned int)trajectory.joint_trajectory.points.size());
      ROS_INFO_STREAM_NAMED("sendTrajectory","Message: " << trajectory);
      return false;
    }

    if (trajectory.joint_trajectory.points[0].positions.empty())
    {
      ROS_ERROR("The ClamArm gripper controller expects a joint trajectory with one point that specifies at least one position, but 0 positions provided)");
      return false;
    }

    clam_msgs::ClamGripperCommandGoal goal;
    goal.max_effort = DEFAULT_MAX_GRIPPER_EFFORT;

    // New gripper trajectories have two points now, for some reason
    if (trajectory.joint_trajectory.points.size() == 1)
      goal.position = trajectory.joint_trajectory.points[0].positions[0];
    else 
      goal.position = trajectory.joint_trajectory.points[1].positions[0];

    /*
      if (trajectory.joint_trajectory.points[0].positions[0] > 0.5)
      {
      goal.position = clam_msgs::ClamGripperCommandGoal::GRIPPER_OPEN;
      closing_ = false;
      ROS_DEBUG_STREAM("Sending gripper open command");
      }
      else
      {
      goal.position = clam_msgs::ClamGripperCommandGoal::GRIPPER_CLOSED;
      closing_ = true;
      ROS_DEBUG_STREAM("Sending gripper close command");
      }
    */

    controller_action_client_->sendGoal(goal,
					boost::bind(&ClamGripperControllerHandle::controllerDoneCallback, this, _1, _2),
					boost::bind(&ClamGripperControllerHandle::controllerActiveCallback, this),
					boost::bind(&ClamGripperControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

private:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const clam_msgs::ClamGripperCommandResultConstPtr& result)
  {
    /*
    // the gripper action reports failure when closing the gripper and an object is inside
    if (state == actionlib::SimpleClientGoalState::ABORTED && closing_)
    {
    ROS_DEBUG_STREAM_NAMED("controllerDoneCallback","gripper reports failure");
    finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    }
    else
    {
    ROS_DEBUG_STREAM_NAMED("controllerDoneCallback","gripper reports sucess");
    finishControllerExecution(state);
    }
    */
    ROS_DEBUG_STREAM_NAMED("controllerDoneCallback","Gripper done callback");
    finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const clam_msgs::ClamGripperCommandFeedbackConstPtr& feedback)
  {
  }

  bool closing_;
};

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Joint Trajectory
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
class ClamFollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>
{
public:

  ClamFollowJointTrajectoryControllerHandle(const std::string &name, const std::string &ns = "follow_joint_trajectory") :
    ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, ns)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The ClamArm FollowJointTrajectory controller cannot execute multi-dof trajectories.");
      return false;
    }
    if (done_)
      ROS_DEBUG_STREAM("Sending trajectory to FollowJointTrajectory action for controller " << name_);
    else
      ROS_DEBUG_STREAM("Sending continuation for the currently executed trajectory to FollowJointTrajectory action for controller " << name_);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;
    controller_action_client_->sendGoal(goal,
					boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
					boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
					boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

protected:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
  {
  }
};

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Manager
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
class ClamMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  ClamMoveItControllerManager() : node_handle_("~")
  {
    node_handle_.param("controller_manager_name", controller_manager_name_, std::string("clam_controller_manager"));
    node_handle_.param("use_controller_manager", use_controller_manager_, true);

    XmlRpc::XmlRpcValue controller_list;
    if (node_handle_.hasParam("controller_list"))
    {
      node_handle_.getParam("controller_list", controller_list);
      if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_WARN("Controller list should be specified as an array");
      else
        for (int i = 0 ; i < controller_list.size() ; ++i)
          if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
            ROS_WARN("Name and joints must be specifed for each controller");
          else
          {
            try
            {
              ControllerInformation ci;
              std::string name = std::string(controller_list[i]["name"]);
              if (controller_list[i].hasMember("default"))
              {
                try
                {
                  if (controller_list[i]["default"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                  {
                    ci.default_ = controller_list[i]["default"];
                  }
                  else
                  {
                    std::string def = std::string(controller_list[i]["default"]);
                    std::transform(def.begin(), def.end(), def.begin(), ::tolower);
                    if (def == "true" || def == "yes")
                      ci.default_ = true;
                  }
                }
                catch (...)
                {
                }
              }
              if (controller_list[i].hasMember("ns"))
                ci.ns_ = std::string(controller_list[i]["ns"]);
              if (controller_list[i]["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray)
              {
                int nj = controller_list[i]["joints"].size();
                for (int j = 0 ; j < nj ; ++j)
                  ci.joints_.push_back(std::string(controller_list[i]["joints"][j]));
              }
              else
                ROS_WARN_STREAM("The list of joints for controller " << name << " is not specified as an array");
              if (!ci.joints_.empty())
                possibly_unloaded_controllers_[name] = ci;
            }
            catch (...)
            {
              ROS_ERROR("Unable to parse controller information");
            }
          }
    }
    else
    {
      if (use_controller_manager_)
	ROS_DEBUG_STREAM("No controller list specified. Using list obtained from the " << controller_manager_name_);
      else
	ROS_ERROR_STREAM("Not using a controller manager and no controllers specified. There are no known controllers.");
    }

    if (use_controller_manager_)
    {
      static const unsigned int max_attempts = 5;
      unsigned int attempts = 0;
      while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/list_controllers", ros::Duration(5.0)) && ++attempts < max_attempts)
	ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/list_controllers" << " to come up");

      // DTC
      //if (attempts < max_attempts)
      //   while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/switch_controller", ros::Duration(5.0)) && ++attempts < max_attempts)
      //    ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/switch_controller" << " to come up");

      if (attempts < max_attempts)
        while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/load_controller", ros::Duration(5.0))  && ++attempts < max_attempts)
          ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/load_controller" << " to come up");

      if (attempts < max_attempts)
        while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/unload_controller", ros::Duration(5.0))  && ++attempts < max_attempts)
          ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/unload_controller" << " to come up");

      if (attempts < max_attempts)
      {
        lister_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::ListControllers>(controller_manager_name_ + "/list_controllers", true);
        //switcher_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::SwitchController>(controller_manager_name_ + "/switch_controller", true);
        loader_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::LoadController>(controller_manager_name_ + "/load_controller", true);
        unloader_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::UnloadController>(controller_manager_name_ + "/unload_controller", true);
      }
      else
        ROS_ERROR("Not using the ClamArm controller manager");
    }
  }

  virtual ~ClamMoveItControllerManager()
  {
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    ROS_DEBUG_STREAM_NAMED("getControllerHandle","Name is " << name);

    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = handle_cache_.find(name);
    if (it != handle_cache_.end())
      return it->second;

    moveit_controller_manager::MoveItControllerHandlePtr new_handle;
    if (possibly_unloaded_controllers_.find(name) != possibly_unloaded_controllers_.end())
    {
      const std::string &ns = possibly_unloaded_controllers_.at(name).ns_;
      new_handle = getControllerHandleHelper(name, ns);
    }
    else
      new_handle = getControllerHandleHelper(name, "");

    if (new_handle)
      handle_cache_[name] = new_handle;
    return new_handle;
  }

  virtual void getControllersList(std::vector<std::string> &names)
  {
    const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
    std::set<std::string> names_set;
    names_set.insert(res.controllers.begin(), res.controllers.end());

    for (std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.begin() ; it != possibly_unloaded_controllers_.end() ; ++it)
      names_set.insert(it->first);

    names.clear();
    names.insert(names.end(), names_set.begin(), names_set.end());

  }

  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    names.clear();
    if (use_controller_manager_)
    {
      const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
      for (std::size_t i = 0; i < res.controllers.size(); ++i)
        if (res.state[i] == "running")
          names.push_back(res.controllers[i]);
    }
    else
      // we assume best case scenario if we can't test whether the controller is active or not
      for (std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.begin() ; it != possibly_unloaded_controllers_.end() ; ++it)
        names.push_back(it->first);

  }

  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    if (use_controller_manager_)
    {
      const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
      names = res.controllers;
    }
    else
    {
      names.clear();
      for (std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.begin() ; it != possibly_unloaded_controllers_.end() ; ++it)
        names.push_back(it->first);
    }

  }

  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.find(name);
    if (it != possibly_unloaded_controllers_.end())
      joints = it->second.joints_;
    else
    {
      joints.clear();
      std::string param_name;
      if (node_handle_.searchParam(name + "/joints", param_name))
      {
        XmlRpc::XmlRpcValue joints_list;
        node_handle_.getParam(param_name, joints_list);
        if (joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
          for (int i = 0 ; i < joints_list.size() ; ++i)
            joints.push_back((std::string)joints_list[i]);
      }
      else
        if (node_handle_.searchParam(name + "/joint", param_name))
        {
          std::string joint_name;
          if (node_handle_.getParam(param_name, joint_name))
            joints.push_back(joint_name);
        }
      if (joints.empty())
        ROS_DEBUG("The joints for controller '%s' are not known and were not found on the ROS param server under '%s/joints'or '%s/joint'. "
                  "Perhaps the controller configuration is not loaded on the param server?", name.c_str(), name.c_str(), name.c_str());
      else
      {
        ControllerInformation &ci = possibly_unloaded_controllers_[name];
        ci.joints_ = joints;
      }
    }

  }

  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    if (use_controller_manager_)
    {
      const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
      for (std::size_t i = 0; i < res.controllers.size(); ++i)
      {
        if (res.controllers[i] == name)
        {
          state.active_ = false;
          if (res.state[i] == "running")
            state.active_ = true;
          break;
        }
      }
    }
    else
    {
      // if we cannot test, assume best case scenario.
      //state.loaded_ = true;
      state.active_ = true;
    }

    std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.find(name);
    if (it != possibly_unloaded_controllers_.end())
      if (it->second.default_)
        state.default_ = true;
    return state;
  }

  virtual bool loadController(const std::string &name)
  {
    ROS_DEBUG_STREAM_NAMED("clam_moveit_controller_manager","loadController(" << name << ")");

    // Fake out the controller manager if we are requesting the clam_gripper_controller
    // TODO: make this cleaner somehow - move the clam_gripper_controller into the domain of dynamixel_hardware_interface?
    if (name == "clam_gripper_controller")
    {
      ROS_DEBUG_STREAM_NAMED("loadController","Faked true response for clam_gripper_controller");
      return true;
    }

    if (!use_controller_manager_)
    {
      ROS_WARN_STREAM("Cannot load controller without using the controller manager");
      return false;
    }

    last_lister_response_ = ros::Time();
    handle_cache_.erase(name);
    dynamixel_hardware_interface::LoadController::Request req;
    dynamixel_hardware_interface::LoadController::Response res;
    req.name = name;
    if (!loader_service_.call(req, res))
    {
      ROS_WARN_STREAM("Something went wrong with loader service");
      return false;
    }
    if (!res.ok)
      ROS_WARN_STREAM("Loading controller " << name << " failed");
    return res.ok;
  }

  virtual bool unloadController(const std::string &name)
  {
    ROS_DEBUG_STREAM_NAMED("clam_moveit_controller_manager","unloadController(" << name << ")");

    if (!use_controller_manager_)
    {
      ROS_WARN_STREAM("Cannot unload controller without using the controller manager");
      return false;
    }
    last_lister_response_ = ros::Time();
    handle_cache_.erase(name);
    dynamixel_hardware_interface::UnloadController::Request req;
    dynamixel_hardware_interface::UnloadController::Response res;
    req.name = name;
    if (!unloader_service_.call(req, res))
    {
      ROS_WARN_STREAM("Something went wrong with unloader service");
      return false;
    }
    if (!res.ok)
      ROS_WARN_STREAM("Unloading controller " << name << " failed");
    return res.ok;
  }

  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate)
  {
    ROS_DEBUG_STREAM_NAMED("clam_moveit_controller_manager","switchController NOT IMPLEMENTED. activate:");
    //std::copy(activate.begin(),activate.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

    /*
      if (!use_controller_manager_)
      {
      ROS_WARN_STREAM("Cannot switch controllers without using the controller manager");
      return false;
      }
      last_lister_response_ = ros::Time();
      dynamixel_hardware_interface::SwitchController::Request req;
      dynamixel_hardware_interface::SwitchController::Response res;

      req.strictness = dynamixel_hardware_interface::SwitchController::Request::BEST_EFFORT;
      req.load_controllers = activate;
      req.unload_controllers = deactivate;
      if (!switcher_service_.call(req, res))
      {
      ROS_WARN_STREAM("Something went wrong with switcher service");
      return false;
      }
      if (!res.ok)
      ROS_WARN_STREAM("Switcher service failed");
      return res.ok;
    */
    return true;
  }

protected:

  const dynamixel_hardware_interface::ListControllers::Response &getListControllerServiceResponse()
  {
    if (use_controller_manager_)
    {
      static const ros::Duration max_cache_age(10.0);
      if ((ros::Time::now() - last_lister_response_) > max_cache_age)
      {
	dynamixel_hardware_interface::ListControllers::Request req;
	if (!lister_service_.call(req, cached_lister_response_))
	  ROS_WARN_STREAM("Something went wrong with lister service");
	last_lister_response_ = ros::Time::now();
      }
    }
    return cached_lister_response_;
  }

  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandleHelper(const std::string &name, const std::string &ns)
  {
    moveit_controller_manager::MoveItControllerHandlePtr new_handle;
    ROS_DEBUG_STREAM_NAMED("getControllerHandleHelper","Name = " << name);

    if (name == "clam_gripper_controller")
    {
      new_handle.reset(ns.empty() ? new ClamGripperControllerHandle(name) : new ClamGripperControllerHandle(name, ns));
      if (!static_cast<ClamGripperControllerHandle*>(new_handle.get())->isConnected())
	new_handle.reset();
    }
    else
    {
      new_handle.reset(ns.empty() ? new ClamFollowJointTrajectoryControllerHandle(name) : new ClamFollowJointTrajectoryControllerHandle(name, ns));
      if (!static_cast<ClamFollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
	new_handle.reset();
    }
    return new_handle;
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle root_node_handle_;

  std::string controller_manager_name_;
  bool use_controller_manager_;
  ros::ServiceClient loader_service_;
  ros::ServiceClient unloader_service_;
  //ros::ServiceClient switcher_service_;
  ros::ServiceClient lister_service_;

  ros::Time last_lister_response_;
  dynamixel_hardware_interface::ListControllers::Response cached_lister_response_;

  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> handle_cache_;

  struct ControllerInformation
  {
    ControllerInformation() : default_(false)
    {
    }

    bool default_;
    std::string ns_;
    std::vector<std::string> joints_;
  };
  std::map<std::string, ControllerInformation> possibly_unloaded_controllers_;
};

}

PLUGINLIB_EXPORT_CLASS(clam_moveit_controller_manager::ClamMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
