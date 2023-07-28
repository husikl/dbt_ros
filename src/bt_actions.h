#include <actionlib/server/simple_action_server.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <dbt_ros/bt_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <asar_hybrid_tmp/GraspAction.h>
#include <asar_hybrid_tmp/GraspTrajectory.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <ctime>
#include <cstdio>
#include <typeinfo>

using namespace BT;
using namespace std;

//-----------------------------------------------------


class GraspNeedle: public RosActionNode<asar_hybrid_tmp::GraspAction>
{

public:
  GraspNeedle( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<asar_hybrid_tmp::GraspAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      // InputPort<std::string>("id"),
      InputPort<std::string>("command"),
      InputPort<geometry_msgs::Pose>("grasp"),
      // InputPort<geometry_msgs::Pose>("pose"),
      // InputPort<std::vector<geometry_msgs::Pose>>("traj"),
      // InputPort<geometry_msgs::PoseArray>("graspTraj"),
      InputPort<int>("suture_id"),
    //   ,
      OutputPort<bool>("success")
      };
  }

  bool sendGoal(GoalType& goal) override
  {
    ROS_WARN("sending goal ...");
    if (!getInput("command", goal.command) ) 
    {
      ROS_INFO("no command specified ...");
      // ROS_INFO_STREAM("goal.targetPose= " << goal.targetPose);
      return false;
    }
    // if (!getInput("grasp", goal.grasp))
    // {
    //   if (goal.command == "grasp_n")
    //   {
    //     // goal.command = "grasp_with_sampling";
    //     ROS_WARN("grasp with sampling is requested ...");
    //   }
    // }
    getInput("command",goal.command);
    getInput("grasp",goal.grasp);
    // getInput("pose",goal.targetPose);
    getInput("suture_id",goal.id);
    // geometry_msgs::PoseArray grasps;
    // getInput("graspTraj",grasps);
    // for (auto g : grasps.poses)
    // {
    //   goal.multi_grasp.push_back(g);
    // }
    ROS_INFO("GraspNeedle: sending goal");
    ROS_INFO_STREAM("goal.command= " << goal.command);
    // ROS_INFO_STREAM("goal.grasp= " << goal.grasp);
    // sleep(10.0);
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("GraspNeedle: result received");
    if (res.success)
    {
      setOutput<bool>("success",res.success);
      return NodeStatus::SUCCESS;
    }
      
    else 
      return NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("GraspNeedle request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("GraspNeedle halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
  std::string id;
};





// -----------------------------------------------------------------------------------------------------------------------
