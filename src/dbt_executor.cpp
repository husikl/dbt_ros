#include <dbt_ros/bt_service_node.h>
#include <dbt_ros/bt_action_node.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/controls/parallel_node.h"
#include <dbt_ros/GetPddl.h>
#include <dbt_ros/SendTree.h>
#include <dbt_ros/GetBT.h>
#include <dbt_ros/SuturePoints.h>
#include <dbt_ros/BTExecutorAction.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>
#include "bt_actions.h"
#include "bt_condition_node.h"


using namespace BT;
using namespace std;


string receivedTree;


bool treeReceived = false;
class BehaviorTreeReceiver
{
private:

  ros::NodeHandle nh_;
  std::string sub_name_;
  std::string pub_name_;

  ros::Subscriber suture_points_sub_;
  ros::Publisher result_pub_;
  // ros::ServiceServer send_tree;
  ros::ServiceClient update_tree_client;
  ros::ServiceClient pddl_client;
  ros::ServiceClient set_suture_points_client_;
  
  // for benchmarking only
  ros::ServiceClient bm_client_;
  NodeStatus status;
  double z;


public:
  bool updated;
  int  i;
  geometry_msgs::PoseArray suture_points;
  

  BehaviorTreeReceiver(ros::NodeHandle nodeHandle) :
    nh_(nodeHandle)
  {
    readParameters();
    update_tree_client = nh_.serviceClient<dbt_ros::GetBT>("/get_bt_service");
    pddl_client = nh_.serviceClient<dbt_ros::GetPddl>("/pddl_service");
    bm_client_ = nh_.serviceClient<std_srvs::SetBool>("/bt_completion_tracker");

    set_suture_points_client_ = nh_.serviceClient<dbt_ros::SuturePoints>("/is_sutured");
    i = 0;
    ROS_INFO("initialized ...");
  }

  virtual ~BehaviorTreeReceiver()
  {
    nh_.shutdown();
  }

  bool readParameters()
  {
    nh_.param("sub_topic", sub_name_, std::string("behavior_tree"));
    nh_.param("sub_topic", pub_name_, std::string("bt_result"));
    return true;
  }

  void sendExecutionCompletedStatus(bool success)
  {
    std_srvs::SetBool srvMsg;
    srvMsg.request.data = success;
    bm_client_.call(srvMsg);
    return;
  }

  bool getUpdateTree(std::string tree , NodeStatus status, string failedNode)
  {
    string updatedTree;
    dbt_ros::GetBT srv ;
    srv.request.type = "update";
    srv.request.bt = tree;
    srv.request.goal_condition = failedNode;
    update_tree_client.call(srv);
    updatedTree = srv.response.behavior_tree;
    
    if (!srv.response.success)
    {
      ROS_WARN("failed to update the tree ...");
      sendExecutionCompletedStatus(false);
      updated = false;
      // dbt_ros::GetPddl pddlSrv;
      // pddlSrv.request.type = "replan";
      // pddl_client.call(pddlSrv);
      // ROS_INFO("1");
      return false;
    }
      receivedTree = updatedTree.c_str();
      // std::cout << " ==========================" <<std::endl;
      // std::cout << receivedTree << endl;
      // std::cout << " ==========================" <<std::endl;
      treeReceived = true;
      updated = true;
      return true;
  }

  std::string tree;
};



std::string xml_text = R"(
 <root >
     <BehaviorTree>
          <isExtracted service_name="/suture_service" location="1"/>
     </BehaviorTree>
 </root>
 )";

bool tree_cb(dbt_ros::SendTree::Request &req, dbt_ros::SendTree::Response &res)
  {
    ROS_WARN("for request test_cb ...");
    receivedTree = req.tree.c_str();
    // std::cout << " --------------------------" <<std::endl;
    // std::cout << receivedTree << endl;
    // std::cout << " --------------------------" <<std::endl;
    treeReceived = true;
    ROS_WARN("got new tree !!!");    
    return true;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tree_executor");
  ros::NodeHandle nh, nh2;
  ros::ServiceServer service = nh2.advertiseService("/send_tree", tree_cb);
  BehaviorTreeReceiver receiver(nh);
  BehaviorTreeFactory factory;

  RegisterRosService<isExtracted>(factory, "isExtracted", nh);
  RegisterRosService<isInserted>(factory, "isInserted", nh);
  RegisterRosService<isClose>(factory, "isClose", nh);
  RegisterRosService<IsHandFree>(factory, "IsHandFree", nh);
  RegisterRosService<isHolding>(factory, "isHolding", nh);
  RegisterRosService<isGraspIKValid>(factory, "isGraspIKValid", nh);
  RegisterRosAction<GraspNeedle>(factory, "GraspNeedle", nh);



  

  ros::Rate r(50);

  ros::spinOnce();
  r.sleep();

  bool success = true;
  auto tree = factory.createTreeFromText(xml_text);

  
  time_t now = time(0);
  tm *ltm = localtime(&now);

  string filename1 = "/home/nir/dbt_logs/DBT2";
  // string filename2 = "BT_Log_";
  filename1.append(to_string(ltm->tm_mon));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_mday));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_hour));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_min));
  filename1.append(".fbl");
  FileLogger logger_file(tree, filename1.c_str());

  NodeStatus status = NodeStatus::IDLE;
  
  std::string failedN;
  int i = 0;
  receivedTree = xml_text;

  bool execute = false;

  while( ros::ok() )
  {
    
    // ROS_WARN("looping ");
    // ROS_INFO("waiting ...");
    // while (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)
    //   {
    //     // ROS_INFO("ticking ...");
    //     StdCoutLogger logger(tree);
    //     status = tree.tickRoot();
    //     r.sleep();
    //     failedN =logger.failed_Node;
        
    //   }
    
    if (treeReceived)
    {
      ROS_WARN("got new tree?");
      status = NodeStatus::IDLE;
      auto tree1 = factory.createTreeFromText(receivedTree);
      printTreeRecursively(tree1.rootNode());
      string LogName = "/home/nir/dbt_logs/DBT_";
      LogName.append(to_string(ltm->tm_mon));
      LogName.append("-");
      LogName.append(to_string(ltm->tm_mday));
      LogName.append("-");
      LogName.append(to_string(ltm->tm_hour));
      LogName.append("_");
      LogName.append(to_string(ltm->tm_min));
      LogName.append(".fbl");

      FileLogger logger_file(tree1, LogName.c_str());
      PublisherZMQ publisher_zmq(tree1);

      while (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)
      {
        
        // ROS_INFO("ticking ...");
        
        StdCoutLogger logger(tree1);
        status = tree1.tickRoot();

        r.sleep();

        failedN =logger.failed_Node;
        r.sleep();
      }
      treeReceived = false;
      execute = true;
      // FileLogger logger_file2(tree1, LogName.c_str());
    }
    if (status == NodeStatus::FAILURE && execute)
    {
      ROS_WARN("requesting tree update");

      bool gotTree = receiver.getUpdateTree(receivedTree, status, failedN );
      if (!gotTree)
        execute = false;
      
    }

    if (status == NodeStatus::SUCCESS && execute)
    {
      ROS_WARN("bt execution succeede send signal to record data");
      receiver.sendExecutionCompletedStatus(true);
      execute = false;
      
    }
    
    ros::spinOnce();
    r.sleep();
    // ROS_INFO("looping ...");
  }

  return 0;
}
