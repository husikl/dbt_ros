
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>

#include <asar_hybrid_tmp/isHolding.h>
#include <asar_hybrid_tmp/SuturePoints.h>
#include <asar_hybrid_tmp/PddlMotions.h>





namespace BT
{

// ------------------------------------------------------------------------------------------------------------
class isHolding : public RosServiceNode<asar_hybrid_tmp::isHolding> 
{
  public:
  isHolding( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::isHolding>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("unit_id")
      };
  }
  void sendRequest(RequestType& request) override
  {

    getInput("unit_id", request.state);
    ROS_INFO_STREAM("is holding request ~ " << request);
    ROS_INFO("isHolding: sending request");
    // sleep(10.0);
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("isHolding: response received");
    if( rep.holding  )
    {

      return NodeStatus::SUCCESS;
      ROS_INFO("holding ...");
      
    }
    else{
      ROS_INFO("not holding ...");
      // ROS_WARN("Not holding ,!");
      // sleep(3.0);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("isHolding request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// *--------------------------------------------------**************************************************************

class IsHandFree : public RosServiceNode<asar_hybrid_tmp::isHolding> 
{
  public:
  IsHandFree( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::isHolding>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("unit_id")
      };
  }
  void sendRequest(RequestType& request) override
  {

    getInput("unit_id", request.state);
    ROS_INFO("IsHandFree: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsHandFree: response received");
    if( !rep.holding )
    {

      return NodeStatus::SUCCESS;
    }
    else{

      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsHandFree request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// ------------------------------------------------------------------------------------------------------------

class isClose : public RosServiceNode<asar_hybrid_tmp::SuturePoints> 
{
  public:
  isClose( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::SuturePoints>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("location"),
      // BT::InputPort<geometry_msgs::Point>("pose")
      };
  }
  void sendRequest(RequestType& request) override
  {
    request.type = "is_close";
    getInput("location", request.id);
    ROS_INFO("isClose: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("isClose: response received");
    if( rep.close_to )
    {

      return NodeStatus::SUCCESS;
    }
    else{

      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("isClose request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};


class isInserted : public RosServiceNode<asar_hybrid_tmp::SuturePoints> 
{
  public:
  isInserted( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::SuturePoints>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<geometry_msgs::Point>("inP"),
      // BT::InputPort<geometry_msgs::Point>("exP"),
      // BT::InputPort<int>("id")
      BT::InputPort<int>("location")
      };
  }
  void sendRequest(RequestType& request) override
  {

    // getInput("inP", request.insertion_point);
    // getInput("exP", request.extraction_point);
    // getInput("location", request.location);
    request.type = "is_inserted";
    getInput("location", request.id);
    // ROS_INFO("isInserted: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    // ROS_INFO("isInserted: response received");
    if( rep.inserted )
    {
      ROS_WARN("is inserted ..!!!");
      // sleep(3.0);
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;

  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("isInserted request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};


class isExtracted : public RosServiceNode<asar_hybrid_tmp::SuturePoints> 
{
  public:
  isExtracted( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::SuturePoints>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<geometry_msgs::Point>("inP"),
      // BT::InputPort<geometry_msgs::Point>("exP"),
      // BT::InputPort<int>("id")
      BT::InputPort<int>("location")
      };
  }
  void sendRequest(RequestType& request) override
  {

    // getInput("inP", request.insertion_point);
    // getInput("exP", request.extraction_point);
    getInput("location", request.id);
    request.type = "is_extracted";
    ROS_INFO("isExtracted: sending request");

  }
  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("isExtracted: response received");
    if( rep.extracted )
    {

      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;

  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("isExtracted request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};

class isGraspIKValid : public RosServiceNode<asar_hybrid_tmp::PddlMotions> 
{
  public:
  isGraspIKValid( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<asar_hybrid_tmp::PddlMotions>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("location"),
       BT::InputPort<std::string>("unit_id")
      };
  }
  void sendRequest(RequestType& request) override
  {

    getInput("location", request.location_id);
    request.type = "grasp_check";
    ROS_INFO("isGraspIKValid: sending request");

  }
  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("isGraspIKValid: response received");
    if( rep.IKReachable )
    {

      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;

  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("isGraspIKValid request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};


class RosConditionNode : public BT::ConditionNode
{
protected:

  RosConditionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::ConditionNode(name, conf), node_(nh) { }

public:

  // using BaseClass    = RosConditionNode<ConditionT>;
  // using ConditionType  = ConditionT;

  RosConditionNode() = delete;

  virtual ~RosConditionNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
      };
  }


protected:

  // The node that will be used for any ROS operations
  virtual NodeStatus tick() override;
  ros::NodeHandle& node_;

};




/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosCondition(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosConditionNode::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

// #endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
