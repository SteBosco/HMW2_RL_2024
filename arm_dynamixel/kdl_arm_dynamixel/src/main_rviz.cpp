#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"

using std::placeholders::_1;


class LegIK : public rclcpp::Node
{
public:
  LegIK() : Node("leg_inverse_kinematics_example")
  {
    // Subscribe to URDF
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1))
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&LegIK::robotDescriptionCallback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 
      10
      );
  }

private:
  //!
  //! Construct KDL IK solver from URDF string

  void robotDescriptionCallback(const std_msgs::msg::String& msg)
  {
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;
    kdl_parser::treeFromString(urdf, tree_);
    // Print basic information about the tree
    std::cout << "nb joints:        " << tree_.getNrOfJoints() << std::endl;
    std::cout << "nb segments:      " << tree_.getNrOfSegments() << std::endl;
    std::cout << "root segment:     " << tree_.getRootSegment()->first
              << std::endl;
    // Get kinematic chain of the leg
    tree_.getChain("base_link", "wrist", chain_);
    std::cout << "chain nb joints:  " << chain_.getNrOfJoints() << std::endl;
    // Create IK solver
    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
    solver_d_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    auto joint_state_msg = sensor_msgs::msg::JointState();

    double j1 = 0.0;
    double j2 = 0.05;
    double j3 = -0.05;
    double j4 = 0.1;

    KDL::JntArray q_init  = KDL::JntArray(chain_.getNrOfJoints());
    q_init(0) = j1;
    q_init(1) = j2;
    q_init(2) = j3;
    q_init(3) = j4;

    KDL::Frame frame;
    KDL::JntArray q  = q_init;

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"j0", "j1", "j2", "j3"};
    joint_state_msg.position = {j1, j2, j3, j4};
    publisher_->publish(joint_state_msg);

    solver_d_->JntToCart(q_init, frame);

    double x = frame.p.x();
    double y = frame.p.y();
    double z = frame.p.z(); 

    while(rclcpp::ok()){
      std::cout << "Current position: " << x << ", " << y << ", " << z << std::endl;
      std::cout << "Enter x: ";
      std::cin >> x;
      std::cout << "Enter y: ";
      std::cin >> y;
      std::cout << "Enter z: ";
      std::cin >> z;
      if(z < 0.0){
        std::cout << "z value too low. Defaulting to 0" << std::endl;
        z = 0.0;
        continue;
      }
      usageExample(x, y, z, q);
    }
  }

  //!
  //! Cartesian x, z => hip and knee joint angles
  void getJointAngles(
    double x, double y, double z, KDL::JntArray& q)
  {
    const KDL::Frame p_in(KDL::Vector(x, y, z));
    KDL::JntArray q_out(chain_.getNrOfJoints());
    // Run IK solver
    solver_->CartToJnt(q, p_in, q_out);

    for(int i = 1; i < chain_.getNrOfJoints(); i++){
      if (q_out(i) < -1.57){
        q_out(i) = -1.57;
        std::cout << "Joint " << i << " is too low. Defaulting to -1.57" << std::endl;
      }else if(q_out(i) > 1.57){
        q_out(i) = 1.57;
        std::cout << "Joint " << i << " is too high. Defaulting to 1.57" << std::endl;
      }  
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();

    // Publish joint states
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"j0", "j1", "j2", "j3"};
    joint_state_msg.position = {q_out(0), q_out(1), q_out(2), q_out(3)};
    publisher_->publish(joint_state_msg);    

    q = q_out;
  }

  //!
  //! Calculate and print the joint angles
  //! for moving the foot to {x: 0.3, z: -0.6}
  void usageExample(double x, double y, double z, KDL::JntArray q)
  {
    getJointAngles(x, y, z, q);
    printf("Required Joint Angles: %.1f, %.1f, %1f, %1f \n",
           q(0), q(1), q(2), q(3));
    fflush(stdout);
  }

  // Class members
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_d_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegIK>());
  rclcpp::shutdown();
  return 0;
}
