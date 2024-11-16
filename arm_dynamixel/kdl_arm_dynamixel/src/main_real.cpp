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

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

using std::placeholders::_1;


class ArmDynamixel : public rclcpp::Node
{
public:
  ArmDynamixel() : Node("armando_robot")
  {
    // Subscribe to URDF
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1))
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&ArmDynamixel::robotDescriptionCallback, this, _1));

    publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
      "set_position", 
      10
      );
  }

  uint16_t convert_to_bits(double rad){
    double angle = rad * 180.0 / M_PI + 150.0;
    uint16_t bits = angle * 1024.0 / 300.0;
    if (bits < 0){
      bits = 0;
    }
    if (bits > 1023)  
    {
      bits = 1023;
    }
    
    return bits;
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
    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, 1e-2, 10000, 1e-5);
    solver_d_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    auto joint_state_msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();

    double j1 = 0.0;
    double j2 = 0.3;
    double j3 = 0.3;
    double j4 = 0.3;

    KDL::JntArray q_init  = KDL::JntArray(chain_.getNrOfJoints());
    q_init(0) = j1;
    q_init(1) = j2;
    q_init(2) = j3;
    q_init(3) = j4;

    KDL::JntArray q  = q_init;
    KDL::Frame frame;

    for(int i = 0; i < chain_.getNrOfJoints();  i++){
      joint_state_msg.id = i+1;
      joint_state_msg.position = convert_to_bits(q_init(i));
      publisher_->publish(joint_state_msg);
      std::cout << "Joint " << i << " set to " << convert_to_bits(q_init(i)) << std::endl;
    }

    solver_d_->JntToCart(q_init, frame);

    double x = frame.p.x();
    double y = frame.p.y();
    double z = frame.p.z(); 

    while(rclcpp::ok()){
      std::cout << "Current target: " << x << ", " << y << ", " << z << std::endl;
      std::cout << "Enter x: ";
      std::cin >> x;
      std::cout << "Enter y: ";
      std::cin >> y;
      std::cout << "Enter z: ";
      std::cin >> z;
      if(z < 0.05){
        std::cout << "z value too low. Limit is 0" << std::endl;
        continue;
      }
      if(x < 0.05 || y < 0.05){
        std::cout << "x and/or y values too low. Limit is 0.05" << std::endl;
        continue;
      }
      usageExample(x, y, z, q);
    }
  }

  //!
  //! Cartesian x, z => hip and knee joint angles
  void getJointAngles(
    double x, double y, double z,  KDL::JntArray& q)
  {
    const KDL::Frame p_in(KDL::Vector(x, y, z));
    KDL::JntArray q_out(chain_.getNrOfJoints());
    // Run IK solver
    int error_code = solver_->CartToJnt(q, p_in, q_out);
    
    std::cout << solver_->strError(error_code) << std::endl;

    auto joint_state_msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();

    for(int i = 1; i < chain_.getNrOfJoints(); i++){
      if (q_out(i) < -1.57){
        q_out(i) = -1.57;
        std::cout << "Joint " << i << " is too low. Defaulting to -1.57" << std::endl;
      }else if(q_out(i) > 1.57){
        q_out(i) = 1.57;
        std::cout << "Joint " << i << " is too high. Defaulting to 1.57" << std::endl;
      }  
    }

    // Publish joint states
    for(int i = 0; i < chain_.getNrOfJoints(); i++){
      joint_state_msg.id = i+1;
      joint_state_msg.position = convert_to_bits(q_out(i));
      publisher_->publish(joint_state_msg);
      std::cout << "Joint " << i << " set to " << convert_to_bits(q_out(i)) << std::endl;
    }   

    q = q_out;
      
  }

  //!
  //! Calculate and print the joint angles
  //! for moving the foot to {x: 0.3, z: -0.6}
  void usageExample(double x, double y, double z, KDL::JntArray& q)
  {
    getJointAngles(x, y, z, q);
    printf("Required Joint Angles: %.1f, %.1f, %1f, %1f \n",
           q(0), q(1), q(2), q(3));
    fflush(stdout);
  }

  // Class members
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_d_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmDynamixel>());
  rclcpp::shutdown();
  return 0;
}
