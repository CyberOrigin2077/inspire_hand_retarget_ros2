#include "inspire_retarget/inspire.h"
#include "inspire_retarget/param.h"

#include "inspire_retarget/dds/Publisher.h"
#include "inspire_retarget/dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class InspireRunner : public rclcpp::Node
{
public:
  InspireRunner() : Node("inspire_runner")
  {
    serial = std::make_shared<SerialPort>(param::serial_port, B115200);

    // inspire
    righthand = std::make_shared<inspire::InspireHand>(serial, 1); // ID 1
    lefthand = std::make_shared<inspire::InspireHand>(serial, 2); // ID 2

    // Initialize DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

    // ROS2 subscriber for retargeted_qpos
    retargeted_qpos_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/retargeted_qpos", 10,
      std::bind(&InspireRunner::retargeted_qpos_callback, this, std::placeholders::_1));
  }

  /**
   * @brief Convert Float32MultiArray data to Eigen vector
   * @param msg_data Input Float32MultiArray data vector
   * @return Eigen::Matrix<double, 6, 1> Output vector
   */
  Eigen::Matrix<double, 6, 1> arrayToEigen(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    Eigen::Matrix<double, 6, 1> eigen_vec;
    const auto& msg_data = msg->data;
    // convert robot qpos to control signal
    for (size_t i = 0; i < 6; i++) {
      if(i <= 3)
      {
        eigen_vec(i) = (1.471 - static_cast<double>(msg_data[i]) )/ 1.471;
      }
      else if(i == 4)
      {
        eigen_vec(i) = (0.3 - static_cast<double>(msg_data[i])) * 3.3;
      }
      else if(i == 5)
      {
        eigen_vec(i) = (1.31 - static_cast<double>(msg_data[i]) )/ 1.31;
      }
    }
    return eigen_vec;
  }

  void retargeted_qpos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 6) {
      RCLCPP_WARN(this->get_logger(), "Received retargeted_qpos with incorrect size. Expected 6, got %zu", msg->data.size());
      return;
    }
    Eigen::Matrix<double, 6, 1> qcmd = arrayToEigen(msg);
    RCLCPP_INFO(this->get_logger(), "===========================");
    RCLCPP_INFO(this->get_logger(), "msg: %.3f %.3f %.3f %.3f %.3f %.3f", 
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
    RCLCPP_INFO(this->get_logger(), "qcmd: %.3f %.3f %.3f %.3f %.3f %.3f",
                qcmd(0), qcmd(1), qcmd(2), qcmd(3), qcmd(4), qcmd(5));
    righthand->SetPosition(qcmd.block<6, 1>(0, 0));
  }

private:
  // inspire
  SerialPort::SharedPtr serial;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, 12, 1> qcmd;

  // ROS2 subscriber
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr retargeted_qpos_sub_;
};

int main(int argc, char ** argv)
{
  auto vm = param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  RCLCPP_INFO(rclcpp::get_logger("inspire_retarget"), "Inspire Hand Retargeting...");

  rclcpp::init(argc, argv);
  auto runner = std::make_shared<InspireRunner>();
  rclcpp::spin(runner);
  rclcpp::shutdown();
  
  return 0;
}