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

# define DEG2RAD 0.017453292519943295

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

    // dds
    // handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
    //     "rt/" + param::ns + "/cmd");
    // handcmd->msg_.cmds().resize(12);
    
    // handstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
    //     "rt/" + param::ns + "/state");
    // handstate->msg_.states().resize(12);

    // ROS2 subscriber for retargeted_qpos
    retargeted_qpos_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/retargeted_qpos", 10,
      std::bind(&InspireRunner::retargeted_qpos_callback, this, std::placeholders::_1));

    // Start running
    // thread = std::make_shared<unitree::common::RecurrentThread>(
    //   10000, std::bind(&InspireRunner::run, this)
    // );
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
        // auto angle_val = static_cast<double>(msg_data[i]) * 2;
        // eigen_vec(i) = (kAngleRangeMap[i].second - static_cast<double>(msg_data[i])) / (kAngleRangeMap[i].second - kAngleRangeMap[i].first);
        eigen_vec(i) = (0.3 - static_cast<double>(msg_data[i])) * 3.3;
        // eigen_vec(i) = 1;
      }
      else if(i == 5)
      {
        // eigen_vec(i) = (kAngleRangeMap[i].second - kAngleRangeMap[i].first - static_cast<double>(msg_data[i])) / (kAngleRangeMap[i].second - kAngleRangeMap[i].first);
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
    std::cout << "===========================" << std::endl;
    std::cout << "msg: " << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << " " << msg->data[3] << " " << msg->data[4] << " " << msg->data[5] << std::endl;
    std::cout << "qcmd: " << qcmd.transpose() << std::endl;
    righthand->SetPosition(qcmd.block<6, 1>(0, 0));
  }


//   void run()
//   {
//     // Set command for left hand (keeping existing functionality)
//     for(int i(6); i<12; i++)
//     {
//       qcmd(i) = handcmd->msg_.cmds()[i].q();
//     }
//     lefthand->SetPosition(qcmd.block<6, 1>(6, 0));

//     // Recv state
//     Eigen::Matrix<double, 6, 1> qtemp;
//     if(righthand->GetPosition(qtemp) == 0)
//     {
//       qstate.block<6, 1>(0, 0) = qtemp;
//     }
//     else
//     {
//       for(int i(0); i<6; i++)
//       {
//         handstate->msg_.states()[i].lost()++;
//       }
//     }
//     if(lefthand->GetPosition(qtemp) == 0)
//     {
//       qstate.block<6, 1>(6, 0) = qtemp;
//     }
//     else
//     {
//       for(int i(0); i<6; i++)
//       {
//         handstate->msg_.states()[i+6].lost()++;
//       }
//     }
//     if(handstate->trylock())
//     {
//         for(int i(0); i<12; i++)
//         {
//             handstate->msg_.states()[i].q() = qstate(i);
//         }
//         handstate->unlockAndPublish();
//     }
//   }

private:
  unitree::common::ThreadPtr thread;

  // inspire
  SerialPort::SharedPtr serial;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, 12, 1> qcmd, qstate;

  // dds
  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;

  // ROS2 subscriber
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr retargeted_qpos_sub_;

  const std::vector<std::pair<double, double>> kAngleRangeMap = {
    {19.0 * DEG2RAD, 176.7 * DEG2RAD},
    {19.0 * DEG2RAD, 176.7 * DEG2RAD}, 
    {19.0 * DEG2RAD, 176.7 * DEG2RAD},
    {19.0 * DEG2RAD, 176.7 * DEG2RAD},
    {-13.0 * DEG2RAD, 53.6 * DEG2RAD},
    {90 * DEG2RAD, 165 * DEG2RAD}
  };
};

int main(int argc, char ** argv)
{
  auto vm = param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;

  rclcpp::init(argc, argv);
  auto runner = std::make_shared<InspireRunner>();
  rclcpp::spin(runner);
  rclcpp::shutdown();
  
  return 0;
}