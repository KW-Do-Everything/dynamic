#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include </home/capstone1/dynamic/src/dynamixel-workbench/dynamixel_workbench_toolbox/src/DynamixelWorkbench.h>
#include </home/capstone1/dynamic/src/dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_driver.h>

#include "../include/ctrl_gripper/gripper.h"
#include "open_manipulator_msgs/srv/setgripperpos.hpp"

using namespace std;

class GripperNode : public rclcpp::Node
{
private:
  DynamixelWorkbench dxl_wb;

  bool result, result1, result2;
  const char* log = nullptr;

  uint16_t model_number = 1060; // XL330-M288 1060
  const char *port_name = "/dev/ttyACM0";
  int baud_rate = 1000000;

  vector<uint8_t> dxlId = {8, 7};

  std::thread gripper_thread_;
  std::shared_ptr<rclcpp::Service<open_manipulator_msgs::srv::Setgripperpos>> service_;

  void gripper_run()
  {
    int32_t global_pos1 = 2048;
    int32_t global_pos2 = 2048;

    while (rclcpp::ok())
    {
      char ch;
      cin >> ch;

      cout << "Entered: " << ch << endl;
      int32_t goal_position1 = (ch == 'a') ? global_pos1 += 50 : (ch == 'd') ? global_pos1 -= 50 : -1;
      int32_t goal_position2 = (ch == 'a') ? global_pos2 -= 50 : (ch == 'd') ? global_pos2 += 50 : -1;

      if (ch == 's'){
        global_pos1 = 2048;
        global_pos2 = 2048;

        // Prepare data for sync write
        int32_t data[2] = { global_pos1, global_pos2 };
        uint8_t ids[2] = { dxlId[0], dxlId[1] };
        
        result = dxl_wb.syncWrite(0, ids, 2, data, 1, &log); // 0은 index 값입니다. 필요에 맞게 설정하



        if (!result)
        {
          std::cerr << "Failed to set goal position: " << log << std::endl;
        }

       }
       
      if ((goal_position1 != -1) and (goal_position2 !=-1))
      {
        // Prepare data for sync write
        int32_t data[2] = { goal_position1, goal_position2 };
        uint8_t ids[2] = { dxlId[0], dxlId[1] };
        
        result = dxl_wb.syncWrite(0, ids, 2, data, 1, &log); // 0은 index 값입니다. 필요에 맞게 설정하세요.


        if (!result)
        {
          std::cerr << "Failed to set goal position: " << log << std::endl;
        }
      }
    }
  }

public:
  GripperNode() : Node("gripper_node")
  {
    // init dxl
    result = dxl_wb.init(port_name, baud_rate, &log);
    if (!result)
    {
      printf("%s\n", log);
      printf("Failed to init\n");
      return;
    }
    printf("Succeeded to init(%d)\n", baud_rate);

    // check dxl
    for (auto dxl_id : dxlId)
    {
      result = dxl_wb.ping(dxl_id, &model_number, &log);
      if (!result)
      {
        printf("%s\n", log);
        printf("Failed to ping\n");
        return;
      }
      printf("Succeed to ping\n");
      printf("id : %d, model_number : %d\n", dxl_id, model_number);

      // change dxl mode to current based position mode
      result = dxl_wb.currentBasedPositionMode(dxl_id, 30, &log);
      if (!result)
      {
        printf("%s\n", log);
        printf("Failed to change current based position mode\n");
        return;
      }
      printf("Succeed to change current based position mode\n");

      // enable torque
      if (!dxl_wb.itemWrite(dxl_id, "Torque_Enable", 1, &log))
      {
        std::cerr << "Failed to enable torque for ID " << (int)dxl_id << ": " << log << std::endl;
        return;
      }
    }


    // Add Sync Write Handler for Goal_Position
    const ControlItem* goal_position_item1 = dxl_wb.getItemInfo(dxlId[0], "Goal_Position", &log);
    const ControlItem* goal_position_item2 = dxl_wb.getItemInfo(dxlId[1], "Goal_Position", &log);

    if (goal_position_item1 == nullptr)
    {
      std::cerr << "Failed to get Goal_Position item info: " << log << std::endl;
      return;
    }

    result1 = dxl_wb.addSyncWriteHandler(goal_position_item1->address, goal_position_item1->data_length, &log);
    result2 = dxl_wb.addSyncWriteHandler(goal_position_item2->address, goal_position_item2->data_length, &log);

    if (!result1 && result2)
    {
      std::cerr << "Failed to add sync write handler: " << log << std::endl;
      return;
    }

    gripper_thread_ = std::thread(&GripperNode::gripper_run, this);

    service_ = this->create_service<open_manipulator_msgs::srv::Setgripperpos>("set_gripper_position", std::bind(&GripperNode::handle_set_gripper_pos, this, std::placeholders::_1, std::placeholders::_2));
    std::cerr << "Gripper service is ready!" << std::endl; //서비스 생성

  }

// 서비스에 대해서 받은거에 따라 행동하는 부분
  void handle_set_gripper_pos(const std::shared_ptr<open_manipulator_msgs::srv::Setgripperpos::Request> request, 
                              std::shared_ptr<open_manipulator_msgs::srv::Setgripperpos::Response> response)
  {
      // std::cerr << "Received gripper command: " << (request->gripper_moving ? "Close" : "Open") << std::endl;

      // 로직 구현: 그리퍼를 열거나 닫습니다.
      uint8_t ids[2] = { dxlId[0], dxlId[1] };
      int32_t goal_position[2] = {2048, 2048}; // 초기 위치

      if (request->gripper_moving == "1") {
          goal_position[0] += 80; // 잡기/ 좁게하기
          goal_position[1] -= 80;

      } 
      else if(request->gripper_moving == "2")
      {
          goal_position[0] -= 340; // 놓기 /벌리기
          goal_position[1] += 340;
          
      }
      else if(request->gripper_moving == "3")
      {
          goal_position[0] += 120; // 크게 잡기
          goal_position[1] -= 120;
      }
      else if(request->gripper_moving == "4")
      {
          goal_position[0] -= 260; // 크게 놓기
          goal_position[1] += 260;
      }
      std::cerr << goal_position[1] << goal_position[0] << std::endl;

      std::cerr << "Gripper operation fuckcessful." << std::endl;
      result = dxl_wb.syncWrite(0, ids, 2, goal_position, 1, &log);
      if (result) {
          response->gripper_flag = true;
          std::cerr << "Gripper operation successful." << std::endl;
      } else {
          response->gripper_flag = false;
          std::cerr << "Failed to operate gripper: " << log << std::endl;
      }
  }

  ~GripperNode()
  {
    if (gripper_thread_.joinable())
    {
      gripper_thread_.join();
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
