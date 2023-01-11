#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <tool_control/tool_servo.h>
#include <mutex>

class ToolHW:public hardware_interface::RobotHW{
public:
    ToolHW();
    void read();
    void write();
  int initialize();
  int finalize();


protected:
 std::mutex m;
  /*hardware_interface::JointStateInterface rotation_JntStInterface;
  hardware_interface::PositionJointInterface rotation_PosJntInterface;
  hardware_interface::JointStateInterface linear_JntStInterface;
  hardware_interface::PositionJointInterface linear_PosJntInterface;
  */
  hardware_interface::JointStateInterface JntStInterface;
  hardware_interface::PositionJointInterface PosJntInterface;
  std::shared_ptr<tool_servo> servo;

  double rot_cmd=0.0;
  double rot_pos=0.0;
  double rot_vel=0.0;
  double rot_eff=0.0;
  double lin_cmd=0.0;
  double lin_pos=0.0;
  double lin_vel=0.0;
  double lin_eff=0.0;
  tool_data cmd_data;


};