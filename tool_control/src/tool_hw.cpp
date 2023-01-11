#include "tool_control/tool_hw.h"
#include <string>

ToolHW::ToolHW(){
}
int ToolHW::initialize(){

    //cmd_data.linear_data=1.0;
    std::string name ="joint_7"  ;
    hardware_interface::JointStateHandle rot_state(name,&rot_pos,&rot_vel,&rot_eff);
    //rotation_JntStInterface.registerHandle(rot_state);
    JntStInterface.registerHandle(rot_state);
    //hardware_interface::JointHandle rot_handle(rotation_JntStInterface.getHandle(name),&cmd_data.rotation_data);
    hardware_interface::JointHandle rot_handle(JntStInterface.getHandle(name),&cmd_data.rotation_data);
    //rotation_PosJntInterface.registerHandle(rot_handle);
    PosJntInterface.registerHandle(rot_handle);
    name ="openclose"  ;
    hardware_interface::JointStateHandle lin_state(name,&lin_pos,&lin_vel,&lin_eff);
    //linear_JntStInterface.registerHandle(lin_state);
    JntStInterface.registerHandle(lin_state);
    //hardware_interface::JointHandle lin_handle(linear_JntStInterface.getHandle(name),&cmd_data.linear_data);
    hardware_interface::JointHandle lin_handle(JntStInterface.getHandle(name),&cmd_data.linear_data);
    PosJntInterface.registerHandle(lin_handle);

    servo=std::shared_ptr<tool_servo>(new tool_servo);
    int ret=servo->initialize();



   /*registerInterface(&linear_JntStInterface);
    registerInterface(&linear_PosJntInterface);

    registerInterface(&rotation_JntStInterface);
    registerInterface(&rotation_PosJntInterface);*/
    registerInterface(&JntStInterface);
    registerInterface(&PosJntInterface);
    //ROS_INFO("here");
    cmd_data.linear_data=0.0;
    return ret;

}

void ToolHW::read(){
    std::lock_guard<std::mutex> lock(m);
    rot_pos=servo->get_enc().rotation_data;
}
void ToolHW::write(){
    std::lock_guard<std::mutex> lock(m);
    //cmd_data.rotation_data*=180;
    //cmd_data.linear_data=1.0-cmd_data.linear_data;
    servo->set_target(cmd_data);
}
int ToolHW::finalize(){
    std::lock_guard<std::mutex> lock(m);
    return servo->finish();
    
}