#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "tool_control/tool_hw.h"

int main(int argc, char* argv[]){
    ros::init(argc,argv,"tool_control");
    ros::NodeHandle nh;
    ToolHW tool;
    int ret =tool.initialize();
    if(ret==1){
        controller_manager::ControllerManager cm(&tool,nh);
        ros::Rate rate(2000); // 2000[Hz]
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::Time t = ros::Time::now();
        while(ros::ok()){
            ros::Duration d = ros::Time::now() - t;
            t = ros::Time::now();
            tool.read();
            cm.update(t,d);
            tool.write();
            rate.sleep();
        }
        tool.finalize();
    spinner.stop();
    }
    return 0;
}