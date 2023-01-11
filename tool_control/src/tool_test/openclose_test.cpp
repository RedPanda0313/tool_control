#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

//send test data(sin wave to evaluate pid gains)

int main(int argc, char* argv[]){
    ros::init(argc,argv,"tool_openclose_test");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float64>("/tool/linear_position_controller/command",10);
    std_msgs::Float64 angle;
    angle.data=0.0;
    int hz=500;
    int loop_time=10;
    double amplitude=1;
    ros::Rate rate(hz);
    int count=0;
    int wait_time=3;
    bool wait=true;
    bool kill_this_process=false;
    while(!kill_this_process&&ros::ok()){
        if(wait){
            if(count>=hz*wait_time){
                wait=false;
                count=0;
            }
        }
        else{
            angle.data=amplitude*(1-cos(2.0*M_PI*count/(hz*loop_time)))/2;
            if(count>=hz*loop_time*3){

                kill_this_process=true;
            }
        }
        pub.publish(angle);
        count++;
        rate.sleep();
    }
    
    return 0;
}