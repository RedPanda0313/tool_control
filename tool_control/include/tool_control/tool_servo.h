#pragma once
#include <stdlib.h>
#include <cmath>
#include "pci/compatibility.h"
#include "pci/bdaqctrl.h"
#include <iostream>

#define     ONE_WAVE_POINT_COUNT  3 //define how many data to makeup a waveform period.

typedef unsigned char byte;
struct tool_data{
    double rotation_data=0.0;
    double linear_data=0.0;
    tool_data operator+(const tool_data &other){
        tool_data ret;
        ret.rotation_data=rotation_data+other.rotation_data;
        ret.linear_data=linear_data+other.linear_data;
        return ret;
    }
    tool_data operator-(const tool_data &other){
        tool_data ret;
        ret.rotation_data=rotation_data-other.rotation_data;
        ret.linear_data=linear_data-other.linear_data;
        return ret;
    }
    tool_data operator*(double other){
        tool_data ret;
        ret.rotation_data=rotation_data*other;
        ret.linear_data=linear_data*other;
        return ret;
    }
    tool_data operator/(double other){
        tool_data ret;
        ret.rotation_data=rotation_data/other;
        ret.linear_data=linear_data/other;
        return ret;
    }
    tool_data operator*(double other[2]){
        tool_data ret;
        ret.rotation_data=rotation_data*other[0];
        ret.linear_data=linear_data*other[1];
        return ret;
    }
    tool_data operator/(double other[2]){
        tool_data ret;
        ret.rotation_data=rotation_data/other[0];
        ret.linear_data=linear_data/other[1];
        return ret;
    }
};

using namespace Automation::BDaq;
class tool_servo{
    public:
        tool_servo(){};
        int initialize();
        int finish();
        tool_data get_enc();
        int set_target(tool_data target);
        void write();
    protected:
        void read_enc();
        double test=10.0;
        tool_data current_data_rad;
        tool_data current_data_enc;
        tool_data target_data_enc;
        tool_data target_data_voltage;
        tool_data error0_data;
        tool_data error1_data;
        tool_data i_error_data;
        tool_data d_error_data;
        double rotation_command[3]={0.0,0.0,0.0};

        int32 Rotation_value;
        int32 Linear_value;
        void calc_pid();
        void voltage_limit();
        //temp 
        double count=0.0;

    private:
    //setting
        float vol_limit=10.0;
        double resolution=8192.0;
        double stroke=0.0;//linear motor stroke
        double hz=1000;
        double div_dt=(double)hz;
        double dt=1.0/hz;
        double convert2rad[2]={M_PI*2/8192,60.0/(40*31000)};//convert encoder  to rad and mm stroke or open angle (TBD)
        double convert2enc[2]={M_1_PI*0.5*8192.0,31000.0*40/60};//convert encoder  to rad and mm stroke or open angle (TBD)
        //const double kp=0.0075; //p value no vibration
        const double kc=0.1;//0.02 worked for point to point, 0.1 good for sin wave 
        const double tc=0.08;//1.0 worked
        //double kp=kc*0.6;
        //double ki=kp/(0.5*tc);
        //double kd=kp*(0.125* tc);
        double kp=0.007;
        double ki=0.1;
        double kd=0.0;

    private:
    ErrorCode ret_AO = Automation::BDaq::Success;
    ErrorCode ret_ENC0 = Automation::BDaq::Success;
    ErrorCode ret_ENC1 = Automation::BDaq::Success;
    ErrorCode ret_DI = Automation::BDaq::Success;
    InstantAoCtrl * instantAoCtrl; 
    InstantDiCtrl * instantDiCtrl; 
    UdCounterCtrl* udCounterCtrl0; 
    UdCounterCtrl* udCounterCtrl1; 
    int32       channelStart_ENC = 0;
    int32       channelCount_ENC = 1;
    int32       RotationChannel_ENC = 0;
    int32       LinearChannel_ENC = 1;
    int32       channelStart_AO = 0;
    int32       channelCount_AO = 2;
    int32       onechannelCount_AO = 1;
    int32       RotationChannel_AO = 0;
    int32       LinearChannel_AO = 1;


    int32       DI_startPort = 0;
    int32       DI_portCount = 3;

    byte  bufferForReading[64] = {0};//the first element of this array is used for start port
   int Store[64] = {255};
};
