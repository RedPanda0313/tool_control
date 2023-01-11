#include "tool_control/tool_servo.h"
#include <ros/ros.h>

using namespace Automation::BDaq;


tool_data tool_servo::get_enc(){
    //get encoder value from counter
    read_enc();

    return current_data_rad;
}

void tool_servo::read_enc(){
    ret_ENC0 = udCounterCtrl0->Read(Rotation_value);
    ret_ENC1 = udCounterCtrl1->Read(Linear_value);
    //count+=1;
    uint8 bit_data=0;
    int bit=0;
    //current_data_rad.rotation_data=(double) count*dt*360/5;
    //ret_DI = instantDiCtrl->ReadBit(0, bit, &bit_data);
         //NOTE:

    ret_DI = instantDiCtrl->Read(DI_startPort,DI_portCount,bufferForReading);
      for ( int32 i = DI_startPort;i < DI_startPort+DI_portCount; ++i)
      {
         //printf(" DI port %d status is: 0x%X\n\n", i, bufferForReading);
    //     printf(" DI port %d status is: 0x%x\n", i, bufferForReading[i-DI_startPort]);
        
         //std_msgs::Float64 DI;
         //DI.data =bufferForReading[i-startPort];
         //pub.publish(DI);
      }

      //if(1)
      //printf(" DI port %d status is: 0x%X\n\n", 0, bit_data);
     //printf("\n");  

    current_data_enc.rotation_data=(double)Rotation_value;
    current_data_enc.linear_data=(double)Linear_value;
    current_data_rad.rotation_data=current_data_enc.rotation_data*convert2rad[0];
    current_data_rad.linear_data=(double)current_data_enc.linear_data*stroke;
}
int tool_servo::set_target(tool_data target){
    read_enc();

    //target.linear_data=0.0-target.linear_data;
    /*if(target.linear_data<0){
        target.linear_data=0;
    }
    else if(target.linear_data>1){
        target.linear_data=1;
    }*/
    target_data_enc=target*convert2enc;
    ROS_INFO("target %lf",target_data_enc.rotation_data);
    ROS_INFO("currrent %lf",current_data_enc.rotation_data);
    //target_data_enc.rotation_data=0.0;
    calc_pid();
    write();

    //send to output 
    return 1;
    }

void tool_servo::calc_pid(){
    error0_data=error1_data;
    error1_data=current_data_enc-target_data_enc;
    i_error_data=i_error_data+(error1_data+error0_data)*(dt/2);
    d_error_data=(error1_data-error0_data)*div_dt;
    //kp=0.013;
    //kd=0.00000;
    //ki=0.007;
    target_data_voltage=error1_data*kp+i_error_data*ki+d_error_data*kd;
    //kp=0.6;
    //target_data_voltage=error1_data*kp;
    voltage_limit();

    //target_data_voltage.linear_data=0.0;
    //target_data_voltage.rotation_data=0.0;
    //write();

}
void tool_servo::write(){
    //for(int i=0;i<ONE_WAVE_POINT_COUNT;i++){
    //    rotation_command[i]=target_data_voltage.rotation_data;
    //}
    //target_data_voltage.rotation_data=0.75;

    //ret_AO = instantAoCtrl->Write(LinearChannel_AO,target_data_voltage.linear_data);
    ret_AO = instantAoCtrl->Write(RotationChannel_AO,target_data_voltage.rotation_data);//sing value
    //ret_AO = instantAoCtrl->Write(RotationChannel_AO, 1,rotation_command);//3 wave point [array]

}

void tool_servo::voltage_limit(){
    if(target_data_voltage.rotation_data>vol_limit){
        target_data_voltage.rotation_data=vol_limit;
    printf("%.16lf\n",target_data_voltage.rotation_data);
    }
    else if(target_data_voltage.rotation_data<-vol_limit){
        target_data_voltage.rotation_data=-vol_limit;
    printf("%.16lf\n",target_data_voltage.rotation_data);
    }
    else{
    }
    if(target_data_voltage.linear_data>5){
        target_data_voltage.linear_data=5;
    }
    else if(target_data_voltage.linear_data<-5){
        target_data_voltage.linear_data=-5;
    }
}



int tool_servo::initialize(){
    //init pci boards
    //ao
    #define     ONE_WAVE_POINT_COUNT  3 //define how many data to makeup a waveform period.
    #define     deviceDescription_AO  L"PCI-1727U,BID#0"
    const wchar_t* profilePath_AO = L"pci/DemoDevice.xml";

    #define  deviceDescription_DI  L"PCI-1727U,BID#0"
    
const wchar_t* profilePath_DI = L"pci/DemoDevice.xml";
    //encoder
    #define     deviceDescription_ENC L"PCI-1784,BID#0"
    const wchar_t* profilePath_ENC = L"pci/PCI-1784.xml";
    
    instantAoCtrl = InstantAoCtrl::Create();
    instantDiCtrl = InstantDiCtrl::Create();
   //InstantAoCtrl * instantAoCtrl = AdxInstantAoCtrlCreate();
   //InstantDiCtrl * instantDiCtrl = AdxInstantDiCtrlCreate();
    udCounterCtrl0 = UdCounterCtrl::Create();
    udCounterCtrl1 = UdCounterCtrl::Create();

    

      
      DeviceInformation devInfo_ENC(deviceDescription_ENC);
      ret_ENC0 = udCounterCtrl0->setSelectedDevice(devInfo_ENC);
      if(BioFailed(ret_ENC0))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
      ret_ENC1 = udCounterCtrl1->setSelectedDevice(devInfo_ENC);
      if(BioFailed(ret_ENC1))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
      
    DeviceInformation devInfo_AO(deviceDescription_AO);
      ret_AO = instantAoCtrl->setSelectedDevice(devInfo_AO);
      if(BioFailed(ret_AO)) 
      {ROS_ERROR_STREAM("Error Access PCI-1727 (AO)");return -1;}
    DeviceInformation devInfo_DI(deviceDescription_DI);
    ret_DI = instantDiCtrl->setSelectedDevice(devInfo_DI);  
      if(BioFailed(ret_DI))
      {
         ROS_ERROR_STREAM("Error Access PCI-1727 (DI)");
         return -1;
      }

      ret_AO = instantAoCtrl->LoadProfile(profilePath_AO);//Loads a profile to initialize the device.
       if(BioFailed(ret_AO))
      {ROS_ERROR_STREAM("Error Access PCI-1727");return -1;}
      ret_ENC0 = udCounterCtrl0->LoadProfile(profilePath_ENC);
      if(BioFailed(ret_ENC0))
      {ROS_ERROR_STREAM("Error Access PCI-1727");return -1;}
      ret_ENC1 = udCounterCtrl1->LoadProfile(profilePath_ENC);
      if(BioFailed(ret_ENC1))
      {ROS_ERROR_STREAM("Error Access PCI-1727");return -1;}
      ret_DI = instantDiCtrl->LoadProfile(profilePath_DI);//Loads a profile to initialize the device.
       if(BioFailed(ret_DI))
      {ROS_ERROR_STREAM("Error Access PCI-1727");return -1;}
      
      
      ret_ENC0 = udCounterCtrl0->setChannelStart(RotationChannel_ENC);
       if(BioFailed(ret_ENC0))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
      ret_ENC0 = udCounterCtrl0->setChannelCount(channelCount_ENC);
       if(BioFailed(ret_ENC0))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}

      ret_ENC1 = udCounterCtrl1->setChannelStart(LinearChannel_ENC);
       if(BioFailed(ret_ENC1))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
      ret_ENC1 = udCounterCtrl1->setChannelCount(channelCount_ENC);
       if(BioFailed(ret_ENC1))
      {ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
       Array<UdChannel>*udChannel0 = udCounterCtrl0->getChannels();
        Array<UdChannel>*udChannel1 = udCounterCtrl1->getChannels();

      for(int i = channelStart_ENC; i < channelStart_ENC + channelCount_ENC; i++)
      {
         //ret_ENC = udChannel->getItem(i).setCountingType(PulseDirection);
         ret_ENC0 = udChannel0->getItem(i).setCountingType(AbPhaseX1);
         ret_ENC1 = udChannel1->getItem(i).setCountingType(AbPhaseX1);

         if(BioFailed(ret_ENC0)){ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
         if(BioFailed(ret_ENC1)){ROS_ERROR_STREAM("Error Access PCI-1784");return -1;}
      }

      ret_ENC0= udCounterCtrl0->setEnabled(true);
      ret_ENC1= udCounterCtrl1->setEnabled(true);

      bool enforced = false;

        int32 counter = 0;
        int32 roopcounter = 0;
        int32 phase3_roopcounter = 0;
        target_data_enc.linear_data=0.0;
        return 1;

        
}
int tool_servo::finish(){
    target_data_voltage.linear_data=0.0;
    target_data_voltage.rotation_data=0.0;
    write();

target_data_voltage.rotation_data=0.0;

    ret_AO = instantAoCtrl->Write(LinearChannel_AO,target_data_voltage.linear_data);
    ret_AO = instantAoCtrl->Write(RotationChannel_AO,target_data_voltage.rotation_data);//sing value
   

    ret_ENC0 = udCounterCtrl0->setEnabled(false);
    ret_ENC1 = udCounterCtrl1->setEnabled(false);
    
	instantAoCtrl->Dispose();
	instantDiCtrl->Dispose();
    udCounterCtrl0->Dispose();
    udCounterCtrl1->Dispose();
    if(BioFailed(ret_AO))
   {
      wchar_t enumString[256];
      AdxEnumToString(L"ErrorCode", (int32)ret_AO, 256, enumString);
      printf("Some error occurred. And the last error code is 0x%X. [%ls]\n", ret_AO, enumString);
      return 1;
   }
    if(BioFailed(ret_ENC0))
   {
      wchar_t enumString[256];
      AdxEnumToString(L"ErrorCode", (int32)ret_ENC0, 256, enumString);
      printf("Some error occurred. And the last error code is 0x%X. [%ls]\n", ret_ENC0, enumString);
      return 1;
   };    if(BioFailed(ret_ENC1))
   {
      wchar_t enumString[256];
      AdxEnumToString(L"ErrorCode", (int32)ret_ENC1, 256, enumString);
      printf("Some error occurred. And the last error code is 0x%X. [%ls]\n", ret_ENC1, enumString);
      return 1;
   };
   if(BioFailed(ret_DI))
   {
      wchar_t enumString[256];
      AdxEnumToString(L"ErrorCode", (int32)ret_DI, 256, enumString);
      printf("Some error occurred. And the last error code is 0x%X. [%ls]\n", ret_DI, enumString);
      return 1;
   }
    return 0;

}