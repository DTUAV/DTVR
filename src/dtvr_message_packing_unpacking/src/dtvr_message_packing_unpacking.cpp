#include "../include/dtvr_message_packing_unpacking/dtvr_message_packing_unpacking.h"

dtvr_message_packing_unpacking::dtvr_message_packing_unpacking()
{
  ros::NodeHandle n("~");
  if(!n.getParam("SourceID",_sourceID))
  {
    _sourceID = 0;
    std::cout<<"message_packing_unpacking--SourceID No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--SourceID: "<<_sourceID<<std::endl;

  if(!n.getParam("TargetID",_targetID))
  {
    _targetID = 100;
    std::cout<<"message_packing_unpacking--TargetID No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--TargetID: "<<_targetID<<std::endl;

  if(!n.getParam("DtObjectID",_dtObjectID))
  {
    _dtObjectID = 100;
    std::cout<<"message_packing_unpacking--DtObjectID No Configure"<<_dtObjectID<<std::endl;
  }
  std::cout<<"message_packing_unpacking--DtObjectID: "<<_dtObjectID<<std::endl;

  std::string CloudMsgSubTopic = "/R_UAV_0/Message_from_Cloud";
  if(!n.getParam("CloudMessageSubTopic",CloudMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--CloudMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--CloudMessageSubTopic: "<<CloudMsgSubTopic<<std::endl;

  std::string GlobalPosMsgSubTopic = "/mavros/global_position/global";

  if(!n.getParam("GlobalPositionMessageSubTopic",GlobalPosMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--GlobalPositionMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--GlobalPositionMessageSubTopic: "<<GlobalPosMsgSubTopic<<std::endl;

  std::string LocalPosMsgSubTopic = "/mavros/local_position/pose";
  if(!n.getParam("LocalPositionMessageSubTopic",LocalPosMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--LocalPositionMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--LocalPositionMessageSubTopic: "<<LocalPosMsgSubTopic<<std::endl;

  std::string LocalVelMsgSubTopic = "/mavros/local_position/velocity";
  if(!n.getParam("LocalVelocityMessageSubTopic",LocalVelMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--LocalVelocityMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--LocalVelocityMessageSubTopic: "<<LocalVelMsgSubTopic<<std::endl;

  std::string UavStateMsgSubTopic = "/mavros/state";
  if(!n.getParam("UavStateMessageSubTopic",UavStateMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--UavStateMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--UavStateMessageSubTopic: "<<UavStateMsgSubTopic<<std::endl;

  std::string BatteryMsgSubTopic = "/mavros/battery";
  if(!n.getParam("BatteryMessageSubTopic",BatteryMsgSubTopic))
  {
    std::cout<<"message_packing_unpacking--BatteryMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--BatteryMessageSubTopic: "<<BatteryMsgSubTopic<<std::endl;

  if(!n.getParam("MessagePubFrequency",_msgPubHz))
  {
    _msgPubHz = 10;
    std::cout<<"message_packing_unpacking--MessagePubFrequency No Configure"<<_dtObjectID<<std::endl;
  }
  std::cout<<"message_packing_unpacking--MessagePubFrequency: "<<_msgPubHz<<std::endl;

  _local_pos_sub = n.subscribe(LocalPosMsgSubTopic,1,&dtvr_message_packing_unpacking::local_pos_sub_cb,this);
  _global_pos_sub = n.subscribe(GlobalPosMsgSubTopic,1,&dtvr_message_packing_unpacking::global_pos_sub_cb,this);
  _battery_info_sub = n.subscribe(BatteryMsgSubTopic,1,&dtvr_message_packing_unpacking::battery_info_sub_cb,this);
  _state_info_sub = n.subscribe(UavStateMsgSubTopic,1,&dtvr_message_packing_unpacking::state_info_sub_cb,this);
  _vel_info_sub = n.subscribe(LocalVelMsgSubTopic,1,&dtvr_message_packing_unpacking::vel_info_sub_cb,this);
  _cloud_msg_sub = n.subscribe(CloudMsgSubTopic,1,&dtvr_message_packing_unpacking::cloud_msg_cb,this);

  std::string CloudMsgPubTopic = "/R_UAV_0/Message_to_Cloud";
  if(!n.getParam("CloudMessagePubTopic",CloudMsgPubTopic))
  {
    std::cout<<"message_packing_unpacking--CloudMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--CloudMessagePubTopic: "<<CloudMsgPubTopic<<std::endl;

  std::string TargetVelPubTopic = "/R_UAV_0/Target/Velocity";
  if(!n.getParam("TargetVelocityMessagePubTopic",TargetVelPubTopic))
  {
    std::cout<<"message_packing_unpacking--TargetVelocityMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--TargetVelocityMessagePubTopic: "<<TargetVelPubTopic<<std::endl;

  std::string TargetPosPubTopic = "/R_UAV_0/Target/Position";
  if(!n.getParam("TargetPositionMessagePubTopic",TargetPosPubTopic))
  {
    std::cout<<"message_packing_unpacking--TargetPositionMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--TargetPositionMessagePubTopic: "<<TargetPosPubTopic<<std::endl;

  std::string ArmComPubTopic = "/R_UAV_0/Set/Arm";
  if(!n.getParam("ArmCommandMessagePubTopic",ArmComPubTopic))
  {
    std::cout<<"message_packing_unpacking--ArmCommandMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--ArmCommandMessagePubTopic: "<<ArmComPubTopic<<std::endl;

  std::string TargetFModePubTopic = "/R_UAV_0/Set/FMode";
  if(!n.getParam("TargetFModeMessagePubTopic",TargetFModePubTopic))
  {
    std::cout<<"message_packing_unpacking--TargetFModeMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--TargetFModeMessagePubTopic: "<<ArmComPubTopic<<std::endl;

  std::string VRControlPubTopic = "/R_UAV_0/VR/Control";
  if(!n.getParam("VRControlMessagePubTopic",VRControlPubTopic))
  {
    std::cout<<"message_packing_unpacking--VRControlPubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--VRControlPubTopic: "<<VRControlPubTopic<<std::endl;

  std::string ComputerCmdPubTopic = "/R_UAV_0/Computer/Cmd";
  if(!n.getParam("ComputerCmdMessagePubTopic",ComputerCmdPubTopic))
  {
    std::cout<<"message_packing_unpacking--ComputerCmdMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--ComputerCmdMessagePubTopic: "<<ComputerCmdPubTopic<<std::endl;

  std::string ApplyCamPubTopic = "/R_UAV_0/Apply/Camera";
  if(!n.getParam("ApplyCameraMessagePubTopic",ApplyCamPubTopic))
  {
    std::cout<<"message_packing_unpacking--ApplyCameraMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_packing_unpacking--ApplyCameraMessagePubTopic: "<<ApplyCamPubTopic<<std::endl;
  _cloud_msg_pub= n.advertise<dt_message_package::CloudMessage>(CloudMsgPubTopic,1);
  _target_pos_pub = n.advertise<geometry_msgs::PoseStamped>(TargetPosPubTopic,1);
  _target_vel_pub = n.advertise<geometry_msgs::TwistStamped>(TargetVelPubTopic,1);
  _arm_com_pub = n.advertise<std_msgs::Bool>(ArmComPubTopic,1);
  _target_fmode_pub = n.advertise<std_msgs::Int8>(TargetFModePubTopic,1);
  _vr_control_pub = n.advertise<std_msgs::Bool>(VRControlPubTopic,1);
  _computer_cmd_pub = n.advertise<std_msgs::Bool>(ComputerCmdPubTopic,1);
  _apply_cam_pub = n.advertise<std_msgs::Bool>(ApplyCamPubTopic,1);

  int flag_thread = pthread_create(&_runThread,NULL,&dtvr_message_packing_unpacking::run,this);
  if (flag_thread < 0) {
    ROS_ERROR("message_packing_unpacking--pthread_create ros_process_thread failed: %d\n", flag_thread);
  }
}

void *dtvr_message_packing_unpacking::run(void *args)
{
  dtvr_message_packing_unpacking* dtvrPtr = (dtvr_message_packing_unpacking*)(args);
  ros::Rate rate(dtvrPtr->_msgPubHz);
  dt_message_package::CloudMessage pubMsg;
  pubMsg.SourceID = dtvrPtr->_sourceID;
  pubMsg.TargetID = dtvrPtr->_targetID;
  pubMsg.MessageID = UavInfoID;
  UavInfo info;
  while(ros::ok())
  {
    pubMsg.TimeStamp = ros::Time::now().toNSec();
    {
      std::lock_guard<mutex> guard(dtvrPtr->m);
      info.AVelX = dtvrPtr->_aVelX;
      info.AVelY = dtvrPtr->_aVelY;
      info.AVelZ = dtvrPtr->_aVelZ;
      info.FMode = dtvrPtr->_fMode;
      info.IsArm = dtvrPtr->_isArm;
      info.LVelX = dtvrPtr->_lVelX;
      info.LVelY = dtvrPtr->_lVelY;
      info.LVelZ = dtvrPtr->_lVelZ;
      info.NetPx4 = dtvrPtr->_netPx4;
      info.PosX = dtvrPtr->_lposX;
      info.PosY = dtvrPtr->_lposY;
      info.PosZ = dtvrPtr->_lposZ;
      info.Remaining = dtvrPtr->_remaining;
      info.RotW = dtvrPtr->_rotW;
      info.RotX = dtvrPtr->_rotX;
      info.RotY = dtvrPtr->_rotY;
      info.RotZ = dtvrPtr->_rotZ;
      info.Voltage = dtvrPtr->_voltage;
    }
    pubMsg.MessageData = x2struct::X::tojson(info);
    dtvrPtr->_cloud_msg_pub.publish(pubMsg);
    rate.sleep();
  }
  pthread_join(dtvrPtr->_runThread,NULL);
}

void dtvr_message_packing_unpacking::vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _lVelX = msg.get()->twist.linear.x;
  _lVelY = msg.get()->twist.linear.y;
  _lVelZ = msg.get()->twist.linear.z;
  _aVelX = msg.get()->twist.angular.x;
  _aVelY = msg.get()->twist.angular.y;
  _aVelZ = msg.get()->twist.angular.z;
}

void dtvr_message_packing_unpacking::local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _lposX = msg.get()->pose.position.x;
  _lposY = msg.get()->pose.position.y;
  _lposZ = msg.get()->pose.position.z;
  _rotX = msg.get()->pose.orientation.x;
  _rotY = msg.get()->pose.orientation.y;
  _rotZ = msg.get()->pose.orientation.z;
  _rotW = msg.get()->pose.orientation.w;
}

void dtvr_message_packing_unpacking::global_pos_sub_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  _latitude = msg.get()->latitude;
  _longitude = msg.get()->longitude;
  _altitude = msg.get()->altitude;
}

void dtvr_message_packing_unpacking::battery_info_sub_cb(const mavros_msgs::BatteryStatusConstPtr &msg)
{
  _voltage = msg.get()->voltage;
  _remaining = msg.get()->remaining;
}

void dtvr_message_packing_unpacking::state_info_sub_cb(const mavros_msgs::StateConstPtr &msg)
{
  if(msg.get()->connected)
    _netPx4 = true;
  else
    _netPx4 = false;
  if(msg.get()->armed)
    _isArm = true;
  else
    _isArm = false;
  std::string mode = msg.get()->mode;
  if(mode == msg.get()->MODE_PX4_MANUAL)
    _fMode = 0;
  else if(mode == msg.get()->MODE_PX4_STABILIZED)
    _fMode = 1;
  else if(mode == msg.get()->MODE_PX4_ALTITUDE)
    _fMode = 2;
  else if(mode == msg.get()->MODE_PX4_POSITION)
    _fMode = 3;
  else if(mode == msg.get()->MODE_PX4_OFFBOARD)
    _fMode = 4;
  else if(mode == msg.get()->MODE_PX4_RTL)
    _fMode = 5;
  else
    _fMode = -1;
}

void dtvr_message_packing_unpacking::cloud_msg_cb(const dt_message_package::CloudMessageConstPtr &msg)
{
  switch (msg.get()->MessageID) {
  case UavControlID:
  {
    UavControl uavControlMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavControlMsg,false);
    if(isLoad)
    {
      if(uavControlMsg.Mode==0)
      {
        //position mode
        geometry_msgs::PoseStamped targetPosMsg;
        targetPosMsg.header.stamp = ros::Time::now();
        targetPosMsg.pose.position.x = uavControlMsg.ComLX;
        targetPosMsg.pose.position.y = uavControlMsg.ComLY;
        targetPosMsg.pose.position.z = uavControlMsg.ComLZ;
        tf::Quaternion qua = tf::createQuaternionFromRPY(uavControlMsg.ComAX,uavControlMsg.ComAY,uavControlMsg.ComAZ);
        targetPosMsg.pose.orientation.x = qua.x();
        targetPosMsg.pose.orientation.y = qua.y();
        targetPosMsg.pose.orientation.z = qua.z();
        targetPosMsg.pose.orientation.w = qua.w();
        _target_pos_pub.publish(targetPosMsg);
      }
      else if(uavControlMsg.Mode==1)
      {
        //velocity mode
        geometry_msgs::TwistStamped targetVelMsg;
        targetVelMsg.header.stamp = ros::Time::now();
        targetVelMsg.twist.linear.x = uavControlMsg.ComLX;
        targetVelMsg.twist.linear.y = uavControlMsg.ComLY;
        targetVelMsg.twist.linear.z = uavControlMsg.ComLZ;
        targetVelMsg.twist.angular.x = uavControlMsg.ComAX;
        targetVelMsg.twist.angular.y = uavControlMsg.ComAY;
        targetVelMsg.twist.angular.z = uavControlMsg.ComAZ;
        _target_vel_pub.publish(targetVelMsg);
      }
    }
    else
      ROS_INFO("Unpack UavControl Message Fail!!!");
  }
    break;
  case UavCommandID:
  {
    UavCommand uavCommandMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavCommandMsg,false);
    if(isLoad)
    {
      if(uavCommandMsg.ComMode == 1)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);
      }
      else if(uavCommandMsg.ComMode == 2)
      {
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }
      else if (uavCommandMsg.ComMode == 3)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }
      else if(uavCommandMsg.ComMode == 4)
      {
        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }
      else if(uavCommandMsg.ComMode == 5)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);

        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }

      else if(uavCommandMsg.ComMode == 6)
      {
        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);

        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }

      else if(uavCommandMsg.ComMode == 7)
      {
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);

        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);

        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }
    }
    else
      ROS_INFO("Unpack UavCommand Message Fail!!!");

  }
    break;
  case ComputerControlID:
  {
    ComputerControl computerControlMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,computerControlMsg,false);
    if(isLoad)
    {
      std_msgs::Bool computerCmdMsg;
      computerCmdMsg.data = computerControlMsg.IsClose;
      _computer_cmd_pub.publish(computerCmdMsg);
    }
  }
    break;
  case ApplyCameraID:
  {
    ApplyCameraMsg applyCamMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,applyCamMsg,false);
    if(isLoad)
    {
      std_msgs::Bool apply;
      apply.data = applyCamMsg.isOpen;
      _apply_cam_pub.publish(apply);
    }
  }
    break;
  default:
    break;
  }
}














