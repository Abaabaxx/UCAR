/*
 * client.cpp - 讯飞麦克风唤醒词设置程序
 * 主要功能：设置讯飞麦克风的唤醒词
 */

 #include <ros/ros.h>
 #include <std_msgs/String.h>
 #include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
 #include <iostream>
 
 
 int main(int argc, char *argv[])
 {
     // 初始化ROS节点
     ros::init(argc, argv, "client_node");
     ros::NodeHandle nh;
 
     // 创建设置唤醒词的服务客户端
     ros::ServiceClient Set_Awake_Word_client =
     nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");
 
     // 创建设置唤醒词的服务消息对象
     xf_mic_asr_offline::Set_Awake_Word_srv SetAwakeWord_srv;
 
     // 等待其他节点就绪
     sleep(2);
 
     // 设置唤醒词为"小飞小飞"
     SetAwakeWord_srv.request.awake_word = "小飞小飞";

     if(Set_Awake_Word_client.call(SetAwakeWord_srv))
     {
         if(SetAwakeWord_srv.response.result == "ok")
         {
             printf("\n>>>>>Client:设置唤醒词成功！\n");
         }
         else
         {
             printf("\n>>>>>Client:设置唤醒词失败：%s\n", SetAwakeWord_srv.response.fail_reason.c_str());
         }
     }
     else
     {
         printf("\n>>>>>Client:调用设置唤醒词服务失败！\n");
     }
 
     // 保持节点运行
     ros::spin();
 
     return 0;
 }