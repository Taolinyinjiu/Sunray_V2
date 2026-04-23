#pragma once

#include <ros/ros.h>

class PlanningFSM {
    public:
    // 构造函数与析构函
    PlanningFSM(ros::NodeHandle &nh);
    ~PlanningFSM()=default;

    // 主要的处理函数
    void process();
    // 打印日志到终端
    void printf_terminal();

    private:
    // -----------------------函数------------------------------
    // 判断当前planning模块是否准备好，如果返回为false，则pass或者对false部分进行处理
    bool is_ready();

    // 控制指令回调
    void process_control_cmd();
    // 发布当前状态
    void pub_planning_state();
    // 发布无人机控制指令或无人车控制指令
    void pub_control_cmd();


    // ---------------------变量-----------------------

};