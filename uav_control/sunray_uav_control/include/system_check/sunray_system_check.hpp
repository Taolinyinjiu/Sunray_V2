/**
 * @file sunray_system_check.hpp
 * @brief
 * 构造SystemCheck类，作用为检查Sunray框架中各部分，是否正常工作，假设定位模块为A模块，运动规划模块为B模块，只有在A模块正常工作的情况下B模块才能执行运动
 *  SystemCheck依据这种关系，对A模块工作进行检查，然后将结果反馈给B模块，指导B模块是否开始工作
 * 简单来说，由于我们希望将Sunray_FSM与mavros进行解耦，那么对px4参数，定位状态的判断，就交给systam_check模块
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-25
 * @version 0.1
 *
 */

#include <ros/node_handle.h>

class SystemCheck {
    explicit SystemCheck(ros::NodeHandle& nh);
    ~SystemCheck() = default;

  public:
    // 初始化，主要包括
    // 1. 读取参数
    // 2. 构造ros话题的订阅者与发布者
    // 3. 构造ros定时器
    bool Init();

  private:
};
