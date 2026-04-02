#include "statemachine/sunray_fsm.hpp"
#include <stdexcept>

// 构造函数
Sunray_FSM::Sunray_FSM(ros::NodeHandle& nh) {
    nh_ = nh;
};

void Sunray_FSM::init() {
    // 请注意init中所有函数均为无返回类型
    load_param();
    init_publisher();
    init_subscriber();
    register_controller();

    // 初始化状态转移表
    init_transition_table();
}
