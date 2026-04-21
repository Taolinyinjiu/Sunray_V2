#include "statemachine/sunray_fsm.hpp"

// OFF -> INIT -> TAKEOFF -> HOVER -> MOVE
// |                 |         |        |
// LAND  <- - -  - - - - - - - | - - -  |
//                             |        |
// RETUAN <- - - - - - - - - - - - - -  |
//
// LAND / RETURN　-> INIT
// note：这里并没有处理emergency_kill相关状态，因为我们在设计的时候认为emergency_kill是一个比较危险的状态，主要表现在空中停机等一系列措施，
// 当需要进入emergency状态时，我们需要思考的是如何减少无人机的损伤，以及如何避免对周围的伤害，不重启/检查无人机就立即复飞是一个不太可能的情况
void Sunray_FSM::init_transition_table() {
    // 首先，检查表是否为空，如果非空则返回，避免重复初始化
    if (!sunray_state_transmit_table_.empty())
        return;
    // always是一个lambda表达式，用于表示，这一项不需要进行检查or执行
    auto always = [] { return true; };

    // struct Transition {
    //     sunray_fsm::SunrayState current_state;   // 状态机当前状态
    //     sunray_fsm::SunrayEvent event;           // 发生的事件(control_cmd)
    //     sunray_fsm::SunrayState transmit_state;  // 状态机要转移到的状态
    //     std::function<bool()> guard;             // 判断是否允许转移
    //     std::function<bool()> action;            // 转移成功后执行的命令
    // };

    // OFF -> INIT,要求控制器准备就绪
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::OFF,
                                            sunray_fsm::SunrayEvent::CONTROLLER_READY,
                                            sunray_fsm::SunrayState::INIT,
                                            [this] { return controller_ready_; },
                                            always});

    // INIT -> TAKEOFF，要求takeoff标识符置位true
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::INIT,
                                            sunray_fsm::SunrayEvent::TAKEOFF_REQUEST,
                                            sunray_fsm::SunrayState::TAKEOFF,
                                            [this] { return allow_takeoff_; },
                                            [this] { return update_home_point(); }});

    // TAKEOFF -> HOVER
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::TAKEOFF,
                                            sunray_fsm::SunrayEvent::TAKEOFF_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // HOVER -> MOVE (POINT)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::POINT_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (POINT)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});
    // MOVE -> HOVER (HOVER_REQUEST)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::HOVER_REQUEST,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});
    // MOVE -> HOVER (POINT_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});

    // MOVE -> HOVER (VELOCITY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});

    // MOVE -> HOVER (TRAJECTORY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});

    // MOVE -> HOVER (POINT_WGS84_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});
    // HOVER -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            always,
                                            always});

    // MOVE -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            always,
                                            always});
    // HOVER-> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // MOVE -> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // RETURN -> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // RETURN -> HOVER
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::HOVER_REQUEST,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});
    // RETURN -> HOVER
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::RETURN_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_from_latest_odom(); }});
    // LAND -> READY
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::LAND,
                                            sunray_fsm::SunrayEvent::LAND_COMPLETED,
                                            sunray_fsm::SunrayState::INIT,
                                            always,
                                            always});
}

const std::vector<sunray_fsm::Transition>& Sunray_FSM::get_transition_table() {
    // 如果状态转移表没有初始化(表容器为空)，则进行一次初始化
    if (sunray_state_transmit_table_.empty()) {
        init_transition_table();
    }
    // 返回表容器，类成员sunray_state_transmit_table_
    return sunray_state_transmit_table_;
}
// 处理全局高优先级状态
bool Sunray_FSM::handle_global_event(sunray_fsm::SunrayEvent event) {
    switch (event) {
    // 全局最高优先级：紧急锁桨
    // 任意状态收到该事件，都需要进入EMERGENCY_KILL
    case sunray_fsm::SunrayEvent::KILL_REQUEST: {
        // 切换状态为
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            fsm_state_ = sunray_fsm::SunrayState::EMERGENCY_KILL;
        }
        return true;
    }
    default:
        // 非全局事件，交给普通状态转移表处理
        return false;
    }
}
// 处理正常情况下的状态转移
bool Sunray_FSM::handle_event(sunray_fsm::SunrayEvent event) {
    // 首先丢进全局高优先级状态检查
    if (handle_global_event(event)) {
        return true;
    }
    // handle_event() -> false 说明不是kill状态，按照正常流程往下走
    // 打个快照
    sunray_fsm::SunrayState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = fsm_state_;
    }
    // 获取缓存的状态转移表
    const auto state_table = get_transition_table();
    // 使用迭代器 查找匹配字段
    for (const auto& t : state_table) {
        // 如果当前迭代器找到的状态转移规则不是针对当前状态的，则跳过
        if (t.current_state != current_state) {
            continue;
        }
        // 如果当前迭代器找到的状态转移规则不是针对当前事件的，跳过
        if (t.event != event) {
            continue;
        }
        // 那么运行到这里，就应当得到了针对当前状态和事件的转移规则
        // 这里分为两个方面，一个是判断是否允许转移，一个是转移后执行的函数
        // 首先判断是否允许转移
        // (t.guard ? t.guard() : true) -> 如果t.guard存在，则返回t.guard()的结果
        //                                 如果t.guard不存在，则直接返回true
        const bool allow_checkout_state = (t.guard ? t.guard() : true);
        if (!allow_checkout_state) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            fsm_state_ = t.transmit_state;
        }
        // (t.action ? t.action() : true) -> 如果t.action存在，则执行t.action()
        //                                   如果t.action不存在，则直接返回true
        const bool need_action = (t.action ? t.action() : true);
        // 但其是我们不关心action的结果，因为写到这里的时候，我意识到handle_event其实是用作controlcmd的回调和运动判断完成的回调，而不是一个高频更新的函数
        //  因此这里的action并不能够与无人机的运动相关联
        //  但是我认为保留这一项可能有一些好处，比如action可以作为日志，存储每一次状态切换的细节
        (void)need_action;
        return true;
    }
    return false;
}

// 向状态事件队列中传入事件
// note:如果传入的事件与队列中最新的事件相同，则不会传入
void Sunray_FSM::enqueue_fsm_event(sunray_fsm::SunrayEvent input_event) {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (!fsm_event_queue_.empty() && fsm_event_queue_.back() == input_event) {
        // 如果事件队列非空，并且传入的与队列中最新的是一致的，那么就return
        return;
    }
    fsm_event_queue_.push(input_event);
}
// 处理状态机事件中的事件
void Sunray_FSM::process_fsm_event_queue() {
    sunray_fsm::SunrayEvent event;
    {
        std::lock_guard<std::mutex> lk(event_mutex_);
        if (fsm_event_queue_.empty()) {
            return;
        }
        event = fsm_event_queue_.front();
        fsm_event_queue_.pop();
    }
    handle_event(event);
}
