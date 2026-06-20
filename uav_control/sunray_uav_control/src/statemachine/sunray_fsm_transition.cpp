#include "statemachine/sunray_fsm.hpp"
#include <ros/ros.h>

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
    auto command_request_or_control_lost = [this] {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        return has_transition_control_cmd_ || control_msg_lost_;
    };

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
                                            [this] {
                                                control_common::UavControlCmd cmd_snapshot;
                                                {
                                                    std::lock_guard<std::mutex> lk(cmd_mutex_);
                                                    cmd_snapshot = has_transition_control_cmd_
                                                                       ? transition_control_cmd_
                                                                       : last_control_cmd_;
                                                }
                                                double takeoff_height =
                                                    fsm_config_.takeoff_land_param.takeoff_relative_height;
                                                double takeoff_velocity =
                                                    fsm_config_.takeoff_land_param.takeoff_max_velocity;
                                                if (!resolve_takeoff_command_params(
                                                        cmd_snapshot,
                                                        &takeoff_height,
                                                        &takeoff_velocity,
                                                        nullptr)) {
                                                    return false;
                                                }
                                                return check_allow_takeoff(takeoff_height,
                                                                          takeoff_velocity);
                                            },
                                            [this] {
                                                if (!update_home_point()) {
                                                    return false;
                                                }
                                                sunray_controller_->reset_takeoff_status();
                                                return true;
                                            }});

    // TAKEOFF -> HOVER
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::TAKEOFF,
                                            sunray_fsm::SunrayEvent::TAKEOFF_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // TAKEOFF -> LAND，controller 报告起飞失败后进入恢复降落。
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::TAKEOFF,
                                            sunray_fsm::SunrayEvent::TAKEOFF_FAILED,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});

    // TAKEOFF -> LAND，允许起飞异常或未完成时用降落指令中止起飞
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::TAKEOFF,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
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
    // MOVE -> HOVER (HOVER_REQUEST)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::HOVER_REQUEST,
                                            sunray_fsm::SunrayState::HOVER,
                                            command_request_or_control_lost,
                                            [this] { return set_hover_point_from_latest_odom(); }});
    // MOVE -> HOVER (POINT_COMPLETED)
    // 用 move_point 的目标点而不是当前里程计作为悬停点，避免到达判断阶段的位置误差被锁死
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return set_hover_point_to_target_or_odom(); }});

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
bool Sunray_FSM::handle_global_event(const QueuedFsmEvent_t& queued_event) {
    switch (queued_event.event) {
    // 全局最高优先级：紧急锁桨
    // 任意状态收到该事件，都需要进入EMERGENCY_KILL
    case sunray_fsm::SunrayEvent::KILL_REQUEST: {
        std::scoped_lock lock(state_mutex_, cmd_mutex_);
        fsm_state_ = sunray_fsm::SunrayState::EMERGENCY_KILL;
        if (queued_event.has_control_cmd) {
            last_control_cmd_ = queued_event.control_cmd;
        }
        has_transition_control_cmd_ = false;
        return true;
    }
    default:
        // 非全局事件，交给普通状态转移表处理
        return false;
    }
}
// 处理正常情况下的状态转移
bool Sunray_FSM::handle_event(const QueuedFsmEvent_t& queued_event) {
    const auto event = queued_event.event;
    // 首先丢进全局高优先级状态检查
    if (handle_global_event(queued_event)) {
        return true;
    }
    // 打个快照
    sunray_fsm::SunrayState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = fsm_state_;
    }
    // 获取缓存的状态转移表
    const auto& state_table = get_transition_table();
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
        if (queued_event.has_control_cmd) {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            transition_control_cmd_ = queued_event.control_cmd;
            has_transition_control_cmd_ = true;
        }
        auto clear_transition_cmd = [this] {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            has_transition_control_cmd_ = false;
        };
        // 那么运行到这里，就应当得到了针对当前状态和事件的转移规则
        // 这里分为两个方面，一个是判断是否允许转移，一个是转移后执行的函数
        const bool allow_checkout_state = (t.guard ? t.guard() : true);
        if (!allow_checkout_state) {
            clear_transition_cmd();
            ROS_WARN_STREAM("[Sunray_FSM] command rejected by current controller state, event="
                            << static_cast<int>(event));
            return false;
        }
        const bool action_ok = (t.action ? t.action() : true);
        if (!action_ok) {
            clear_transition_cmd();
            ROS_WARN_STREAM("[Sunray_FSM] transition action failed, event="
                            << static_cast<int>(event)
                            << ", state=" << static_cast<int>(current_state)
                            << ", target=" << static_cast<int>(t.transmit_state));
            return false;
        }
        {
            std::scoped_lock lock(state_mutex_, cmd_mutex_);
            fsm_state_ = t.transmit_state;
            if (queued_event.has_control_cmd) {
                last_control_cmd_ = queued_event.control_cmd;
            }
            has_transition_control_cmd_ = false;
        }
        return true;
    }
    switch (event) {
    case sunray_fsm::SunrayEvent::TAKEOFF_REQUEST:
    case sunray_fsm::SunrayEvent::LAND_REQUEST:
    case sunray_fsm::SunrayEvent::RETURN_REQUEST:
    case sunray_fsm::SunrayEvent::POINT_REQUEST:
    case sunray_fsm::SunrayEvent::VELOCITY_REQUEST:
    case sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST:
        ROS_WARN_STREAM("[Sunray_FSM] command not allowed in current FSM state, event="
                        << static_cast<int>(event)
                        << ", state=" << static_cast<int>(current_state));
        break;
    default:
        break;
    }
    return false;
}

// 向状态事件队列中传入事件
// note:如果传入的事件与队列中最新的事件相同，则不会传入
void Sunray_FSM::enqueue_fsm_event(sunray_fsm::SunrayEvent input_event) {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (!fsm_event_queue_.empty() && fsm_event_queue_.back().event == input_event) {
        // 如果事件队列非空，并且传入的与队列中最新的是一致的，那么就return
        return;
    }
    fsm_event_queue_.push(QueuedFsmEvent_t{input_event, false, control_common::UavControlCmd{}});
}

void Sunray_FSM::enqueue_fsm_event(sunray_fsm::SunrayEvent input_event,
                                   const control_common::UavControlCmd& control_cmd) {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (!fsm_event_queue_.empty() && fsm_event_queue_.back().event == input_event) {
        fsm_event_queue_.back().has_control_cmd = true;
        fsm_event_queue_.back().control_cmd = control_cmd;
        return;
    }
    fsm_event_queue_.push(QueuedFsmEvent_t{input_event, true, control_cmd});
}
// 处理状态机事件中的事件
void Sunray_FSM::process_fsm_event_queue() {
    QueuedFsmEvent_t queued_event;
    {
        std::lock_guard<std::mutex> lk(event_mutex_);
        if (fsm_event_queue_.empty()) {
            return;
        }
        queued_event = fsm_event_queue_.front();
        fsm_event_queue_.pop();
    }
    handle_event(queued_event);
}
