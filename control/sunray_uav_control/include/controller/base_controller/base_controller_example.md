在构造Base_Controller之前，我们应该先写出Base_Controller的使用接口，或者说使用场景
1. 控制器向状态机中进行注册，状态机保留指向控制器的指针

// 基类控制器实例化
auto controller = std::make_shared<Base_Controller>();
// 向状态机中注册控制器
Sunray_fsm.register_controller(controller);
// 仲裁器实例化
Sunray_Control_Arbiter arbiter;

-----------以下为Sunray状态机中代码-------------------
2. 状态机OFF阶段,控制器停止输出
controller->set_currentstate(current_state); // 注入里程计信息
controller->set_mode(Base_Controller_Stage::GROUND);
controller->stop();

3. 状态机进入起飞阶段

controller->set_currenstate(current_state); // 注入里程计信息

// 如果上个状态是OFF，则切换控制器模式为起飞
if(last_state == fsm_state::OFF)
        controller->set_mode(Base_Controller_Stage::ARM);
// 如果上个状态是TAKEOFF，则先检查控制器是否完成解锁，再检查是否完成起飞
else if (last_state == fsm_state::TAKEOFF)
        arm_success = controller->has_arm();
        if(arm_success)
        {
                //在解锁成功的基础上检查是否起飞成功
                takeoff_success = controller->has_takeoff()
                if(takeoff_success == true) // 如果起飞成功(满足控制器参数容差)
                        // 状态机切换到下一个状态 也就是Hover
        }
// 得到控制器的输出
controller_output = controller->update();
// 将输出给到仲裁层(由仲裁层向mavros发布)
arbiter.update(controller_output);

//
由于我们将px4的读取与写入分离，通过不向外提供与控制强相关的封装接口，因此有关px4的模式切换需要在状态机中进行实现
// 首先需要切换到OFFBOARD模式，条件为10Hz的控制指令+手动切换
// 控制指令由 arbiter + controller 保证，在未解锁时，输出为期望推力 0

// 检查当前px4 模式
// 如果当前px4的模式为位置控制模式，说明当前是由OFF阶段切换到TAKEOFF阶段
if(px4_flightmode == px4_data_types::FlightMode::kPosctl)
{
        // 尝试切换到OFFBOARD
        set_px4_mode(px4_data_types::FlightMode::kOffboard);
}
else if (px4_flightmode == px4_data_types::FlightMode::kOffboard)
{
        // 如果已经切换到了OFFBOARD模式
        if(arm_success == false) // 如果没有解锁，就尝试进行解锁
                set_px4_arm(true); // false为上锁
}

4. 状态机进入悬停状态

controller->set_currentstate(current_state); // 注入里程计信息
controller->set_mode(Base_Controller_Stage::HOVER)
if(last_state == fsm_state::TAKEOFF) // 如果上一个状态是起飞
// 在这里，设置悬停点为参数配置的xyz
        controller->set_desiredstate(takeofftarget_state);
else if (last_state == 其他状态)
        controller->set_desiredstate(current_state); // 在当前位置悬停
// 状态切换，根据当前触发的切换事件来进行，假设没有，则应该保持悬停

5. 其他状态
controller->set_mode(Base_Controller_Stage::MOVE)








and then我们现在来修改另一个问题，控制器期望什么样的数据