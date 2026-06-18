// NOTICE：本文件描述的数据类型会变成ulog日志中的数据字段
#pragma once

#include <cstdint>
#include <vector>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace controller_data_types {

struct GeometricControllerParamRecord {
    uint64_t timestamp = 0;
    uint8_t enable_ulog = 0;
    uint8_t control_type = 0;
    uint8_t hover_thrust_estimator_type = 0;
    float mass_kg = 0.0f;
    float gravity = 0.0f;
    float hover_thrust_init = 0.0f;
    float hover_thrust_min = 0.0f;
    float hover_thrust_max = 0.0f;
    float accepted_hover_thrust_rate = 0.0f;
    float max_acc = 0.0f;
    float max_d_acc = 0.0f;
    float attitude_tau = 0.0f;
    float zb_z_min = 0.0f;
    float controller_hz = 0.0f;
    float pos_kp[3] = {0.0f, 0.0f, 0.0f};
    float pos_ki[3] = {0.0f, 0.0f, 0.0f};
    float pos_kd[3] = {0.0f, 0.0f, 0.0f};
    float vel_kp[3] = {0.0f, 0.0f, 0.0f};
    float vel_ki[3] = {0.0f, 0.0f, 0.0f};
    float vel_kd[3] = {0.0f, 0.0f, 0.0f};
    float vel_error_limit[3] = {0.0f, 0.0f, 0.0f};
    float vel_error_filter_tau[3] = {0.0f, 0.0f, 0.0f};
    float trajectory_vel_error_weight[3] = {1.0f, 1.0f, 1.0f};
    float velocity_vel_error_weight[3] = {1.0f, 1.0f, 1.0f};
    float vel_error_gate_threshold[3] = {0.0f, 0.0f, 0.0f};
    float vel_error_gate_decay[3] = {1.0f, 1.0f, 1.0f};
    float vel_error_gate_hold_s = 0.0f;
    float int_max_pos[3] = {0.0f, 0.0f, 0.0f};
    float int_max_vel[3] = {0.0f, 0.0f, 0.0f};
    float rotor_drag_d[3] = {0.0f, 0.0f, 0.0f};

    static const char* formatName() {
        return "sunray_geometric_controller_param";
    }

    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint8_t", "enable_ulog"},
            {"uint8_t", "control_type"},
            {"uint8_t", "hover_thrust_estimator_type"},
            {"float", "mass_kg"},
            {"float", "gravity"},
            {"float", "hover_thrust_init"},
            {"float", "hover_thrust_min"},
            {"float", "hover_thrust_max"},
            {"float", "accepted_hover_thrust_rate"},
            {"float", "max_acc"},
            {"float", "max_d_acc"},
            {"float", "attitude_tau"},
            {"float", "zb_z_min"},
            {"float", "controller_hz"},
            {"float[3]", "pos_kp"},
            {"float[3]", "pos_ki"},
            {"float[3]", "pos_kd"},
            {"float[3]", "vel_kp"},
            {"float[3]", "vel_ki"},
            {"float[3]", "vel_kd"},
            {"float[3]", "vel_error_limit"},
            {"float[3]", "vel_error_filter_tau"},
            {"float[3]", "trajectory_vel_error_weight"},
            {"float[3]", "velocity_vel_error_weight"},
            {"float[3]", "vel_error_gate_threshold"},
            {"float[3]", "vel_error_gate_decay"},
            {"float", "vel_error_gate_hold_s"},
            {"float[3]", "int_max_pos"},
            {"float[3]", "int_max_vel"},
            {"float[3]", "rotor_drag_d"},
        };
    }

    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(enable_ulog);
        writer.write(control_type);
        writer.write(hover_thrust_estimator_type);
        writer.write(mass_kg);
        writer.write(gravity);
        writer.write(hover_thrust_init);
        writer.write(hover_thrust_min);
        writer.write(hover_thrust_max);
        writer.write(accepted_hover_thrust_rate);
        writer.write(max_acc);
        writer.write(max_d_acc);
        writer.write(attitude_tau);
        writer.write(zb_z_min);
        writer.write(controller_hz);
        writer.writeArray(pos_kp, 3);
        writer.writeArray(pos_ki, 3);
        writer.writeArray(pos_kd, 3);
        writer.writeArray(vel_kp, 3);
        writer.writeArray(vel_ki, 3);
        writer.writeArray(vel_kd, 3);
        writer.writeArray(vel_error_limit, 3);
        writer.writeArray(vel_error_filter_tau, 3);
        writer.writeArray(trajectory_vel_error_weight, 3);
        writer.writeArray(velocity_vel_error_weight, 3);
        writer.writeArray(vel_error_gate_threshold, 3);
        writer.writeArray(vel_error_gate_decay, 3);
        writer.write(vel_error_gate_hold_s);
        writer.writeArray(int_max_pos, 3);
        writer.writeArray(int_max_vel, 3);
        writer.writeArray(rotor_drag_d, 3);
    }
};

struct GeometricControllerInputRecord {
    uint64_t timestamp = 0;
    uint8_t valid = 0;
    uint8_t control_type = 0;
    uint8_t thrust_policy = 0;
    float reference_position[3] = {0.0f, 0.0f, 0.0f};
    float reference_velocity[3] = {0.0f, 0.0f, 0.0f};
    float reference_acceleration[3] = {0.0f, 0.0f, 0.0f};
    float reference_jerk[3] = {0.0f, 0.0f, 0.0f};
    float reference_yaw = 0.0f;
    float reference_yaw_rate = 0.0f;
    float odom_position[3] = {0.0f, 0.0f, 0.0f};
    float odom_velocity[3] = {0.0f, 0.0f, 0.0f};
    float odom_orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float odom_bodyrate[3] = {0.0f, 0.0f, 0.0f};

    static const char* formatName() {
        return "sunray_geometric_controller_input";
    }

    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint8_t", "valid"},
            {"uint8_t", "control_type"},
            {"uint8_t", "thrust_policy"},
            {"float[3]", "reference_position"},
            {"float[3]", "reference_velocity"},
            {"float[3]", "reference_acceleration"},
            {"float[3]", "reference_jerk"},
            {"float", "reference_yaw"},
            {"float", "reference_yaw_rate"},
            {"float[3]", "odom_position"},
            {"float[3]", "odom_velocity"},
            {"float[4]", "odom_orientation_q"},
            {"float[3]", "odom_bodyrate"},
        };
    }

    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(valid);
        writer.write(control_type);
        writer.write(thrust_policy);
        writer.writeArray(reference_position, 3);
        writer.writeArray(reference_velocity, 3);
        writer.writeArray(reference_acceleration, 3);
        writer.writeArray(reference_jerk, 3);
        writer.write(reference_yaw);
        writer.write(reference_yaw_rate);
        writer.writeArray(odom_position, 3);
        writer.writeArray(odom_velocity, 3);
        writer.writeArray(odom_orientation_q, 4);
        writer.writeArray(odom_bodyrate, 3);
    }
};

struct GeometricControllerDebugRecord {
    uint64_t timestamp = 0;
    uint8_t valid = 0;
    uint8_t control_type = 0;
    uint8_t thrust_policy = 0;
    uint8_t pid_accel_saturated = 0;
    uint8_t fixed_anchor_active = 0;
    float current_dt = 0.0f;
    float position_error[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error[3] = {0.0f, 0.0f, 0.0f};
    float yaw_error = 0.0f;
    float attitude_error[3] = {0.0f, 0.0f, 0.0f};
    float integral_pos[3] = {0.0f, 0.0f, 0.0f};
    float integral_vel[3] = {0.0f, 0.0f, 0.0f};
    float last_pos_error[3] = {0.0f, 0.0f, 0.0f};
    float last_vel_error[3] = {0.0f, 0.0f, 0.0f};
    float position_error_dot[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_dot[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_limited[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_filtered[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_weight[3] = {1.0f, 1.0f, 1.0f};
    float velocity_error_gate_triggered[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_gate_active[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_gate_scale[3] = {1.0f, 1.0f, 1.0f};
    float velocity_error_processed[3] = {0.0f, 0.0f, 0.0f};
    float velocity_error_processed_dot[3] = {0.0f, 0.0f, 0.0f};
    float velocity_feedback_term[3] = {0.0f, 0.0f, 0.0f};
    float derivative_term_raw[3] = {0.0f, 0.0f, 0.0f};
    float derivative_term[3] = {0.0f, 0.0f, 0.0f};
    float integral_term[3] = {0.0f, 0.0f, 0.0f};
    float pid_feedback_acceleration_unsaturated[3] = {0.0f, 0.0f, 0.0f};
    float pid_feedback_acceleration[3] = {0.0f, 0.0f, 0.0f};
    float desired_acceleration[3] = {0.0f, 0.0f, 0.0f};
    float hover_thrust_estimate = 0.0f;
    float accepted_hover_thrust = 0.0f;
    float selected_hover_anchor = 0.0f;
    float fixed_anchor_override = 0.0f;

    static const char* formatName() {
        return "sunray_geometric_controller_debug";
    }

    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint8_t", "valid"},
            {"uint8_t", "control_type"},
            {"uint8_t", "thrust_policy"},
            {"uint8_t", "pid_accel_saturated"},
            {"uint8_t", "fixed_anchor_active"},
            {"float", "current_dt"},
            {"float[3]", "position_error"},
            {"float[3]", "velocity_error"},
            {"float", "yaw_error"},
            {"float[3]", "attitude_error"},
            {"float[3]", "integral_pos"},
            {"float[3]", "integral_vel"},
            {"float[3]", "last_pos_error"},
            {"float[3]", "last_vel_error"},
            {"float[3]", "position_error_dot"},
            {"float[3]", "velocity_error_dot"},
            {"float[3]", "velocity_error_limited"},
            {"float[3]", "velocity_error_filtered"},
            {"float[3]", "velocity_error_weight"},
            {"float[3]", "velocity_error_gate_triggered"},
            {"float[3]", "velocity_error_gate_active"},
            {"float[3]", "velocity_error_gate_scale"},
            {"float[3]", "velocity_error_processed"},
            {"float[3]", "velocity_error_processed_dot"},
            {"float[3]", "velocity_feedback_term"},
            {"float[3]", "derivative_term_raw"},
            {"float[3]", "derivative_term"},
            {"float[3]", "integral_term"},
            {"float[3]", "pid_feedback_acceleration_unsaturated"},
            {"float[3]", "pid_feedback_acceleration"},
            {"float[3]", "desired_acceleration"},
            {"float", "hover_thrust_estimate"},
            {"float", "accepted_hover_thrust"},
            {"float", "selected_hover_anchor"},
            {"float", "fixed_anchor_override"},
        };
    }

    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(valid);
        writer.write(control_type);
        writer.write(thrust_policy);
        writer.write(pid_accel_saturated);
        writer.write(fixed_anchor_active);
        writer.write(current_dt);
        writer.writeArray(position_error, 3);
        writer.writeArray(velocity_error, 3);
        writer.write(yaw_error);
        writer.writeArray(attitude_error, 3);
        writer.writeArray(integral_pos, 3);
        writer.writeArray(integral_vel, 3);
        writer.writeArray(last_pos_error, 3);
        writer.writeArray(last_vel_error, 3);
        writer.writeArray(position_error_dot, 3);
        writer.writeArray(velocity_error_dot, 3);
        writer.writeArray(velocity_error_limited, 3);
        writer.writeArray(velocity_error_filtered, 3);
        writer.writeArray(velocity_error_weight, 3);
        writer.writeArray(velocity_error_gate_triggered, 3);
        writer.writeArray(velocity_error_gate_active, 3);
        writer.writeArray(velocity_error_gate_scale, 3);
        writer.writeArray(velocity_error_processed, 3);
        writer.writeArray(velocity_error_processed_dot, 3);
        writer.writeArray(velocity_feedback_term, 3);
        writer.writeArray(derivative_term_raw, 3);
        writer.writeArray(derivative_term, 3);
        writer.writeArray(integral_term, 3);
        writer.writeArray(pid_feedback_acceleration_unsaturated, 3);
        writer.writeArray(pid_feedback_acceleration, 3);
        writer.writeArray(desired_acceleration, 3);
        writer.write(hover_thrust_estimate);
        writer.write(accepted_hover_thrust);
        writer.write(selected_hover_anchor);
        writer.write(fixed_anchor_override);
    }
};

struct GeometricControllerOutputRecord {
    uint64_t timestamp = 0;
    uint8_t has_controller_debug = 0;
    uint8_t attitude_command_mode = 0;
    uint8_t mavros_type_mask = 0;
    uint8_t setpoint_valid = 0;
    float desired_orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float desired_bodyrate[3] = {0.0f, 0.0f, 0.0f};
    float desired_thrust = 0.0f;
    float mavros_orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float mavros_bodyrate[3] = {0.0f, 0.0f, 0.0f};
    float mavros_thrust = 0.0f;

    static const char* formatName() {
        return "sunray_geometric_controller_output";
    }

    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint8_t", "has_controller_debug"},
            {"uint8_t", "attitude_command_mode"},
            {"uint8_t", "mavros_type_mask"},
            {"uint8_t", "setpoint_valid"},
            {"float[4]", "desired_orientation_q"},
            {"float[3]", "desired_bodyrate"},
            {"float", "desired_thrust"},
            {"float[4]", "mavros_orientation_q"},
            {"float[3]", "mavros_bodyrate"},
            {"float", "mavros_thrust"},
        };
    }

    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(has_controller_debug);
        writer.write(attitude_command_mode);
        writer.write(mavros_type_mask);
        writer.write(setpoint_valid);
        writer.writeArray(desired_orientation_q, 4);
        writer.writeArray(desired_bodyrate, 3);
        writer.write(desired_thrust);
        writer.writeArray(mavros_orientation_q, 4);
        writer.writeArray(mavros_bodyrate, 3);
        writer.write(mavros_thrust);
    }
};

struct GeometricControllerRuntimeRecord {
    uint64_t timestamp = 0;
    uint8_t controller_ready = 0;
    uint8_t odom_valid = 0;
    uint8_t early_return_reason = 0;
    uint8_t takeoff_phase = 0;
    uint8_t takeoff_curve_valid = 0;
    uint8_t takeoff_curve_rebased = 0;
    uint8_t px4_connected = 0;
    uint8_t px4_armed = 0;
    uint8_t px4_flight_mode = 0;
    uint8_t px4_landed_state = 0;
    uint8_t thrust_policy = 0;
    float takeoff_curve_position[3] = {0.0f, 0.0f, 0.0f};
    float takeoff_curve_velocity[3] = {0.0f, 0.0f, 0.0f};
    float takeoff_curve_acceleration[3] = {0.0f, 0.0f, 0.0f};
    float commanded_height = 0.0f;
    float commanded_vmax = 0.0f;
    float rel_height = 0.0f;
    float odom_vz = 0.0f;
    float prelift_a_ff = 0.0f;
    float takeoff_thrust_limit = 0.0f;
    float odom_ahead_ratio = 0.0f;
    float odom_correction_scale = 0.0f;
    float accepted_hover_thrust = 0.0f;
    float rebase_v_start_norm = 0.0f;
    float rebase_v_xy_norm = 0.0f;
    float rebase_effective_vmax = 0.0f;
    uint8_t rebase_fallback_level = 0;
    uint8_t takeoff_failed = 0;
    uint8_t takeoff_failure_reason = 0;
    uint8_t landing_phase = 0;
    uint8_t landing_near_ground_trigger = 0;
    float landing_height_above_ground = 0.0f;
    float landing_near_ground_h = 0.0f;
    float landing_ground_effect_release_h = 0.0f;
    float landing_release_progress = 1.0f;
    float landing_a_ff = 0.0f;
    float landing_anchor_thrust = 0.0f;
    float landing_thrust_release_limit = 0.0f;
    float landing_setpoint_thrust = 0.0f;
    float landing_xy_error = 0.0f;
    float landing_desc_speed = 0.0f;
    float landing_ref_vz = 0.0f;
    float landing_odom_vz = 0.0f;

    static const char* formatName() {
        return "sunray_geometric_controller_runtime";
    }

    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint8_t", "controller_ready"},
            {"uint8_t", "odom_valid"},
            {"uint8_t", "early_return_reason"},
            {"uint8_t", "takeoff_phase"},
            {"uint8_t", "takeoff_curve_valid"},
            {"uint8_t", "takeoff_curve_rebased"},
            {"uint8_t", "px4_connected"},
            {"uint8_t", "px4_armed"},
            {"uint8_t", "px4_flight_mode"},
            {"uint8_t", "px4_landed_state"},
            {"uint8_t", "thrust_policy"},
            {"float[3]", "takeoff_curve_position"},
            {"float[3]", "takeoff_curve_velocity"},
            {"float[3]", "takeoff_curve_acceleration"},
            {"float", "commanded_height"},
            {"float", "commanded_vmax"},
            {"float", "rel_height"},
            {"float", "odom_vz"},
            {"float", "prelift_a_ff"},
            {"float", "takeoff_thrust_limit"},
            {"float", "odom_ahead_ratio"},
            {"float", "odom_correction_scale"},
            {"float", "accepted_hover_thrust"},
            {"float", "rebase_v_start_norm"},
            {"float", "rebase_v_xy_norm"},
            {"float", "rebase_effective_vmax"},
            {"uint8_t", "rebase_fallback_level"},
            {"uint8_t", "takeoff_failed"},
            {"uint8_t", "takeoff_failure_reason"},
            {"uint8_t", "landing_phase"},
            {"uint8_t", "landing_near_ground_trigger"},
            {"float", "landing_height_above_ground"},
            {"float", "landing_near_ground_h"},
            {"float", "landing_ground_effect_release_h"},
            {"float", "landing_release_progress"},
            {"float", "landing_a_ff"},
            {"float", "landing_anchor_thrust"},
            {"float", "landing_thrust_release_limit"},
            {"float", "landing_setpoint_thrust"},
            {"float", "landing_xy_error"},
            {"float", "landing_desc_speed"},
            {"float", "landing_ref_vz"},
            {"float", "landing_odom_vz"},
        };
    }

    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(controller_ready);
        writer.write(odom_valid);
        writer.write(early_return_reason);
        writer.write(takeoff_phase);
        writer.write(takeoff_curve_valid);
        writer.write(takeoff_curve_rebased);
        writer.write(px4_connected);
        writer.write(px4_armed);
        writer.write(px4_flight_mode);
        writer.write(px4_landed_state);
        writer.write(thrust_policy);
        writer.writeArray(takeoff_curve_position, 3);
        writer.writeArray(takeoff_curve_velocity, 3);
        writer.writeArray(takeoff_curve_acceleration, 3);
        writer.write(commanded_height);
        writer.write(commanded_vmax);
        writer.write(rel_height);
        writer.write(odom_vz);
        writer.write(prelift_a_ff);
        writer.write(takeoff_thrust_limit);
        writer.write(odom_ahead_ratio);
        writer.write(odom_correction_scale);
        writer.write(accepted_hover_thrust);
        writer.write(rebase_v_start_norm);
        writer.write(rebase_v_xy_norm);
        writer.write(rebase_effective_vmax);
        writer.write(rebase_fallback_level);
        writer.write(takeoff_failed);
        writer.write(takeoff_failure_reason);
        writer.write(landing_phase);
        writer.write(landing_near_ground_trigger);
        writer.write(landing_height_above_ground);
        writer.write(landing_near_ground_h);
        writer.write(landing_ground_effect_release_h);
        writer.write(landing_release_progress);
        writer.write(landing_a_ff);
        writer.write(landing_anchor_thrust);
        writer.write(landing_thrust_release_limit);
        writer.write(landing_setpoint_thrust);
        writer.write(landing_xy_error);
        writer.write(landing_desc_speed);
        writer.write(landing_ref_vz);
        writer.write(landing_odom_vz);
    }
};

}  // namespace controller_data_types
