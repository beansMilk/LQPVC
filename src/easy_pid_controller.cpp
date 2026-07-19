#include "easy_pid_controller_pkg/easy_pid_controller.h"

// ============ 构造函数 ============
pid_controller::pid_controller(
    double _kp,
    double _p_cut_val,
    double _ki,
    double _kd,
    double _p_max,
    double _i_max,
    double _d_max,
    double _total_max,
    double _dead_zone,
    double _d_fillter_alpha,
    double _frequency
) : kp(_kp),
    p_cut_val(_p_cut_val),
    kd(_kd),
    ki(_ki),
    p_max(_p_max),
    d_max(_d_max),
    i_max(_i_max),
    total_max(_total_max),
    dead_zone(_dead_zone),
    d_fillter_alpha(_d_fillter_alpha),
    d_fillter_beta(1 - _d_fillter_alpha),
    last_error(0),
    last_i_val(0),
    freq(_frequency),
    max_dt(1 / freq * 1.2),
    down_machine_deadzone(0.1),
    last_error_t(std::chrono::steady_clock::now()),
    last_d_fillter_val(0),
    last_error_vec(Eigen::Vector2d::Zero()),
    last_d_fillter_val_vec(Eigen::Vector2d::Zero()),
    last_i_val_vec(Eigen::Vector2d::Zero()) {
    p_cut_val = p_cut_val > p_max / kp ? p_max / kp : p_cut_val;
}

// ============ 内部函数：比例项 ============
double pid_controller::p_function(const double& x) {
    if (std::abs(x) > p_cut_val) return std::copysign(p_max, x);
    return x * kp;   
}

// ============ 内部函数：微分项 + 低通滤波 ============
double pid_controller::d_fillter(const double& _dx, const double& _last_d_val, const double& _dt) {
    double d_val = _dx / (std::max)(1e-9, _dt) * kd;
    d_val = d_fillter_alpha * d_val + d_fillter_beta * _last_d_val;
    d_val = abs(d_val) > d_max ? std::copysign(d_max, d_val) : d_val;
    return d_val;
}

// ============ 内部函数：积分项 + 梯形积分 + 限幅 ============
double pid_controller::i_function(const double& error, const double& dt) {
    double i_val = last_i_val + ki * error * dt;
    if (i_val > i_max) i_val = i_max;
    if (i_val < -i_max) i_val = -i_max;
    return i_val;
}

// ============ 重置控制器全部状态 ============
void pid_controller::reset() {
    last_error = 0;
    last_d_fillter_val = 0;
    last_i_val = 0;
    last_error_vec = Eigen::Vector2d::Zero();
    last_d_fillter_val_vec = Eigen::Vector2d::Zero();
    last_i_val_vec = Eigen::Vector2d::Zero();
    last_error_t = std::chrono::steady_clock::now();
    current_t = std::chrono::steady_clock::now();
}

// ============ 仅重置积分项 ============
void pid_controller::reset_integral() {
    last_i_val = 0;
    last_i_val_vec = Eigen::Vector2d::Zero();
}

// ============ 更新时间戳 ============
void pid_controller::time() {
    current_t = std::chrono::steady_clock::now();
}

// ============ 标量 PID 输出 ============
double pid_controller::pd_output(const double& x) {
    double dt = std::chrono::duration<double>(current_t - last_error_t).count();
    dt = (std::min)(dt, max_dt);

    // ========== 【跨任务安全保护】 ==========
    // 如果 dt 超过 MAX_STALE_DT（0.5s），说明距离上次调用间隔太久，
    // 极大概率是一个新任务（而非正常循环延迟），此时自动清零全部历史状态，
    // 只输出比例项作为首次控制量。
    if (dt > MAX_STALE_DT) {
        reset();
        // 首次调用：只输出 P 项，不启动 I/D
        if (abs(x) < dead_zone) return 0;
        double p_out = p_function(x);
        last_error = x;
        last_error_t = current_t;
        double res = p_out;
        res = abs(res) < down_machine_deadzone ? std::copysign(down_machine_deadzone, res) : res;
        return abs(res) > total_max ? std::copysign(total_max, res) : res;
    }

    // ========== 死区处理 ==========
    if (abs(x) < dead_zone) {
        last_d_fillter_val = 0;
        last_i_val = 0;
        last_error = 0;
        last_error_t = current_t;
        return 0;
    }

    // ========== 正常 PID 计算 ==========
    double p_out = p_function(x);
    double d_out = d_fillter(x - last_error, last_d_fillter_val, dt);
    double i_out = i_function(x, dt);
    
    // 更新状态
    last_d_fillter_val = d_out;
    last_i_val = i_out;
    last_error = x;
    last_error_t = current_t;
    
    // 求和 + 静摩擦补偿 + 输出限幅
    double res = p_out + i_out + d_out;
    res = abs(res) < down_machine_deadzone ? std::copysign(down_machine_deadzone, res) : res;
    return abs(res) > total_max ? std::copysign(total_max, res) : res;
}

// ============ 二维向量 PID 输出 ============
Eigen::Vector2d pid_controller::pd_output(const Eigen::Vector2d& x) {
    double dt = std::chrono::duration<double>(current_t - last_error_t).count();
    dt = (std::min)(dt, max_dt);

    // ========== 【跨任务安全保护】 ==========
    if (dt > MAX_STALE_DT) {
        reset();
        Eigen::Vector2d res;
        for (int i = 0; i < 2; i++) {
            if (abs(x[i]) < dead_zone) {
                res[i] = 0;
            } else {
                double p_out = p_function(x[i]);
                res[i] = p_out;
                res[i] = abs(res[i]) < down_machine_deadzone ? std::copysign(down_machine_deadzone, res[i]) : res[i];
                res[i] = abs(res[i]) > total_max ? std::copysign(total_max, res[i]) : res[i];
            }
        }
        last_error_vec = x;
        last_error_t = current_t;
        return res;
    }

    // ========== 正常计算 ==========
    last_error_t = current_t;
    Eigen::Vector2d dx = x - last_error_vec;
    last_error_vec = x;

    Eigen::Vector2d res;

    for (int i = 0; i < 2; i++) {
        if (abs(x[i]) < dead_zone) {
            last_d_fillter_val_vec[i] = 0;
            last_i_val_vec[i] = 0;
            last_error_vec[i] = 0;
            res[i] = 0;
            continue;
        }
        
        double p_out = p_function(x[i]);
        double d_out = d_fillter(dx[i], last_d_fillter_val_vec[i], dt);
        double i_out = i_function(x[i], dt);
        
        last_d_fillter_val_vec[i] = d_out;
        last_i_val_vec[i] = i_out;
        
        res[i] = p_out + i_out + d_out;
        res[i] = abs(res[i]) < down_machine_deadzone ? std::copysign(down_machine_deadzone, res[i]) : res[i];
        res[i] = abs(res[i]) > total_max ? std::copysign(total_max, res[i]) : res[i];
    }
    return res;
}
