#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <chrono>

#ifndef EASY_PID_CONTROLLER_H
#define EASY_PID_CONTROLLER_H

/**
 * @brief 完整的 PID 控制器（位置式）
 * 
 * 支持标量和二维向量两种接口，适用于机器人速度/位置闭环控制。
 * 
 * ### 特性
 * - 比例（P）、积分（I）、微分（D）三项完整
 * - 积分限幅（anti-windup）
 * - 微分一阶低通滤波
 * - 死区（dead_zone）—— 误差小于此值时输出 0
 * - 输出总限幅（total_max）+ 静摩擦补偿（down_machine_deadzone）
 * - 跨任务安全保护：自动检测长时间间隔（新任务）并清零历史状态
 * 
 * ### 跨任务安全问题
 * PID 控制器在多次任务间复用同一个实例时，积分项（last_i_val）和
 * 微分历史（last_error, last_d_fillter_val）会从上一个任务残留到下一个。
 * 本控制器提供两层防护：
 *   1. 自动检测：`pd_output()` 中如果 dt 超过 `MAX_STALE_DT`（默认 0.5s），
 *      自动认为这是一个新任务，清零全部状态后重新开始
 *   2. 显式重置：调用 `reset()` 方法可随时手动清零
 * 
 * 建议在每次 `align_srv` 开始时调用 `reset()` 以确保绝对安全。
 */
class pid_controller {
private:
    // ============ PID 核心参数 ============
    double kp, p_cut_val;          // P 项：比例增益 + 饱和阈值
    double kd;                     // D 项：微分增益
    double ki;                     // I 项：积分增益

    // ============ P/I/D 各单项状态 ============
    double last_error;
    double last_d_fillter_val;     // 微分低通滤波历史值
    double last_i_val;             // 积分累积值

    std::chrono::steady_clock::time_point last_error_t, current_t;

    // ============ 二维向量版本的并行状态 ============
    Eigen::Vector2d last_error_vec, last_d_fillter_val_vec, last_i_val_vec;

    // ============ 滤波与限幅参数 ============
    double d_fillter_alpha, d_fillter_beta;  // 微分低通滤波系数
    double p_max, d_max, i_max, total_max;   // 各单项与总输出限幅
    double dead_zone;                        // 死区
    double freq, max_dt;                     // 频率 + dt 最大值保护
    double down_machine_deadzone;            // 静摩擦补偿阈值

    // ============ 跨任务安全保护 ============
    static constexpr double MAX_STALE_DT = 0.5;   // 超过此间隔视为新任务（秒）

    // ============ 内部计算函数 ============
    double p_function(const double&);
    double d_fillter(const double&, const double&, const double&);
    double i_function(const double&, const double&);

public:
    /**
     * @brief 构造 PID 控制器
     * 
     * @param _p_zoom        比例增益 kp
     * @param _p_cut_val     P 项饱和阈值（误差超过此值直接输出 p_max）
     * @param _kd            微分增益
     * @param _ki            积分增益
     * @param _p_max         P 项输出限幅
     * @param _d_max         D 项输出限幅
     * @param _i_max         I 项输出限幅
     * @param _total_max     总输出限幅（限制最大速度/力矩）
     * @param _dead_zone     死区（误差小于此值时输出 0）
     * @param _d_fillter_alpha  微分低通滤波系数（0~1，越大滤波越弱）
     * @param _frequency     期望控制频率（Hz），用于 dt 最大值保护
     */
    pid_controller(
        double _p_zoom,
        double _p_cut_val,
        double _kd,
        double _ki,
        double _p_max,
        double _d_max,
        double _i_max,
        double _total_max,
        double _dead_zone,
        double _d_fillter_alpha,
        double _frequency
    );

    /**
     * @brief 标量 PID 输出（单自由度）
     * 
     * 内部自动检测跨任务间隔：如果距离上次调用超过 MAX_STALE_DT（0.5s），
     * 自动清零积分和微分历史，只输出 P 项。
     * 
     * @param x 当前误差
     * @return double 控制输出
     */
    double pd_output(const double&);

    /**
     * @brief 二维向量 PID 输出（两个自由度并行）
     * 
     * 对向量的每个分量独立执行 PID 计算。
     * 
     * @param x 当前误差向量 (x, y)
     * @return Eigen::Vector2d 控制输出向量
     */
    Eigen::Vector2d pd_output(const Eigen::Vector2d&);

    /**
     * @brief 更新时间戳
     * 
     * 必须在每次调用 pd_output() 前调用，以正确计算 dt。
     */
    void time();

    /**
     * @brief 重置全部状态
     * 
     * 清零积分累积、微分历史、上一次误差和所有时间戳。
     * 在以下场景应调用：
     * - 新任务开始时（如新的 align_srv 请求）
     * - 任务失败/取消后重试前
     * - 控制器长时间未使用后
     */
    void reset();

    /**
     * @brief 仅重置积分项（保留微分历史）
     * 
     * 当积分饱和导致过冲时可选择性调用。
     * @deprecated 建议使用 reset() 全面重置
     */
    void reset_integral();
};

#endif
