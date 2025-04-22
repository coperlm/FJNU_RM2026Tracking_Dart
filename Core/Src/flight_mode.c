#include "flight_mode.h"
#include "main.h"
#include "attitude_control.h"
#include <string.h>
#include <math.h>

// 静态函数声明
static void search_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);
static void tracking_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);
static void attack_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);
static void return_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);
static void failsafe_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);

// 静态变量定义
static FlightState_t flight_state;
static FlightConfig_t flight_config;

// 模式和阶段名称定义（英文缩写，适用于不支持中文的 OLED）

// 飞行模式（原：待机、搜索、追踪、攻击、返回、故障保护）
static const char* mode_names[MODE_COUNT] = {
    "STBY",    // 待机 (Standby)
    "SRCH",    // 搜索 (Search)
    "TRCK",    // 追踪 (Tracking)
    "ATCK",    // 攻击 (Attack)
    "RTN",     // 返回 (Return)
    "FAIL"     // 故障保护 (Failsafe)
};

// 飞行阶段（原：地面、起飞、巡航、接近、着陆）
static const char* phase_names[PHASE_COUNT] = {
    "GRND",    // 地面 (Ground)
    "TKOF",    // 起飞 (Takeoff)
    "CRUZ",    // 巡航 (Cruise)
    "APPR",    // 接近 (Approach)
    "LAND"     // 着陆 (Landing)
};


/**
 * @brief 初始化飞行状态机
 */
void flight_mode_init(void) {
    // 初始化飞行状态
    memset(&flight_state, 0, sizeof(FlightState_t));
    flight_state.current_mode = MODE_STANDBY;
    flight_state.current_phase = PHASE_GROUND;
    flight_state.mode_entry_time = HAL_GetTick();
    flight_state.armed = false;
    
    // 初始化默认配置
    flight_config.min_altitude = 1.0f;        // 最小飞行高度1米
    flight_config.cruise_altitude = 5.0f;     // 巡航高度5米
    flight_config.cruise_speed = 8.0f;        // 巡航速度8米/秒
    flight_config.approach_speed = 5.0f;      // 接近速度5米/秒
    flight_config.tracking_distance = 10.0f;  // 追踪距离10米
    flight_config.attack_distance = 3.0f;     // 攻击距离3米
    flight_config.landing_speed = 1.5f;       // 着陆速度1.5米/秒
    flight_config.mode_timeout_ms = 30000;    // 模式超时30秒
}

/**
 * @brief 设置飞行模式
 * 
 * @param mode 要设置的飞行模式
 * @return bool 设置是否成功
 */
bool flight_mode_set(FlightMode_t mode) {
    // 检查模式是否有效
    if (mode >= MODE_COUNT) {
        return false;
    }
    
    // 如果处于紧急状态，不允许切换到其他模式
    if (flight_state.current_mode == MODE_FAILSAFE && mode != MODE_STANDBY) {
        return false;
    }
    
    // 如果未解锁，只允许切换到待机模式
    if (!flight_state.armed && mode != MODE_STANDBY) {
        return false;
    }
    
    // 记录上一个模式并切换到新模式
    flight_state.previous_mode = flight_state.current_mode;
    flight_state.current_mode = mode;
    flight_state.mode_entry_time = HAL_GetTick();
    flight_state.mode_change_request = false;
    
    return true;
}

/**
 * @brief 获取当前飞行模式
 * 
 * @return FlightMode_t 当前的飞行模式
 */
FlightMode_t flight_mode_get(void) {
    return flight_state.current_mode;
}

/**
 * @brief 获取当前飞行阶段
 * 
 * @return FlightPhase_t 当前的飞行阶段
 */
FlightPhase_t flight_phase_get(void) {
    return flight_state.current_phase;
}

/**
 * @brief 状态机主更新函数 - 根据当前状态和传感器数据更新控制输出
 * 
 * @param attitude 当前姿态数据
 * @param target 目标数据
 * @param control 输出控制量
 * @param dt 时间增量(秒)
 */
void flight_mode_update(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    // 更新系统运行时间
    flight_state.system_uptime = HAL_GetTick();
    
    // 检查模式超时
    uint32_t mode_duration = flight_state.system_uptime - flight_state.mode_entry_time;
    
    // 如果未解锁，保持在待机模式
    if (!flight_state.armed && flight_state.current_mode != MODE_STANDBY) {
        flight_mode_set(MODE_STANDBY);
    }
    
    // 处理模式切换请求
    if (flight_state.mode_change_request) {
        flight_mode_set(flight_state.requested_mode);
    }
    
    // 基于目标检测状态自动模式切换
    if (flight_state.armed) {
        if (flight_state.current_mode != MODE_FAILSAFE) {
            if (target->detected) {
                // 如果检测到目标，切换到追踪模式
                if (flight_state.current_mode != MODE_TRACKING && 
                    flight_state.current_mode != MODE_ATTACK) {
                    flight_mode_set(MODE_TRACKING);
                }
            } else {
                // 如果丢失目标且当前在追踪或攻击模式，切换到搜索模式
                if (flight_state.current_mode == MODE_TRACKING || 
                    flight_state.current_mode == MODE_ATTACK) {
                    flight_mode_set(MODE_SEARCH);
                }
            }
        }
    }
    
    // 基于当前模式进行控制策略选择
    switch (flight_state.current_mode) {
        case MODE_STANDBY:
            // 待机模式 - 电机怠速，保持水平
            control->thruster_1 = MOTOR_IDLE;
            control->thruster_2 = MOTOR_IDLE;
            control->left_wing = SERVO_CENTER;
            control->right_wing = SERVO_CENTER;
            break;
            
        case MODE_SEARCH:
            // 搜索模式 - 执行搜索模式的控制策略
            // 可以实现简单的扫描或螺旋上升搜索
            // 控制输出在下面单独实现
            search_mode_control(attitude, target, control, dt);
            break;
            
        case MODE_TRACKING:
            // 追踪模式 - 追踪目标
            tracking_mode_control(attitude, target, control, dt);
            // 如果距离目标足够近，切换到攻击模式
            // 这里简化模拟，实际应加入距离判断
            if (target->detected && mode_duration > 3000) {
                flight_mode_set(MODE_ATTACK);
            }
            break;
            
        case MODE_ATTACK:
            // 攻击模式 - 全速接近目标
            attack_mode_control(attitude, target, control, dt);
            break;
            
        case MODE_RETURN:
            // 返回模式 - 返航逻辑
            return_mode_control(attitude, target, control, dt);
            break;
            
        case MODE_FAILSAFE:
            // 故障保护模式 - 尝试安全着陆或返回
            failsafe_mode_control(attitude, target, control, dt);
            break;
            
        default:
            // 未知模式，默认为待机
            flight_mode_set(MODE_STANDBY);
            break;
    }
    
    // 根据飞行阶段调整控制输出 (在实际系统中使用高度和速度传感器)
    // 此处为简化实现，具体根据传感器数据调整
    
    // 更新飞行阶段 (简化实现)
    if (flight_state.current_mode == MODE_STANDBY) {
        flight_state.current_phase = PHASE_GROUND;
    } else {
        // 实际系统中应根据高度和速度确定飞行阶段
        flight_state.current_phase = PHASE_CRUISE;
    }
}

/**
 * @brief 搜索模式控制策略
 */
static void search_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    static float search_pattern_time = 0.0f;
    static float search_roll = 0.0f;
    static float search_pitch = 0.0f;
    
    // 增加搜索模式计时器
    search_pattern_time += dt;
    
    // 简单的搜索模式 - 执行S形搜索路径
    // 实际系统中应使用更复杂的搜索算法
    search_roll = 15.0f * sinf(0.5f * search_pattern_time);
    search_pitch = 10.0f * sinf(0.3f * search_pattern_time);
    
    // 直接设置舵机输出 (使用姿态控制器实现)
    Target_t fake_target;
    fake_target.x = search_pitch / 20.0f;  // 映射到[-1,1]范围
    fake_target.y = search_roll / 30.0f;   // 映射到[-1,1]范围
    fake_target.detected = true;
    
    // 使用标准姿态控制器，传入"虚拟目标"实现搜索路径 (移除dt参数)
    update_attitude_control(attitude, &fake_target, control);
    
    // 设置中等推力
    control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.6f);
    control->thruster_2 = control->thruster_1;
}

/**
 * @brief 追踪模式控制策略
 */
static void tracking_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    // 使用标准姿态控制器追踪目标 (移除dt参数)
    update_attitude_control(attitude, target, control);
    
    // 设置适中的推力以保持追踪
    control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.7f);
    control->thruster_2 = control->thruster_1;
}

/**
 * @brief 攻击模式控制策略
 */
static void attack_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    // 使用更激进的PID参数和更高的前馈增益(这里简化为直接使用姿态控制器)
    update_attitude_control(attitude, target, control);
    
    // 增大推力 - 快速接近目标
    control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.9f);
    control->thruster_2 = control->thruster_1;
}

/**
 * @brief 返回模式控制策略
 */
static void return_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    // 简化实现 - 实际系统中应根据IMU和导航数据返航
    
    // 保持水平姿态
    Target_t home_target;
    home_target.x = 0.0f;
    home_target.y = 0.0f;
    home_target.detected = true;
    
    // 移除dt参数
    update_attitude_control(attitude, &home_target, control);
    
    // 降低推力
    control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.5f);
    control->thruster_2 = control->thruster_1;
}

/**
 * @brief 故障保护模式控制策略
 */
static void failsafe_mode_control(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt) {
    // 故障模式 - 尝试保持水平姿态
    Target_t safe_target;
    safe_target.x = 0.0f;
    safe_target.y = 0.0f;
    safe_target.detected = true;
    
    // 移除dt参数
    update_attitude_control(attitude, &safe_target, control);
    
    // 低推力降落
    control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.3f);
    control->thruster_2 = control->thruster_1;
}

/**
 * @brief 解锁系统
 * 
 * @return bool 操作是否成功
 */
bool flight_mode_arm(void) {
    flight_state.armed = true;
    return true;
}

/**
 * @brief 锁定系统
 * 
 * @return bool 操作是否成功
 */
bool flight_mode_disarm(void) {
    if (flight_state.current_phase == PHASE_GROUND) {
        flight_state.armed = false;
        flight_mode_set(MODE_STANDBY);
        return true;
    }
    return false; // 飞行中不允许锁定
}

/**
 * @brief 获取系统是否解锁
 * 
 * @return bool 系统是否解锁
 */
bool flight_is_armed(void) {
    return flight_state.armed;
}

/**
 * @brief 获取模式名称字符串
 * 
 * @param mode 飞行模式
 * @return const char* 模式名称字符串
 */
const char* flight_mode_get_name(FlightMode_t mode) {
    if (mode < MODE_COUNT) {
        return mode_names[mode];
    }
    return "未知";
}

/**
 * @brief 获取飞行阶段名称字符串
 * 
 * @param phase 飞行阶段
 * @return const char* 阶段名称字符串
 */
const char* flight_phase_get_name(FlightPhase_t phase) {
    if (phase < PHASE_COUNT) {
        return phase_names[phase];
    }
    return "未知";
}

/**
 * @brief 设置飞行配置参数
 * 
 * @param config 飞行配置结构体
 */
void flight_set_config(FlightConfig_t config) {
    flight_config = config;
}

/**
 * @brief 获取飞行配置参数
 * 
 * @return FlightConfig_t 飞行配置结构体
 */
FlightConfig_t flight_get_config(void) {
    return flight_config;
}
