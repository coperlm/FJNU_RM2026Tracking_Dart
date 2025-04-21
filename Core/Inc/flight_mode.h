#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include <stdint.h>
#include <stdbool.h>
#include "dart_control.h"

// 飞行模式定义 - 借鉴PX4的模式设计
typedef enum {
    MODE_STANDBY = 0,     // 待机模式 - 系统初始化完成但未激活
    MODE_SEARCH,          // 搜索模式 - 主动搜索目标
    MODE_TRACKING,        // 追踪模式 - 已发现目标并追踪
    MODE_ATTACK,          // 攻击模式 - 接近目标并执行攻击动作
    MODE_RETURN,          // 返回模式 - 任务完成后返回
    MODE_FAILSAFE,        // 故障保护模式 - 发生错误时的安全模式
    MODE_COUNT            // 模式总数（用于边界检查）
} FlightMode_t;

// 飞行阶段定义
typedef enum {
    PHASE_GROUND = 0,     // 地面阶段
    PHASE_TAKEOFF,        // 起飞阶段
    PHASE_CRUISE,         // 巡航阶段
    PHASE_APPROACH,       // 接近阶段
    PHASE_LANDING,        // 着陆阶段
    PHASE_COUNT           // 阶段总数（用于边界检查）
} FlightPhase_t;

// 飞行状态机配置
typedef struct {
    float min_altitude;          // 最小飞行高度（米）
    float cruise_altitude;       // 巡航高度（米）
    float cruise_speed;          // 巡航速度（m/s）
    float approach_speed;        // 接近目标时的速度（m/s）
    float tracking_distance;     // 追踪目标的距离（米）
    float attack_distance;       // 攻击目标的距离（米）
    float landing_speed;         // 着陆速度（m/s）
    uint32_t mode_timeout_ms;    // 模式超时时间（毫秒）
} FlightConfig_t;

// 飞行状态机当前状态
typedef struct {
    FlightMode_t current_mode;   // 当前飞行模式
    FlightMode_t previous_mode;  // 上一个飞行模式
    FlightPhase_t current_phase; // 当前飞行阶段
    uint32_t mode_entry_time;    // 进入当前模式的时间
    uint32_t system_uptime;      // 系统运行时间
    bool armed;                  // 是否解锁
    bool mode_change_request;    // 模式切换请求
    FlightMode_t requested_mode; // 请求切换的模式
} FlightState_t;

// 函数声明
void flight_mode_init(void);
bool flight_mode_set(FlightMode_t mode);
FlightMode_t flight_mode_get(void);
FlightPhase_t flight_phase_get(void);
void flight_mode_update(const Attitude_t *attitude, const Target_t *target, Control_t *control, float dt);
bool flight_mode_arm(void);
bool flight_mode_disarm(void);
bool flight_is_armed(void);
const char* flight_mode_get_name(FlightMode_t mode);
const char* flight_phase_get_name(FlightPhase_t phase);
void flight_set_config(FlightConfig_t config);
FlightConfig_t flight_get_config(void);

#endif /* FLIGHT_MODE_H */