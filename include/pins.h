#ifndef PINS_H
#define PINS_H

// ============================================================================
// ESP32 引脚定义 (共用 STEP, 独立 DIR 和 脉冲屏蔽 EN)
// ============================================================================

// --- 共享的 STEP 引脚 ---
// 所有的 4988 驱动器共用此脉冲引脚
#define SHARED_STEP_PIN    25

// AccelStepper 需要的虚拟 DIR 引脚（不实际接线）
#define DUMMY_DIR_PIN      26

// --- X 轴步进电机 ---
#define X_DIR_PIN          27
#define X_ENABLE_PIN       14 // 硬件电路上屏蔽 STEP 脉冲 (低电平放行，高电平屏蔽)

// --- Y 轴步进电机 ---
#define Y_DIR_PIN          12
#define Y_ENABLE_PIN       13

// --- Z 轴步进电机 ---
#define Z_DIR_PIN          32
#define Z_ENABLE_PIN       33

// --- 限位开关 / 归零传感器 ---
// 内部上拉输入，低电平触发
#define X_HOME_PIN         34
#define Y_HOME_PIN         35
#define Z_HOME_PIN         36

// --- 目标选择按钮 ---
#define BTN_TARGET_1_PIN   39
#define BTN_TARGET_2_PIN   4
#define BTN_TARGET_3_PIN   16
#define BTN_TARGET_4_PIN   17

// --- 其他 ---
#define LED_PIN            2

#endif // PINS_H
