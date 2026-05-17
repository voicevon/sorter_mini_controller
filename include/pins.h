#ifndef PINS_H
#define PINS_H

// ============================================================================
// ESP32 引脚定义 (共用 STEP, 8路独立 EN, 74HC595/165 串行 DIR/HOME)
// ============================================================================

// --- 共享的 STEP 引脚 ---
#define SHARED_STEP_PIN    5

// AccelStepper 需要的虚拟 DIR 引脚（不实际接线）
#define DUMMY_DIR_PIN      22

// --- 8路 ENABLE 脉冲屏蔽引脚 (低电平放行，高电平屏蔽) ---
// 数组在 SorterController 中定义，这里声明具体引脚
#define EN_PIN_0           13
#define EN_PIN_1           12
#define EN_PIN_2           14
#define EN_PIN_3           27
#define EN_PIN_4           26
#define EN_PIN_5           25
#define EN_PIN_6           33
#define EN_PIN_7           32

// --- 细分控制 (全核共用) ---
#define MS1_PIN            4
#define MS2_PIN            16
#define MS3_PIN            17

// --- 74HC595 (DIR 输出) & 74HC165 (HOME 输入) 共享 SPI ---
#define LATCH_PIN          18 // 锁存 (STCP / PL)
#define CLOCK_PIN          19 // 时钟 (SHCP / CP)
#define DIR_DATA_OUT       21 // HC595 串行数据输入 (DS)
#define HOME_DATA_IN       34 // HC165 串行数据输出 (Q7)

// --- 目标选择按钮 (留作参考，未变更) ---
#define BTN_TARGET_1_PIN   39
#define BTN_TARGET_2_PIN   36 // 改为了 36 以避开部分引脚
#define BTN_TARGET_3_PIN   35
#define BTN_TARGET_4_PIN   32 // 这里的按钮可能会冲突，先随便设，用户会自己改

// --- 其他 ---
#define LED_PIN            2

#endif // PINS_H
