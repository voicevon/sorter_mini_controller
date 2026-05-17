#ifndef SORTER_CONTROLLER_H
#define SORTER_CONTROLLER_H

#include <AccelStepper.h>
#include "config.h"
#include "InputQueue.h"

// ============================================================================
// SorterController —— 分拣机核心控制器
//
// 负责管理三级物品流水线与节拍同步状态机。
// 在 setup() 中调用 begin() 完成硬件初始化，
// 在每次 loop() 中调用 update() 驱动状态机与电机。
// 通过 queueTarget() 投入待分拣目标，通过 triggerHoming() 触发归零。
// ============================================================================
class SorterController {
public:
  // ---- 公开状态枚举（可供主程序读取）---------------------------------------
  enum State {
    IDLE,             // 空闲，等待任务
    NEW_BEAT_PREP,    // 新节拍准备：推进流水线、下发电机指令
    MOVING_TO_ROUTE,  // 电机运动中，等待到位
    SLIDING_WAIT,     // 物品滑行等待（自然重力滑道）
    COMPLETED_BEAT,   // 本节拍结束，准备回到空闲
    HOMING            // 归零校准中
  };

  // -------------------------------------------------------------------------
  SorterController(AccelStepper& sharedStepper);

  // 在 setup() 中调用一次——配置电机速度/加速度及限位开关引脚。
  void begin();

  // 在每次 loop() 中调用——驱动状态机与步进电机。
  void update();

  // 将排序目标（1–4）加入队列。
  // 在任意上下文中调用均安全（AVR 上数组写入为原子操作）。
  void queueTarget(int targetID);

  // 中止当前任务，进入归零校准模式。
  void triggerHoming();

  State getState() const { return _state; }

private:
  // ---- 电机引用 ------------------------------------------------------------
  AccelStepper& _sharedStepper;

  // ---- 状态机 --------------------------------------------------------------
  State         _state;
  unsigned long _timerMark;  // 滑行计时起始时间戳

  // ---- 多级物品流水线 -------------------------------------------------------
  // _pipeline[0] = 当前在第一级电机位置的物品（最新入队）
  // ... _pipeline[NUM_MOTORS-1]
  // 值 0 = 空槽；1及以上 = 目标分拣口 ID。
  int _pipeline[NUM_MOTORS];

  // ---- 输入缓冲 ------------------------------------------------------------
  InputQueue _queue;

  // ---- 归零子状态 ---------------------------------------------------------
  bool _homingInit;           // 是否已完成归零初始化
  bool _homingDone[NUM_MOTORS]; // 各轴归零完成标志

  // ---- 硬件配置与状态 -----------------------------------------------------
  uint8_t _enablePins[NUM_MOTORS];
  int _motorDirs[NUM_MOTORS];

  // ---- 私有辅助函数 --------------------------------------------------------

  // 通过共享的 SPI 接口 (74HC595 + 74HC165)
  // 下发 8 位 DIR 状态，同时返回读取到的 8 位 HOME 传感器状态
  uint8_t _transferSPI(uint8_t dir_state);

  // 根据流水线阶段和目标 ID，返回该电机的目标绝对步数位置。
  // 若电机无需移动（物品已在上级分出，或槽位为空），返回 MOTOR_NO_MOVE。
  // 分拣轮具有 90° 旋转对称性，因此任意停止位置均等价于中立位。
  long _getTargetPos(int stage, int targetID) const;

  // 统一下发组运动配置（设置 DIR 和 EN，并让 _sharedStepper 走相应步数）
  void _prepareGroupMove();

  void _runAll();              // 驱动主步进电机产生脉冲
  bool _anyRunning() const;    // 主电机仍在运动则返回 true
  void _doHomingStep();        // 归零例程（每次 loop 调用一步）
  void _resetPositions();      // 到位后将主电机当前位置清零

  void _advancePipeline();     // 流水线整体前移一格，弹出队首目标
  void _printPipelineState() const; // 串口打印当前流水线状态
};

#endif // SORTER_CONTROLLER_H
