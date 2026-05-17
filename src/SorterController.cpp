#include "SorterController.h"
#include "pins.h"
#include <Arduino.h>

// ============================================================================
// 构造函数
// ============================================================================

SorterController::SorterController(AccelStepper& sharedStepper)
  : _sharedStepper(sharedStepper),
    _state(IDLE), _timerMark(0),
    _homingInit(false)
{
  uint8_t ep[NUM_MOTORS] = { EN_PIN_0, EN_PIN_1, EN_PIN_2, EN_PIN_3, EN_PIN_4, EN_PIN_5, EN_PIN_6, EN_PIN_7 };
  int md[NUM_MOTORS] = { -1, -1, 1, 1, 1, 1, 1, 1 };
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    _pipeline[i] = 0;
    _homingDone[i] = false;
    _enablePins[i] = ep[i];
    _motorDirs[i] = md[i];
  }
}

// ============================================================================
// begin() —— 一次性硬件初始化
// ============================================================================

void SorterController::begin() {
  // 初始化 SPI 接口
  pinMode(LATCH_PIN, OUTPUT); digitalWrite(LATCH_PIN, HIGH);
  pinMode(CLOCK_PIN, OUTPUT); digitalWrite(CLOCK_PIN, HIGH);
  pinMode(DIR_DATA_OUT, OUTPUT); digitalWrite(DIR_DATA_OUT, LOW);
  pinMode(HOME_DATA_IN, INPUT); // 也可以用 INPUT_PULLUP，取决于硬件
  
  // 屏蔽步进驱动器脉冲（高电平屏蔽）
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(_enablePins[i], OUTPUT);
    digitalWrite(_enablePins[i], HIGH);
  }

  // 初始清除 DIR 状态
  _transferSPI(0x00);

  // 配置 AccelStepper 速度与加速度
  _sharedStepper.setMaxSpeed(STEPPER_MAX_SPEED); 
  _sharedStepper.setAcceleration(STEPPER_ACCELERATION);

  // 限位开关输入不再直接初始化，由 HC165 接管
}

// ============================================================================
// 公开接口
// ============================================================================

void SorterController::queueTarget(int targetID) {
  _queue.push(targetID);
}

void SorterController::triggerHoming() {
  Serial.println("检测到多键同时按下 -> 进入归零模式。");
  _queue.flush();                            // 清空待处理队列
  // 清空流水线
  for (int i = 0; i < NUM_MOTORS; i++) _pipeline[i] = 0;
  _state = HOMING;
}

// ============================================================================
// update() —— 每次 loop() 调用
// ============================================================================

void SorterController::update() {
  // 始终驱动步进电机（保持脉冲连续性）
  _runAll();

  switch (_state) {

    case IDLE: {
      bool pipeline_empty = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (_pipeline[i] != 0) pipeline_empty = false;
      }
      
      // 队列非空或流水线仍有物品时，启动新节拍
      if (!_queue.empty() || !pipeline_empty) {
        _state = NEW_BEAT_PREP;
      }
      break;
    }

    case NEW_BEAT_PREP:
      Serial.println("\n--- 新节拍开始 ---");
      _advancePipeline();    // 流水线前移，弹入新目标
      _printPipelineState(); // 打印当前流水线状态

      // 向各轴下发组合移动指令
      _prepareGroupMove();

      _state = MOVING_TO_ROUTE;
      break;

    case MOVING_TO_ROUTE:
      // 等待所有轴到位
      if (!_anyRunning()) {
        Serial.println("已到达分拣位置，等待物品滑行...");
        // 分拣轮 90° 旋转对称——将当前停止点重新声明为零点。
        // 这样下一节拍无论目标相同还是不同，都能正确计算相对位移。
        _resetPositions();
        _timerMark = millis();
        _state = SLIDING_WAIT;
      }
      break;

    case SLIDING_WAIT:
      // 物品在重力滑道上自然滑行，等待固定时间后继续
      if (millis() - _timerMark >= SLIDE_WAIT_MS) {
        // 分拣轮 90° 对称——电机留在当前位置，等下次节拍再移动。
        Serial.println("滑行等待结束，本节拍完成。");
        _state = COMPLETED_BEAT;
      }
      break;

    case COMPLETED_BEAT:
      // 节拍冷却完毕，回到空闲
      _state = IDLE;
      break;

    case HOMING:
      _doHomingStep();
      break;
  }
}

// ============================================================================
// 私有辅助函数
// ============================================================================

long SorterController::_getTargetPos(int stage, int targetID) const {
  if (targetID == 0) return MOTOR_NO_MOVE; // 槽位为空，不移动

  // 如果超出原本的3级逻辑，暂时默认不移动（可由用户自行扩展8级树）
  if (stage >= 3) return MOTOR_NO_MOVE;

  switch (stage) {
    case 0: // 第一级分拣
      return (targetID == 1)
        ? _motorDirs[stage] * ROTATE_LEFT_POS
        : _motorDirs[stage] * ROTATE_RIGHT_POS;

    case 1: // 第二级分拣
      if (targetID == 1) return MOTOR_NO_MOVE;
      return (targetID == 2)
        ? _motorDirs[stage] * ROTATE_RIGHT_POS
        : _motorDirs[stage] * ROTATE_LEFT_POS;

    case 2: // 第三级分拣
      if (targetID == 1 || targetID == 2) return MOTOR_NO_MOVE;
      return (targetID == 3)
        ? _motorDirs[stage] * ROTATE_LEFT_POS
        : _motorDirs[stage] * ROTATE_RIGHT_POS;

    default:
      return MOTOR_NO_MOVE;
  }
}

uint8_t SorterController::_transferSPI(uint8_t dir_state) {
  // 1. 锁存当前 HC165 引脚状态到移位寄存器，并将上一次 HC595 数据推到输出端口
  digitalWrite(LATCH_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(LATCH_PIN, HIGH);
  delayMicroseconds(1);

  uint8_t home_state = 0;
  for (int i = 0; i < 8; i++) {
    // 读取 HC165 数据 (通常 Q7 优先输出，对应最高位)
    if (digitalRead(HOME_DATA_IN) == HIGH) {
      home_state |= (1 << (7 - i));
    }
    
    // 写 HC595 数据 (高位先发，保证最后移入 Q0 的是最低位，以此类推)
    digitalWrite(DIR_DATA_OUT, (dir_state & (1 << (7 - i))) ? HIGH : LOW);
    
    // 时钟脉冲，移位
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
  }

  // 2. 再次锁存，将刚才移入 HC595 的数据立即推到输出端口
  digitalWrite(LATCH_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(LATCH_PIN, HIGH);

  return home_state;
}

void SorterController::_prepareGroupMove() {
  uint8_t dir_state = 0;
  bool move_any = false;

  for (int i = 0; i < NUM_MOTORS; i++) {
    long tx = _getTargetPos(i, _pipeline[i]);
    bool move = (tx != MOTOR_NO_MOVE);
    
    if (move) {
      move_any = true;
      digitalWrite(_enablePins[i], LOW); // 放行
      if (tx > 0) {
        dir_state |= (1 << i); // 假设位 i 对应电机 i 的方向
      }
    } else {
      digitalWrite(_enablePins[i], HIGH); // 屏蔽
    }
  }

  // 下发方向到 74HC595
  _transferSPI(dir_state);

  // 让共享的 stepper 执行统一的相对运动。
  if (move_any) {
    _sharedStepper.move(STEPS_PER_90DEG);
  }
}

void SorterController::_runAll() {
  _sharedStepper.run();
}

void SorterController::_resetPositions() {
  _sharedStepper.setCurrentPosition(0);
}

bool SorterController::_anyRunning() const {
  return _sharedStepper.isRunning();
}

void SorterController::_advancePipeline() {
  // 将各级物品向下游推进一格，从队列弹入新物品
  for (int i = NUM_MOTORS - 1; i > 0; i--) {
    _pipeline[i] = _pipeline[i - 1];
  }
  _pipeline[0] = _queue.pop();
}

void SorterController::_printPipelineState() const {
  Serial.print("流水线状态：[M1: "); Serial.print(_pipeline[0]);
  for (int i = 1; i < NUM_MOTORS; i++) {
    Serial.print("] -> [M"); Serial.print(i + 1); Serial.print(": "); Serial.print(_pipeline[i]);
  }
  Serial.println("]");
}

void SorterController::_doHomingStep() {
  if (!_homingInit) {
    Serial.println("开始同步归零...");
    for (int i = 0; i < NUM_MOTORS; i++) {
      _homingDone[i] = false;
    }
    
    // 以恒速直接驱动（绕过加速曲线），速度为负值
    _sharedStepper.setSpeed(HOMING_CONSTANT_SPEED);
    
    // 强制各轴方向为归零方向（负向，假设低电平，发送全 0）
    _transferSPI(0x00);

    // 放开所有轴的屏蔽
    for (int i = 0; i < NUM_MOTORS; i++) {
      digitalWrite(_enablePins[i], LOW);
    }

    _homingInit = true;
  }

  // 通过 SPI 读取所有 HOME 状态 (并维持 DIR=0x00)
  uint8_t home_state = _transferSPI(0x00);
  bool all_done = true;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!_homingDone[i]) {
      // 限位开关通常触发时被拉低，如果低电平触发，位为 0
      bool triggered = ((home_state & (1 << i)) == 0); 
      if (triggered) {
        digitalWrite(_enablePins[i], HIGH); // 触发后立即屏蔽脉冲
        _homingDone[i] = true;
        Serial.print("电机 "); Serial.print(i); Serial.println(" 归零完成。");
      } else {
        all_done = false;
      }
    }
  }

  // 驱动共享脉冲
  if (all_done) {
    _sharedStepper.stop();
    _sharedStepper.setCurrentPosition(0);
    _homingInit = false;
    _state = IDLE;
    Serial.println("--- 所有电机归零并重置原点 ---");
  } else {
    _sharedStepper.runSpeed();
  }
}
