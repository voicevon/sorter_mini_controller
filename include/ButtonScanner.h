#ifndef BUTTON_SCANNER_H
#define BUTTON_SCANNER_H

#include <Arduino.h>
#include "pins.h"
#include "config.h"  // 引入 DEBOUNCE_MS 等时序宏

// ============================================================================
// ButtonScanner —— 四按钮防抖扫描器
//
// 在 setup() 中调用 begin() 配置引脚，
// 在每次 loop() 中调用 scan() 完成防抖采样。
//
// 事件通过构造时注入的两个回调派发：
//   onTarget(int targetID) —— 单键按下时触发（targetID 为 1..4）
//   onHomingTriggered()    —— 同时按下 2 个或更多按键时触发归零
// ============================================================================
class ButtonScanner {
public:
  using TargetCb = void (*)(int targetID); // 单键回调类型
  using HomingCb = void (*)();             // 归零回调类型

  ButtonScanner(TargetCb onTarget, HomingCb onHoming)
    : _onTarget(onTarget), _onHoming(onHoming),
      _homingFired(false), _lastScan(0)
  {
    for (int i = 0; i < NUM_BTNS; i++) _state[i] = false;
  }

  // 在 setup() 中调用一次，初始化按钮引脚为内部上拉输入。
  void begin() {
    pinMode(BTN_TARGET_1_PIN, INPUT_PULLUP);
    pinMode(BTN_TARGET_2_PIN, INPUT_PULLUP);
    pinMode(BTN_TARGET_3_PIN, INPUT_PULLUP);
    pinMode(BTN_TARGET_4_PIN, INPUT_PULLUP);
  }

  // 在每次 loop() 中调用；内部维护 50 ms 防抖间隔。
  void scan() {
    if (millis() - _lastScan < DEBOUNCE_MS) return;
    _lastScan = millis();

    const uint8_t pins[NUM_BTNS] = {
      BTN_TARGET_1_PIN, BTN_TARGET_2_PIN,
      BTN_TARGET_3_PIN, BTN_TARGET_4_PIN
    };

    bool now[NUM_BTNS];
    int  pressCount = 0;

    // 读取所有按键状态（INPUT_PULLUP：LOW 表示按下）
    for (int i = 0; i < NUM_BTNS; i++) {
      now[i] = (digitalRead(pins[i]) == LOW);
      if (now[i]) pressCount++;
    }

    // 多键同时按下 -> 触发归零（每次手势只触发一次，松手后可重新触发）
    if (pressCount >= 2) {
      if (!_homingFired) {
        _homingFired = true;
        if (_onHoming) _onHoming();
      }
    } else {
      _homingFired = false; // 松手后重置
    }

    // 单键上升沿 -> 派发排序目标（归零手势期间不派发）
    if (pressCount < 2) {
      for (int i = 0; i < NUM_BTNS; i++) {
        if (now[i] && !_state[i] && _onTarget) {
          _logTransition(pins[i], i + 1, true);
          _onTarget(i + 1);
        } else if (!now[i] && _state[i]) {
          _logTransition(pins[i], i + 1, false);
        }
      }
    }

    // 提交本轮状态
    for (int i = 0; i < NUM_BTNS; i++) _state[i] = now[i];
  }

private:
  static const int NUM_BTNS = 4; // 按钮总数
  // DEBOUNCE_MS 由 config.h 统一定义

  TargetCb      _onTarget;        // 单键按下回调
  HomingCb      _onHoming;        // 多键归零回调
  bool          _state[NUM_BTNS]; // 各按键上一轮状态
  bool          _homingFired;     // 本次多键手势是否已触发
  unsigned long _lastScan;        // 上次扫描时间戳

  // 串口打印按键状态变化日志
  static void _logTransition(uint8_t pin, int btnNum, bool pressed) {
    Serial.print("[按键] 引脚 "); Serial.print(pin);
    Serial.print("（BTN "); Serial.print(btnNum); Serial.print("）-> ");
    Serial.println(pressed ? "LOW（按下）" : "HIGH（释放）");
  }
};

#endif // BUTTON_SCANNER_H
