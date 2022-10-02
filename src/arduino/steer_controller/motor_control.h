#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Arduino.h"
#define PIN_A_TWO 19 //-> 4
#define PIN_B_TWO 18 //-> 5
#define PIN_A_SIX 3 //-> 1
#define PIN_B_SIX 2 //-> 0
#define PIN_A_TEN 21 //-> 2
#define PIN_B_TEN 20 //-> 3
#define OBJECT_NUM 3 // モータ(エンコーダ)の個数
#define OBJECT_TWO 0
#define OBJECT_SIX 1
#define OBJECT_TEN 2
extern float SPEED_NOW[OBJECT_NUM]; // publishに使用
extern float PERIOD; // delayの引数[ms]

class encoder{
  private:
    int PinA, PinB;
    float dt = 0.0;
    unsigned long time_pre = 0, time_now = 0;
    long value_pre = 0, value_now = 0;
  public:
    encoder(int A, int B);
    void getSPEED(int object); // 現在の速度の取得
};
class motor{
  private:
    int POW, DIR;
  public:
    int PWM = 0;
    motor(int P1, int P2);
    void Write(); // 速度の出力
};

void counter0();
void counter1();

#endif
