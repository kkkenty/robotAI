#include "motor_control.h"

float SPEED_NOW[OBJECT_NUM] = {0}; 
long PULSE_NOW[OBJECT_NUM] = {0}; 
// 割り込み処理(class宣言外)に使われるため，global変数
volatile long value[OBJECT_NUM] = {0}; // エンコーダの値(割り込みで変化)
volatile int nowSig_A[OBJECT_NUM] = {0}, nowSig_B[OBJECT_NUM] = {0};
volatile int oldSig_A[OBJECT_NUM] = {0}, oldSig_B[OBJECT_NUM] = {0}; // A,B相の信号
volatile int nowState[OBJECT_NUM] = {0}, oldState[OBJECT_NUM] = {0}; // A,B相の状態 

encoder::encoder(int A, int B){
  PinA = A; PinB = B;
  pinMode(PinA, INPUT);
  pinMode(PinB, INPUT);
  time_pre = millis();
}
void encoder::getSPEED(int object){
  time_now = millis();
  value_now = value[object];
  //Serial.println(value_now);
  if((dt = time_now - time_pre) > 50){ // 20Hz
    SPEED_NOW[object] = (float)(value_now - value_pre) / dt;
    PULSE_NOW[object] = value_now;
    //Serial.println(SPEED_NOW[object]);
    time_pre = time_now;
    value_pre = value_now;
  }
}

motor::motor(int P1, int P2){
  POW = P1; DIR = P2;
  pinMode(POW, OUTPUT);
  pinMode(DIR, OUTPUT);
}
void motor::Write(){
  if(PWM > 255 ) PWM = 255; // 上限・下限の設定
  else if(PWM < -255) PWM = -255;
  
  if(PWM >= 0){
    analogWrite(POW, PWM);
    digitalWrite(DIR, LOW);
  }
  else if(PWM < 0){
    analogWrite(POW, -PWM);
    digitalWrite(DIR, HIGH);
  }
  //Serial.println(PWM);
}
// 外部割り込み処理
void counterTWO(){
  nowSig_A[OBJECT_TWO] = digitalRead(PIN_A_TWO);
  nowSig_B[OBJECT_TWO] = digitalRead(PIN_B_TWO);
  counterPULSE(OBJECT_TWO);
}
void counterSIX(){
  nowSig_A[OBJECT_SIX] = digitalRead(PIN_A_SIX);
  nowSig_B[OBJECT_SIX] = digitalRead(PIN_B_SIX);
  counterPULSE(OBJECT_SIX);
}
void counterTEN(){
  nowSig_A[OBJECT_TEN] = digitalRead(PIN_A_TEN);
  nowSig_B[OBJECT_TEN] = digitalRead(PIN_B_TEN);
  counterPULSE(OBJECT_TEN);
}
void counterPULSE(int object){
  if(nowSig_A[object] != oldSig_A[object] || nowSig_B[object] != oldSig_B[object]){
    if     (nowSig_A[object] == 0 && nowSig_B[object] == 0) nowState[object] = 0;
    else if(nowSig_A[object] == 1 && nowSig_B[object] == 0) nowState[object] = 1;
    else if(nowSig_A[object] == 1 && nowSig_B[object] == 1) nowState[object] = 2;
    else if(nowSig_A[object] == 0 && nowSig_B[object] == 1) nowState[object] = 3;

    if((oldState[object] == 0 && nowState[object] == 1) || 
       (oldState[object] == 1 && nowState[object] == 2) ||
       (oldState[object] == 2 && nowState[object] == 3) || 
       (oldState[object] == 3 && nowState[object] == 0)){
      value[object]++;
    }
    else if((oldState[object] == 0 && nowState[object] == 3) || 
            (oldState[object] == 3 && nowState[object] == 2) ||
            (oldState[object] == 2 && nowState[object] == 1) || 
            (oldState[object] == 1 && nowState[object] == 0)){
      value[object]--;
    }
    oldSig_A[object] = nowSig_A[object];
    oldSig_B[object] = nowSig_B[object];
    oldState[object] = nowState[object];    
  }
}
