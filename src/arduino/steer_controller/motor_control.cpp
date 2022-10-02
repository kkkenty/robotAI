#include "motor_control.h"

float SPEED_NOW[OBJECT_NUM] = {0}; // publishに使用
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

void counter0(){
  nowSig_A[0] = digitalRead(PIN_A_LEFT);
  nowSig_B[0] = digitalRead(PIN_B_LEFT);
  
  if(nowSig_A[0] != oldSig_A[0] || nowSig_B[0] != oldSig_B[0]){
    if     (nowSig_A[0] == 0 && nowSig_B[0] == 0) nowState[0] = 0;
    else if(nowSig_A[0] == 1 && nowSig_B[0] == 0) nowState[0] = 1;
    else if(nowSig_A[0] == 1 && nowSig_B[0] == 1) nowState[0] = 2;
    else if(nowSig_A[0] == 0 && nowSig_B[0] == 1) nowState[0] = 3;

    if((oldState[0] == 0 && nowState[0] == 1) || 
       (oldState[0] == 1 && nowState[0] == 2) ||
       (oldState[0] == 2 && nowState[0] == 3) || 
       (oldState[0] == 3 && nowState[0] == 0)){
      value[0]++;
    }
    else if((oldState[0] == 0 && nowState[0] == 3) || 
            (oldState[0] == 3 && nowState[0] == 2) ||
            (oldState[0] == 2 && nowState[0] == 1) || 
            (oldState[0] == 1 && nowState[0] == 0)){
      value[0]--;
    }
    oldSig_A[0] = nowSig_A[0];
    oldSig_B[0] = nowSig_B[0];
    oldState[0] = nowState[0];    
  }
}
void counter1(){
  nowSig_A[1] = digitalRead(PIN_A_RIGHT);
  nowSig_B[1] = digitalRead(PIN_B_RIGHT);
  
  if(nowSig_A[1] != oldSig_A[1] || nowSig_B[1] != oldSig_B[1]){
    if     (nowSig_A[1] == 0 && nowSig_B[1] == 0) nowState[1] = 0;
    else if(nowSig_A[1] == 1 && nowSig_B[1] == 0) nowState[1] = 1;
    else if(nowSig_A[1] == 1 && nowSig_B[1] == 1) nowState[1] = 2;
    else if(nowSig_A[1] == 0 && nowSig_B[1] == 1) nowState[1] = 3;

    if((oldState[1] == 0 && nowState[1] == 1) || 
       (oldState[1] == 1 && nowState[1] == 2) ||
       (oldState[1] == 2 && nowState[1] == 3) || 
       (oldState[1] == 3 && nowState[1] == 0)){
      value[1]++;
    }
    else if((oldState[1] == 0 && nowState[1] == 3) || 
            (oldState[1] == 3 && nowState[1] == 2) ||
            (oldState[1] == 2 && nowState[1] == 1) || 
            (oldState[1] == 1 && nowState[1] == 0)){
      value[1]--;
    }
    oldSig_A[1] = nowSig_A[1];
    oldSig_B[1] = nowSig_B[1];
    oldState[1] = nowState[1];    
  }
}
