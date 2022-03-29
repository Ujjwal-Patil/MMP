#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int plus, int minus, int en_a, int en_b) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(en_a,INPUT_PULLUP);
  pinMode(en_b,INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
}

void Motor::rotate(int value) {
  if(value>=0){
    int out = map(value, 0, 100, 0, 225);
    analogWrite(plus,out);
    analogWrite(minus,0);
  }else{
    int out = map(value, 0, -100, 0, 225);
    analogWrite(plus,0);
    analogWrite(minus,out);
  }
}
