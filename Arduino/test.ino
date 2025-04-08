#include <SoftwareSerial.h>//소프트웨어 시리얼 라이브러리 추가
#include <Servo.h>
#include "PCF8574.h" //PCF8574 라이브러리 추가
#include <Wire.h> //I2C 통신을 위한 Wire 라이브러리 추가

#define PCF8574_ADDRESS 0x20

PCF8574 pcf(PCF8574_ADDRESS); //PCF8574 객체 생성, I2C 주소는 0x20으로 설정

#define TXD 2 //TXD를 2번 핀으로 설정
#define RXD 3 //RXD를 3번 핀으로 설정
#define SPEED_L 6
#define DC_IN1_L 4
#define DC_IN2_L 7
#define SPEED_R 11
#define DC_IN1_R 8
#define DC_IN2_R 12

const int LEFTECHO  = 0; // PCF8574의 P0 핀을 LeftEcho로 설정
const int LEFTTRIG  = 1; // PCF8574의 P1 핀을 LeftTrig으로 설정
const int RIGHTECHO = 2; // PCF8574의 P2 핀을 RightEcho로 설정
const int RIGHTTRIG = 3; // PCF8574의 P3 핀을 RightTrig으로 설정
const int FRONTECHO = 4; // PCF8574의 P4 핀을 FrontEcho로 설정
const int FRONTTRIG = 5; // PCF8574의 P5 핀을 FrontTrig으로 설정

Servo myservo;  // 서보 모터 객체 9번핀 사용
int pos;        // 서보 모터 위치 값을 저장하는 변수
String Bs = "";
int speed = 150;
int Bn;
int num = 5;
int cnt = 0;
bool flag_1 = false;
bool Mode_flag = false;

SoftwareSerial mySerial(TXD, RXD); //소프트웨어 시리얼 mySerial 객체 선언
/*
0 : 정지
1 : 전진
2 : 후진
3 : 좌회전(전진)
4 : 우회전(후진)
5 : 좌회전(후진)
6 : 우회전(후진)
*/
void Stop();
void Go();
void Back();
void LeftGo();
void RightGo();
void LeftBack();
void RightBack();
void Right();
void Left();
float getDistance(int trigPin, int echoPin);

void setup() {
  mySerial.begin(9600); //소프트웨어 시리얼 동기화
  myservo.attach(9);
  Wire.begin(); //I2C 통신 시작
  pcf.begin(); //PCF8574 초기화
  pinMode(SPEED_L, OUTPUT);
  pinMode(DC_IN1_L, OUTPUT);
  pinMode(DC_IN2_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(DC_IN1_R, OUTPUT);
  pinMode(DC_IN2_R, OUTPUT);
  Serial.begin(9600);
  pcf.write(LEFTTRIG, LOW); // LeftTrig 핀을 LOW로 설정
  pcf.write(RIGHTTRIG, LOW); // RightTrig 핀을 LOW로 설정
  pcf.write(FRONTTRIG, LOW); // FrontTrig 핀을 LOW로 설정
}

void loop() { 
  if(mySerial.available()) {
  // if(Serial.available()) {
    Bs = mySerial.readStringUntil('c');
    Bn = Bs.toInt();
    // int command = Serial.read() - '0';
    if(Bn > 255){
      Bn = 255;
    }

    if(Bn > 9){
      speed = Bn;
    }
    else{
      if(Bn == 0){
        Serial.print("Mode Change\n");
        cnt++;
        if(cnt % 2 == 1){
          Mode_flag = true;
        }
        else{
          Mode_flag = false;
        }
      }
      else{
        num = Bn;
      }
    }
    if(Mode_flag){
      Serial.println("자율 주행 모드");
    }
    else{
      switch(num){
      case 5 :  // 정지 
        Stop();
        Serial.println("Stop command");
        break;
      case 2 :  // 전진
        Go();
        Serial.println("Go command");
        break;
      case 8 :  // 후진
        Back();
        Serial.println("Back command");
        break;
      case 1 :  // 좌회전(전진)
        Serial.println("Left Go command");
        LeftGo();
        break;
      case 3 :  // 우회전(전진)
        Serial.println("Right Go command");
        RightGo();
        break;
      case 7 :  // 좌회전(후진)
        Serial.println("Left Back command");
        LeftBack();
        break;
      case 9 :  // 우회전(후진)
        Serial.println("Right Back command");
        RightBack();
        break;
      case 4 : // 바퀴 좌측 회전
        Serial.println("Left turn");
        Left();
        break;
      case 6:
        Serial.println("Right turn");
        Right();
        break;
      default :
        Stop();
        break;
      }
    }
  }
  myservo.write(15);
  long leftDistance = getDistance(LEFTTRIG, LEFTECHO); // 왼쪽 거리 측정
  long rightDistance = getDistance(RIGHTTRIG, RIGHTECHO); // 오른쪽 거리 측정
  long frontDistance = getDistance(FRONTTRIG, FRONTECHO); // 앞쪽 거리 측정
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.print(rightDistance);
  Serial.print(" cm, Front: ");
  Serial.print(frontDistance);
  Serial.println(" cm");
  delay(1000); // 0.1초 대기
}

void Stop(){
  analogWrite(SPEED_L, 0);
  analogWrite(SPEED_R, 0);

  digitalWrite(DC_IN1_L, LOW);
  digitalWrite(DC_IN2_L, LOW);
  digitalWrite(DC_IN1_R, LOW);
  digitalWrite(DC_IN2_R, LOW);
}

void Go(){
  digitalWrite(DC_IN1_L, HIGH);
  digitalWrite(DC_IN2_L, LOW);
  digitalWrite(DC_IN1_R, HIGH);
  digitalWrite(DC_IN2_R, LOW);

  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
}

void Back(){
  digitalWrite(DC_IN1_L, LOW);
  digitalWrite(DC_IN2_L, HIGH);
  digitalWrite(DC_IN1_R, LOW);
  digitalWrite(DC_IN2_R, HIGH);

  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
}

void LeftGo(){ 
  digitalWrite(DC_IN1_L, HIGH);
  digitalWrite(DC_IN2_L, LOW);
  digitalWrite(DC_IN1_R, HIGH);
  digitalWrite(DC_IN2_R, LOW);
  for(pos = 15; pos <= 30; pos += 1){
    myservo.write(pos);
    delay(100);
  }
  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
  delay(3000);
  for(pos = 30; pos >= 15; pos -= 1){
    myservo.write(pos);
    delay(100);
  }
  analogWrite(SPEED_L, 0);
}

void RightGo(){
  digitalWrite(DC_IN1_L, HIGH);
  digitalWrite(DC_IN2_L, LOW);
  digitalWrite(DC_IN1_R, HIGH);
  digitalWrite(DC_IN2_R, LOW);
  for(pos = 15; pos >= 0; pos -= 1){
    myservo.write(pos);
    delay(100);
  }
  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
  delay(3000);
  for(pos = 0; pos <= 15; pos += 1){
    myservo.write(pos);
    delay(100);
  }
}

void LeftBack(){
  digitalWrite(DC_IN1_L, LOW);
  digitalWrite(DC_IN2_L, HIGH);
  digitalWrite(DC_IN1_R, LOW);
  digitalWrite(DC_IN2_R, HIGH);
  for(pos = 30; pos <= 60; pos += 1){
    myservo.write(pos);
    delay(15);
  }
  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
  delay(3000);
  for(pos = 60; pos >= 30; pos -= 1){
    myservo.write(pos);
    delay(15);
  }
}

void RightBack(){
  digitalWrite(DC_IN1_L, LOW);
  digitalWrite(DC_IN2_L, HIGH);
  digitalWrite(DC_IN1_R, LOW);
  digitalWrite(DC_IN2_R, HIGH);
  for(pos = 30; pos >= 0; pos -= 1){
    myservo.write(pos);
    delay(15);
  }
  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
  delay(3000);
  for(pos = 0; pos <= 30; pos += 1){
    myservo.write(pos);
    delay(15);
  }
}

void Right(){
  for(pos = 15; pos >= 0; pos -= 1){
    myservo.write(pos);
    delay(100);
  }
  delay(3000);
  for(pos = 0; pos <= 15; pos += 1){
    myservo.write(pos);
    delay(100);
  }
}
void Left(){
  for(pos = 15; pos <= 30; pos += 1){
    myservo.write(pos);
    delay(100);
  }
  delay(3000);
  for(pos = 30; pos >= 15; pos -= 1){
    myservo.write(pos);
    delay(100);
  }
}

float getDistance(int trigPin, int echoPin) {
  pcf.write(trigPin, HIGH);
  delayMicroseconds(10);
  pcf.write(trigPin, LOW);
  unsigned long startTime = millis();
  while(pcf.read(echoPin) == LOW) {
    if (millis() - startTime > 1000) { // 1초 이상 대기하면 타임아웃
      return -1; // 거리 측정 실패
    }
  }
  unsigned long pulseStart = micros(); // Echo 핀에서 HIGH 상태가 시작된 시간
  while(pcf.read(echoPin) == HIGH) {
    if (millis() - startTime > 1000) { // 100ms 이상 대기하면 타임아웃
      return -1; // 거리 측정 실패
    }
  }
  unsigned long pulseEnd = micros(); // Echo 핀에서 HIGH 상태가 끝난 시간
  long duration = pulseEnd - pulseStart; // Echo 핀에서 HIGH 상태가 된 시간
  // Echo 핀에서 HIGH 상태가 되는 시간을 측정
  
  float distance = duration * 0.0343 / 2; // cm 단위 계산
  
  return distance;
}