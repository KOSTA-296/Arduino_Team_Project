#include <SoftwareSerial.h>//소프트웨어 시리얼 라이브러리 추가
#include <Servo.h>
#include "PCF8574.h" //PCF8574 라이브러리 추가
#include <Wire.h> //I2C 통신을 위한 Wire 라이브러리 추가

/* 아두이노 보드 핀 설정 */
#define TXD 2 //TXD를 2번 핀으로 설정
#define RXD 3 //RXD를 3번 핀으로 설정
#define SPEED_L 6
#define DC_IN1_L 4
#define DC_IN2_L 7
#define SPEED_R 11
#define DC_IN1_R 8
#define DC_IN2_R 12

/* PCF8574 모듈 핀 설정 */
#define PCF8574_ADDRESS 0x20
PCF8574 pcf(PCF8574_ADDRESS); //PCF8574 객체 생성, I2C 주소는 0x20으로 설정
const int LEFTECHO  = 0; // PCF8574의 P0 핀을 LeftEcho로 설정
const int LEFTTRIG  = 1; // PCF8574의 P1 핀을 LeftTrig으로 설정
const int RIGHTECHO = 2; // PCF8574의 P2 핀을 RightEcho로 설정
const int RIGHTTRIG = 3; // PCF8574의 P3 핀을 RightTrig으로 설정
const int FRONTECHO = 4; // PCF8574의 P4 핀을 FrontEcho로 설정
const int FRONTTRIG = 5; // PCF8574의 P5 핀을 FrontTrig으로 설정

Servo myservo;  // 서보 모터 객체 9번핀 사용
int pos;        // 서보 모터 위치 값을 저장하는 변수

String Bt_str = "";           // bluetooth 모듈 수신 데이터 저장 변수
int Bt_num;                   // 수신 데이터를 int로 변환
int speed = 150;              // 기본 speed 값 
int command = 5;              // RC카 주행 모드 command 변수
int cnt = 0;
bool Auto_flag = false; // 자율 주행 모드 플래그 변수

SoftwareSerial mySerial(TXD, RXD); //소프트웨어 시리얼 mySerial 객체 선언

void void Set_Motordir(char case);    // dc 모터 방향 지정 함수
void Set_MotorSpeed(int speed);          // dc 모터 속도 지정 함수
void Set_Servoangle(char case);      // 서보 모터를 사용하여 앞바퀴 회전을 제어하는 함수

/*  블루투스 모듈에서 수신한 command에 따라 동작
1 : 좌회전(전진)
2 : 전진
3 : 우회전(후진)
4 : 앞바퀴만 좌로 회전
5 : 정지
6 : 앞바퀴만 우로 회전
7 : 좌회전(후진)
8 : 후진
9 : 우회전(후진)
*/
void manualDrive(int command);               // 수동 조작 함수
void LeftForward();                          // 좌회전(전진)
void Forward();                              // 전진
void RightForward();                         // 우회전(전진)
void TurnLeft();                             // 좌측 바퀴 회전
void Stop();                                 // 정지
void TurnRight();                            // 우측 바퀴 회전
void LeftBack();                             // 좌회전(후진)
void Back();                                 // 후진
void RightBack();                            // 우회전(후진)

void autoDrive();                            // 자율 주행 함수

float getDistance(int trigPin, int echoPin); // 초음파 센서 거리 측정(단위는 cm)
float Get_Distance(char case);   // 방향 거리 측정 함수

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
    // 'c' 문자가 들어올때까지 string을 받아오기
    Bt_str = mySerial.readStringUntil('c');
    Bt_num = Bt_str.toInt();
    // int command = Serial.read() - '0';

    if(Bt_num > 255){ // 255보다 큰 정수는 속도로 활용할 수 없으므로 255로 지정(예외처리)
      Bt_num = 255;
    }

    if(Bt_num > 9){   // 9보다 큰 정수는 속도 변수에 할당
      speed = Bt_num;
    }
    else{
      if(Bt_num == 0){    // 0 -> Mode change 버튼 클릭
        Serial.print("Mode Change\n");
        cnt++;
        Auto_flag = (cnt % 2 == 1);
      }
      else{
        command = Bt_num;
      }
    }
    if(Auto_flag){
      autoDrive();
    }
    else{   // 자율 주행 모드가 아닌 경우 RC카 모드로 동작
      manualDrive(command);
    }
  }
  Servo_direction('C');
  delay(1000); // 0.1초 대기
}

// DC모터 방향 지정 함수(S : 정지, F : 모터 방향 전방, B : 모터 방향 후방)
void Set_Motordir(char case){
  switch(case){
    case 'S':  // 정지
      digitalWrite(DC_IN1_L, LOW);
      digitalWrite(DC_IN2_L, LOW);
      digitalWrite(DC_IN1_R, LOW);
      digitalWrite(DC_IN2_R, LOW);
      break;
    case 'F': // 앞 방향 지정
      digitalWrite(DC_IN1_L, HIGH);
      digitalWrite(DC_IN2_L, LOW);
      digitalWrite(DC_IN1_R, HIGH);
      digitalWrite(DC_IN2_R, LOW);
      break;
    case 'B': // 뒤 방향 지정
      digitalWrite(DC_IN1_L, LOW);
      digitalWrite(DC_IN2_L, HIGH);
      digitalWrite(DC_IN1_R, LOW);
      digitalWrite(DC_IN2_R, HIGH);
      break;
  }
}

// DC모터 속도 지정 함수
void Set_MotorSpeed(int speed){
  analogWrite(SPEED_L, speed);
  analogWrite(SPEED_R, speed);
}

// 서보모터 각도 조절 함수(L : 좌측으로 회전, R : 우측으로 회전, C : 중앙으로 복귀)
void Set_Servoangle(char case){
  switch (case) {
  case 'L':   // 좌측 회전
    if(pos != 15){
      Set_Servoangle('C');
    }
    for(pos = 15; pos <= 30; pos += 1){
      myservo.write(pos);
      delay(50);
    }
    break;
  case 'R':   // 우측 회전
    if(pos != 15){
      Set_Servoangle('C');
    }
    for(pos = 15; pos >= 0; pos -= 1){
      myservo.write(pos);
      delay(50);
    }
    break;
  case 'C':   // 원점으로 복귀
    if(pos > 15){
      for(;pos >= 15; pos -= 1){
        myservo.write(pos);
        delay(50);
      }
    }
    else if(pos < 15){
      for(;pos <= 15; pos += 1){
        myservo.write(pos);
        delay(50);
      }
    }
    break;
  }
}

// 수동 조작 함수
void manualDrive(int cmd) {
  // 수동 제어 모드(1~9)
  switch (cmd) {
  case 1:
    LeftForward();
    break;  
  case 2:
    Forward();
    break;  
  case 3:
    RightForward();
    break;  
  case 4:
    TurnLeft();
    break;  
  case 5:
    Stop();
    break;
  case 6:
    TurnRight();
    break;  
  case 7:
    LeftBack();
    break;  
  case 8:
    Back();
    break;
  case 9:
    RightBack();
    break;
  default:
    Stop();
    break;
  }
}

// 좌회전(전진) : command = 1
void LeftForward(){ 
  Set_Motordir('F');
  Set_Servoangle('L');
  Set_MotorSpeed(speed);
  delay(3000);
  Set_Servoangle('C');
}

// 전진 : command = 2
void Forward(){
  Set_Motordir('F');

  Set_MotorSpeed(speed);
}

// 우회전 : command = 3
void RightForward(){
  Set_Motordir('F');
  Set_Servoangle('R');
  Set_MotorSpeed(speed);
  delay(3000);
  Set_Servoangle('C');
}

// 좌측 바퀴 회전 : command = 4
void TurnLeft(){
  Set_Servoangle('L');
  delay(3000);
  Set_Servoangle('C');
}

// 정지 : command = 5
void Stop(){
  Set_MotorSpeed(0);

  Set_Motordir('S');
}

// 우측 바퀴 회전 : command = 6
void TurnRight(){
  Set_Servoangle('R');
  delay(3000);
  Set_Servoangle('C');
}

// 좌회전(후진) : command = 7
void LeftBack(){
  Set_Motordir('B');
  Set_Servoangle('L');
  Set_MotorSpeed(speed);
  delay(3000);
  Set_Servoangle('C');
}

// 후진 : command = 8;
void Back(){
  Set_Motordir('B');

  Set_MotorSpeed(speed);
}

// 우회전(후진) : command = 9
void RightBack(){
  Set_Motordir('B');
  Set_Servoangle('R');
  Set_MotorSpeed(speed);
  delay(3000);
  Set_Servoangle('C');
}

// 초음파 센서 거리 측정 함수, 단위는 cm
float getDistance(int trigPin, int echoPin) {
  // Trig 핀으로 10μs 펄스 발생
  pcf.write(trigPin, HIGH);
  delayMicroseconds(10);
  pcf.write(trigPin, LOW);

  // Echo 핀 응답 대기 (타임아웃 1초)
  unsigned long startTime = millis();
  while (pcf.read(echoPin) == LOW) {
    if (millis() - startTime > 1000) {
      return -1; // 거리 측정 실패
    }
  }

  // Echo 핀 HIGH 시간 측정 시작
  unsigned long pulseStart = micros();
  while (pcf.read(echoPin) == HIGH) {
    if (millis() - startTime > 1000) {
      return -1; // 거리 측정 실패
    }
  }
  unsigned long pulseEnd = micros();

  // 거리 계산 (음속 343m/s 기준)
  long duration = pulseEnd - pulseStart;
  float distance = duration * 0.0343 / 2; // cm 단위

  return distance;
}

// 방향 거리 측정 함수
float Get_Distance(char case) {
  switch (case) {
  case 'F':
    return Get_Distance(FRONTTRIG, FRONTECHO);
  case 'R':
    return Get_Distance(RIGHTTRIG, RIGHTECHO);
  case 'L':
    return Get_Distance(LEFTTRIG, LEFTECHO);
  default:
    return -1;
  }
}

// 자율 주행 함수
void AutoDrive() {
  // 초음파 센서로 거리 측정 예제
  long frontDistance = Get_Distance('F'); // 앞쪽 거리 측정
  long leftDistance = Get_Distance('L');  // 왼쪽 거리 측정
  long rightDistance = Get_Distance('R'); // 오른쪽 거리 측정

  // 측정된 거리 출력 (디버깅용)
  // Serial.println("Left: " + String(leftDistance) + " cm, Right: " +
  // String(rightDistance) + " cm, Front: " + String(frontDistance) + " cm");

  if (frontDistance <=
      SAFE_DISTANCE) { // 전방 안전거리 이하에 장애물이 있을경우
    // 일단 정지
    Stop();
    delay(500);
    if (rightDistance <=
        SAFE_DISTANCE) { // 오른쪽 안전거리 이하에 장애물이 있을경우
      if (leftDistance <=
          SAFE_DISTANCE) { // 왼쪽 안전거리 이하에 장애물이 있을경우
        // 후진 후 정지
        Back();
        delay(1500);
        Set_Servoangle('R');
        Forward();
        delay(500);
        Set_Servoangle('C');
      } else { // 왼쪽에 장애물이 없을 경우
        // 왼쪽 주행
        Set_Servoangle('L');
        Forward();
        delay(500);
        Set_Servoangle('C');
      }
    } else { // 오른쪽에 장애물이 없을 경우
      // 오른쪽 주행
      Set_Servoangle('R');
      Forward();
      delay(500);
      Set_Servoangle('C');
    }
  } else { // 전방에 장애물이 없을 경우
    // 전진
    Forward();
  }
}
