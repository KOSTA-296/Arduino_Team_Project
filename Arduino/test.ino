#include <SoftwareSerial.h>//소프트웨어 시리얼 라이브러리 추가
#include <Servo.h>

// #define TXD 2 //TXD를 2번 핀으로 설정
// #define RXD 3 //RXD를 3번 핀으로 설정
#define SPEED_L 6
#define DC_IN1_L 4
#define DC_IN2_L 7
#define SPEED_R 11
#define DC_IN1_R 8
#define DC_IN2_R 12
#define mySerial Serial

Servo myservo;  // 서보 모터 객체 9번핀 사용
int pos;        // 서보 모터 위치 값을 저장하는 변수

// SoftwareSerial mySerial(TXD, RXD); //소프트웨어 시리얼 mySerial 객체 선언
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

void setup() {
  mySerial.begin(9600); //소프트웨어 시리얼 동기화
  myservo.attach(9);
  pinMode(SPEED_L, OUTPUT);
  pinMode(DC_IN1_L, OUTPUT);
  pinMode(DC_IN2_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(DC_IN1_R, OUTPUT);
  pinMode(DC_IN2_R, OUTPUT);
  // Serial.begin(9600);
}

void loop() { 
  if(mySerial.available()) {
    int command = mySerial.read() - '0';

    switch(command){
      case 0 :  // 정지 
        Stop();
        break;
      case 1 :  // 전진
        Go();
        break;
      case 2 :  // 후진
        Back();
        break;
      case 3 :  // 좌회전(전진)
        LeftGo();
        break;
      case 4 :  // 우회전(전진)
        RightGo();
        break;
      case 5 :  // 좌회전(후진)
        LeftBack();
        break;
      case 6 :  // 우회전(후진)
        RightBack();
        break;
      default :
        Stop();
        break;
    }
  }
  myservo.write(15);
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

  analogWrite(SPEED_L, 255);
  analogWrite(SPEED_R, 255);
}

void Back(){
  digitalWrite(DC_IN1_L, LOW);
  digitalWrite(DC_IN2_L, HIGH);
  digitalWrite(DC_IN1_R, LOW);
  digitalWrite(DC_IN2_R, HIGH);

  analogWrite(SPEED_L, 255);
  analogWrite(SPEED_R, 255);
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
  analogWrite(SPEED_L, 150);
  analogWrite(SPEED_R, 150);
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
  analogWrite(SPEED_L, 150);
  analogWrite(SPEED_R, 150);
  delay(3000);
  for(pos = 0; pos <= 15; pos += 1){
    myservo.write(pos);
    delay(100);
  }
}

// void LeftBack(){
//   digitalWrite(DC_IN1_L, LOW);
//   digitalWrite(DC_IN2_L, HIGH);
//   digitalWrite(DC_IN1_R, LOW);
//   digitalWrite(DC_IN2_R, HIGH);
//   for(pos = 30; pos <= 60; pos += 1){
//     myservo.write(pos);
//     delay(15);
//   }
//   analogWrite(SPEED_L, 150);
//   analogWrite(SPEED_R, 150);
//   delay(3000);
//   for(pos = 60; pos >= 30; pos -= 1){
//     myservo.write(pos);
//     delay(15);
//   }
// }

// void RightBack(){
//   digitalWrite(DC_IN1_L, LOW);
//   digitalWrite(DC_IN2_L, HIGH);
//   digitalWrite(DC_IN1_R, LOW);
//   digitalWrite(DC_IN2_R, HIGH);
//   for(pos = 30; pos >= 0; pos -= 1){
//     myservo.write(pos);
//     delay(15);
//   }
//   analogWrite(SPEED_L, 150);
//   analogWrite(SPEED_R, 150);
//   delay(3000);
//   for(pos = 0; pos <= 30; pos += 1){
//     myservo.write(pos);
//     delay(15);
//   }
// }
