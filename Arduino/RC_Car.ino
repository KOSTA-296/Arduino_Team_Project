#include "PCF8574.h"        // PCF8574 확장 모듈 라이브러리
#include <U8glib.h>         // OLED 라이브러리
#include <Servo.h>          // Servo 라이브러리
#include <SoftwareSerial.h> // 소프트웨어 시리얼 라이브러리
#include <Wire.h>           // I2C 통신 라이브러리(OLED, PCF8574)

/* 아두이노 보드 핀 설정 */
#define TXD 2                 // TXD 핀
#define RXD 3                 // RXD 핀
#define SERVO_PIN 9           // 서보 모터 PWM 핀
#define SPEED_L 6             // 좌측 모터 PWM 핀`
#define DC_IN1_L 4            // 좌측 모터 IN1
#define DC_IN2_L 7            // 좌측 모터 IN2
#define SPEED_R 11            // 우측 모터 PWM 핀
#define DC_IN3_R 8            // 우측 모터 IN3
#define DC_IN4_R 12           // 우측 모터 IN4
#define LEFT_LED 10           // 좌측 LED 핀
#define RIGHT_LED 5           // 우측 LED 핀
#define FRONT_LED 13          // 정면 LED 핀

#define PCF8574_ADDRESS 0x20  // PCF8574 I2C 주소

// 장애물 감지를 위한 임계값 (cm) -> RC카에 알맞은 임계값을 찾아서 수정하여 사용 가능
constexpr int OBSTACLE_THRESHOLD = 30;             // best : 30cm
// 왼쪽 or 오른쪽에만 존재하는 장애물 감지를 위한 임계값 (cm)
constexpr int LR_THRESHOLD = 20;                   // best : 20cm

/* PCF8574 확장 모듈 핀 설정 (초음파 센서) */
PCF8574 pcf(PCF8574_ADDRESS);
const int LEFTECHO  = 0;   // PCF8574 P0 (좌측 Echo)
const int LEFTTRIG  = 1;   // PCF8574 P1 (좌측 Trig)
const int RIGHTECHO = 2;   // PCF8574 P2 (우측 Echo)
const int RIGHTTRIG = 3;   // PCF8574 P3 (우측 Trig)
const int FRONTECHO = 4;   // PCF8574 P4 (전방 Echo)
const int FRONTTRIG = 5;   // PCF8574 P5 (전방 Trig)

/* OLED 설정 */
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

/* ------------------------------------------------------------------
   1. 모터 제어 클래스 (MotorController)
   - 모터 방향 제어, 속도 제어 클래스
------------------------------------------------------------------- */
class MotorController {
  int speedPinLeft, in1Left, in2Left;
  int speedPinRight, in3Right, in4Right;
public:
  MotorController(int spdL, int i1L, int i2L, int spdR, int i3R, int i4R)
    : speedPinLeft(spdL), in1Left(i1L), in2Left(i2L),
      speedPinRight(spdR), in3Right(i3R), in4Right(i4R) {}

  void begin() {
    pinMode(speedPinLeft, OUTPUT);
    pinMode(in1Left, OUTPUT);
    pinMode(in2Left, OUTPUT);
    pinMode(speedPinRight, OUTPUT);
    pinMode(in3Right, OUTPUT);
    pinMode(in4Right, OUTPUT);
  }
  
  // 모터 방향 설정 ('F': 전진, 'B': 후진, 'S': 정지)
  void setDirection(char dir) {
    switch (dir) {
      case 'F': // 전진
        digitalWrite(in1Left, HIGH);
        digitalWrite(in2Left, LOW);
        digitalWrite(in3Right, HIGH);
        digitalWrite(in4Right, LOW);
        break;
      case 'B': // 후진
        digitalWrite(in1Left, LOW);
        digitalWrite(in2Left, HIGH);
        digitalWrite(in3Right, LOW);
        digitalWrite(in4Right, HIGH);
        break;
      case 'S': // 정지
      default:
        digitalWrite(in1Left, LOW);
        digitalWrite(in2Left, LOW);
        digitalWrite(in3Right, LOW);
        digitalWrite(in4Right, LOW);
        break;
    }
  }
  
  // 모터 속도 설정 (0 ~ 255)
  void setSpeed(int speed) {
    analogWrite(speedPinLeft, speed);
    analogWrite(speedPinRight, speed);
  }
};

/* ------------------------------------------------------------------
   2. 서보 제어 클래스 (ServoController)
   - 중앙을 22도로 설정, 좌측 회전은 2도, 우측 회전은 42도로 조절
------------------------------------------------------------------- */
class ServoController {
  Servo servo;
  int servoPin;
  int pos;  // 현재 서보 각도
  static const int centerPos = 22;
  static const int leftPos = 2;
  static const int rightPos = 42;
public:
  ServoController(int sPin) : servoPin(sPin), pos(centerPos) {}

  void begin() {
    servo.attach(servoPin);
    servo.write(pos);
  }
  
  // 부드러운 각도 이동 (delay 20ms)
  void setAngleSmoothly(int start, int end) {
    if (start < end) {
      for (int i = start; i <= end; i++) {
        servo.write(i);
        delay(20);
      }
    } else {
      for (int i = start; i >= end; i--) {
        servo.write(i);
        delay(20);
      }
    }
    pos = end;
  }

  // 중앙으로 복귀
  void center() {
    if (pos != centerPos) {
      setAngleSmoothly(pos, centerPos);
    }
  }

  // 좌측 회전 : 22 -> 2도
  void turnLeft() {
    setAngleSmoothly(pos, leftPos);
  }
  
  // 우측 회전 : 22 -> 42도
  void turnRight() {
    setAngleSmoothly(pos, rightPos);
  }
};

/* ------------------------------------------------------------------
   3. 초음파 센서 클래스 (UltrasonicSensor)
   - 초음파 센서를 활용한 거리 측정 클래스
------------------------------------------------------------------- */
class UltrasonicSensor {
  PCF8574* pcfPtr;
  int trigPin;
  int echoPin;
public:
  UltrasonicSensor(PCF8574* pcf, int trig, int echo)
    : pcfPtr(pcf), trigPin(trig), echoPin(echo) {}

  // 거리 측정 (cm 단위)
  float getDistance() {
    pcfPtr->write(trigPin, HIGH);
    delayMicroseconds(10);
    pcfPtr->write(trigPin, LOW);

    unsigned long startTime = millis();
    // 타임아웃 1초
    while (pcfPtr->read(echoPin) == LOW) {
      if (millis() - startTime > 1000) return -1;
    }

    unsigned long pulseStart = micros();
    while (pcfPtr->read(echoPin) == HIGH) {
      if (millis() - startTime > 1000) return -1;
    }
    unsigned long pulseEnd = micros();

    long duration = pulseEnd - pulseStart;
    float distance = duration * 0.0343 / 2;
    if(distance > 400.0){
      distance = 400.0;
    }
    return distance;
  }
};

/* ------------------------------------------------------------------
   4. OLED 클래스 (OLED)
   - 초음파 센서 거리 측정 데이터 출력용
   - 이후 추가로 출력할 데이터가 있으면 함수 추가
------------------------------------------------------------------- */
class OLED {
public:
  OLED() { }
  // 좌측, 전방, 우측 순서로 출력
  void print_UltraSensor(long left, long front, long right){
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_fub14);
      u8g.setPrintPos(0, 15);
      u8g.print("left : ");
      if (left > 100){
        u8g.print("Clear");
      }
      else{
        u8g.print(left);
        u8g.print("cm");
      }
      u8g.setPrintPos(0, 35);
      u8g.print("Front : ");
      if (front > 100){
        u8g.print("Clear");
      }
      else{
        u8g.print(front);
        u8g.print("cm");
      }
      u8g.setPrintPos(0, 55);
      u8g.print("right : ");
      if (right > 100){
        u8g.print("Clear");
      }
      else{
        u8g.print(right);
        u8g.print("cm");
      }
    } while(u8g.nextPage());
  }
};

/* ------------------------------------------------------------------
   5. LED 클래스 (OLED)
   - LED 동작 제어 클래스
------------------------------------------------------------------- */
class LED {
  int leftPin;
  int frontPin;
  int rightPin;
public:
  LED(int left, int front, int right) : leftPin(left), frontPin(front), rightPin(right) { }
  // 핀 설정
  void begin(){
    pinMode(leftPin, OUTPUT);
    pinMode(frontPin, OUTPUT);
    pinMode(rightPin, OUTPUT);
    digitalWrite(leftPin, LOW);
    digitalWrite(frontPin, LOW);
    digitalWrite(rightPin, LOW);
  }
  //좌측 LED on, off
  void LeftOn(){
    digitalWrite(leftPin, HIGH);
  }
  void LeftOff(){
    digitalWrite(leftPin, LOW);
  }
  // 전면 LED on, off
  void FrontOn(){
    digitalWrite(frontPin, HIGH);
  }
  void FrontOff(){
    digitalWrite(frontPin, LOW);
  }
  // 우측 LED on, off
  void RightOn(){
    digitalWrite(rightPin, HIGH);
  }
  void RightOff(){
    digitalWrite(rightPin, LOW);
  }
};

/* ------------------------------------------------------------------
   6. RC카 제어 클래스 (RC_Car)
   - 블루투스 명령에 따라 수동 모드 또는 자동 모드를 실행
   - 자동 모드에서는 DC모터는 계속 전진하며, 초음파 센서 정보를 0.2초마다 받아 장애물에 따라 조향합니다.
------------------------------------------------------------------- */
class RC_Car {
  MotorController motor;
  ServoController servo;
  UltrasonicSensor sensorFront;
  UltrasonicSensor sensorLeft;
  UltrasonicSensor sensorRight;
  LED led;
  OLED oled;
  SoftwareSerial* btSerial;   // 블루투스 통신용 소프트웨어 시리얼
  int speed;                  // DC모터 속도
  int command;                // 수동 조종 명령 (1~9)
  bool autoMode;              // 자동 모드 여부 (false: RC카 모드, true: autoMode)
  long front, left, right;    // 초음파 센서가 측정 한 거리 저장용 변수
public:
  RC_Car(SoftwareSerial* serial)
    : motor(SPEED_L, DC_IN1_L, DC_IN2_L, SPEED_R, DC_IN3_R, DC_IN4_R),
      servo(SERVO_PIN),
      sensorFront(&pcf, FRONTTRIG, FRONTECHO),
      sensorLeft(&pcf, LEFTTRIG, LEFTECHO),
      sensorRight(&pcf, RIGHTTRIG, RIGHTECHO),
      led(LEFT_LED, RIGHT_LED, FRONT_LED),
      oled(),
      btSerial(serial),
      speed(150),
      command(5),
      autoMode(false),
      front(0),
      left(0),
      right(0)
  {}

  void begin() {
    motor.begin();
    servo.begin();
    // PCF8574 트리거 핀 LOW 설정
    pcf.write(LEFTTRIG, LOW);
    pcf.write(RIGHTTRIG, LOW);
    pcf.write(FRONTTRIG, LOW);
    // LED 핀 설정
    led.begin();
  }
  
  // 블루투스 명령 처리 및 모드 업데이트  
  // 앱인벤터에서 Mode change 버튼이 터치되면 자율주행모드가 토글됩니다.
  void update() {
    if (btSerial->available()) {
      String btStr = btSerial->readStringUntil('c'); // 'c'까지 읽기
      int btNum = btStr.toInt();
      // 10 이상의 값은 속도 업데이트로 처리
      if (btNum > 9) {
        speed = btNum;
      }
      // 0번 명령: 모드 토글
      else if (btNum == 0) {
        autoMode = !autoMode;
        delay(200);  // 모드 전환 debounce용 짧은 딜레이
      }
      // 수동 명령 (1~9)은 autoMode가 아닐 때만 반영
      else if (!autoMode) {
        command = btNum;
        manualDrive(command);
      }
    }
    front = sensorFront.getDistance();
    left = sensorLeft.getDistance();
    right = sensorRight.getDistance();
    oled.print_UltraSensor(left, front, right);
    // autoMode 플래그에 따라 실행
    if (autoMode) {
      autoDrive(left, front, right);
      delay(200);  // 0.2초 주기로 센서 읽기 및 판단
    }
    else {
      // 수동 모드인 경우 주기적으로 명령 실행 (필요 시 delay 조정)
      delay(1000);
    }
  }
  
  // 수동 조종 함수 case별로 동작
  void manualDrive(int cmd) {
    switch (cmd) {
      case 1: leftForward();  break;
      case 2: forward();      break;
      case 3: rightForward(); break;
      case 4: turnLeft();     break;
      case 5: stop();         break;
      case 6: turnRight();    break;
      case 7: leftBack();     break;
      case 8: back();         break;
      case 9: rightBack();    break;
      default: stop();        break;
    }
  }
  
  /* 
    자동차 동작 함수
    ex) 좌회전 버튼 누르기 : 좌회전 동작 -> 좌회전 버튼 떼기 : 정지 동작
  */
  void leftForward() {
    motor.setDirection('F');
    servo.turnLeft();
    motor.setSpeed(speed);
    led.LeftOn();
  }
  
  void forward() {
    servo.center();
    motor.setDirection('F');
    motor.setSpeed(speed);
    led.FrontOn();
  }
  
  void rightForward() {
    motor.setDirection('F');
    servo.turnRight();
    motor.setSpeed(speed);
    led.RightOn();
  }
  
  void turnLeft() {
    servo.turnLeft();
    led.LeftOn();
  }
  
  void stop() {
    motor.setSpeed(0);
    motor.setDirection('S');
    servo.center();
    led.FrontOff();
    led.LeftOff();
    led.RightOff();
  }
  
  void turnRight() {
    servo.turnRight();
    led.RightOn();
  }
  
  void leftBack() {
    motor.setDirection('B');
    servo.turnLeft();
    motor.setSpeed(speed);
    led.LeftOn();
  }
  
  void back() {
    servo.center();
    motor.setDirection('B');
    motor.setSpeed(speed);
  }
  
  void rightBack() {
    motor.setDirection('B');
    servo.turnRight();
    motor.setSpeed(speed);
    led.RightOn();
  }
  
  void autoLeft(){
    led.LeftOn();
    motor.setDirection('B');
    motor.setSpeed(speed);
    delay(500);
    motor.setDirection('F');
    servo.turnLeft();
  }

  void autoRight(){
    led.RightOn();
    motor.setDirection('B');
    motor.setSpeed(speed);
    delay(500);
    motor.setDirection('F');
    servo.turnRight();
  }

  // 자동 주행 함수  
  // DC모터는 계속 전진하면서 0.2초마다 센서 값을 읽어 아래 규칙에 따라 조향합니다.
  void autoDrive(long left, long front, long right) {
    // DC모터는 항상 전진 상태로 설정
    motor.setDirection('F');
    motor.setSpeed(speed);
    
    /* 
      조건 판단 
      (우선순위: 세 센서 모두 장애물 > front+right > front+left > front 단독 = left 단독 = right 단독)
    */
    if (front < OBSTACLE_THRESHOLD && left < OBSTACLE_THRESHOLD && right < OBSTACLE_THRESHOLD) {
      // 세 센서 모두 30cm 이내: 후진 동작 (2000ms)
      stop();
      // Serial.println("All obstacles! Reverse");
      servo.center();
      motor.setDirection('B');
      motor.setSpeed(speed);
      delay(2000);
      motor.setDirection('F');
    }
    else if (front < OBSTACLE_THRESHOLD && right < OBSTACLE_THRESHOLD) {
      // 전방 및 오른쪽 장애물: 좌측 회전
      stop();
      // Serial.println("Front & Right blocked. Turn Left");
      autoLeft();
      delay(100);
    }
    else if (front < OBSTACLE_THRESHOLD && left < OBSTACLE_THRESHOLD) {
      // 전방 및 왼쪽 장애물: 우측 회전
      stop();
      // Serial.println("Front & Left blocked. Turn Right");
      autoRight();
      delay(100);
    }
    else if (front < OBSTACLE_THRESHOLD) {
      // 전방 장애물만 존재할 때: 기본적으로 우측으로 회전
      stop();
      if(left < right){
        autoRight();
      }
      else{
        autoLeft();
      }
      // Serial.println("Front blocked. Turn Right");
      delay(100);
    }
    else if (left < LR_THRESHOLD) {     // 왼쪽에만 장애물 있으면 오른쪽으로 짧게 회전
      servo.turnRight();
    }
    else if (right < LR_THRESHOLD){     // 오른쪽에만 장애물 있으면 왼쪽으로 짧게 회전
      servo.turnLeft();
    }
    else {
      // 장애물이 없으면 중앙 유지
      servo.center();
    }
  }
};

/* ------------------------------------------------------------------
   전역 객체 생성 및 초기화
------------------------------------------------------------------- */
SoftwareSerial btSerial(TXD, RXD);  // 블루투스 통신용 SoftwareSerial 전역 객체 생성
RC_Car car(&btSerial);              // RC_Car class 전역 객체 생성
void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  Wire.begin();
  pcf.begin();
  car.begin();
}

void loop() {
  car.update();
}
