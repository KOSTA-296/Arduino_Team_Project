#include "PCF8574.h"        // PCF8574 라이브러리
#include <Servo.h>          // Servo 라이브러리
#include <SoftwareSerial.h> // 소프트웨어 시리얼 라이브러리
#include <Wire.h>           // I2C 통신 라이브러리

/* 아두이노 보드 핀 설정 */
#define TXD 2                 // TXD 핀
#define RXD 3                 // RXD 핀
#define SPEED_L 6             // 좌측 모터 PWM 핀
#define DC_IN1_L 4            // 좌측 모터 IN1
#define DC_IN2_L 7            // 좌측 모터 IN2
#define SPEED_R 11            // 우측 모터 PWM 핀
#define DC_IN1_R 8            // 우측 모터 IN1
#define DC_IN2_R 12           // 우측 모터 IN2
#define SAFE_DISTANCE 30      // 안전 거리 (cm 단위)
#define PCF8574_ADDRESS 0x20  // PCF8574 I2C 주소

/* PCF8574 확장 모듈 핀 설정 (초음파 센서) */
PCF8574 pcf(PCF8574_ADDRESS);
const int LEFTECHO  = 0;   // PCF8574의 P0 (좌측 Echo)
const int LEFTTRIG  = 1;   // PCF8574의 P1 (좌측 Trig)
const int RIGHTECHO = 2;   // PCF8574의 P2 (우측 Echo)
const int RIGHTTRIG = 3;   // PCF8574의 P3 (우측 Trig)
const int FRONTECHO = 4;   // PCF8574의 P4 (전방 Echo)
const int FRONTTRIG = 5;   // PCF8574의 P5 (전방 Trig)

/* 자율 주행 모드 상태 변수 */
enum AutoState {
  STATE_FORWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_BACKWARD,
  STATE_WAIT
};

/* ------------------------------------------------------------------
   1. 모터 제어를 위한 클래스 (MotorController)
   - 좌/우 모터의 방향과 속도를 제어하는 기능을 캡슐화
------------------------------------------------------------------- */
class MotorController {
  int speedPinLeft, in1Left, in2Left;
  int speedPinRight, in1Right, in2Right;
public:
  MotorController(int spdL, int i1L, int i2L, int spdR, int i1R, int i2R)
    : speedPinLeft(spdL), in1Left(i1L), in2Left(i2L),
      speedPinRight(spdR), in1Right(i1R), in2Right(i2R) {}

  void begin() {
    pinMode(speedPinLeft, OUTPUT);
    pinMode(in1Left, OUTPUT);
    pinMode(in2Left, OUTPUT);
    pinMode(speedPinRight, OUTPUT);
    pinMode(in1Right, OUTPUT);
    pinMode(in2Right, OUTPUT);
  }
  
  // 모터 방향 설정 ('F': 전진, 'B': 후진, 'S': 정지)
  void setDirection(char dir) {
    switch (dir) {
      case 'F': // 전진
        digitalWrite(in1Left, HIGH);
        digitalWrite(in2Left, LOW);
        digitalWrite(in1Right, HIGH);
        digitalWrite(in2Right, LOW);
        break;
      case 'B': // 후진
        digitalWrite(in1Left, LOW);
        digitalWrite(in2Left, HIGH);
        digitalWrite(in1Right, LOW);
        digitalWrite(in2Right, HIGH);
        break;
      case 'S': // 정지
      default:
        digitalWrite(in1Left, LOW);
        digitalWrite(in2Left, LOW);
        digitalWrite(in1Right, LOW);
        digitalWrite(in2Right, LOW);
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
   2. 서보 제어를 위한 클래스 (ServoController)
   - 서보 모터로 앞바퀴 회전을 부드럽게 제어
------------------------------------------------------------------- */
class ServoController {
  Servo servo;
  int servoPin;
  int pos;  // 현재 서보 각도 (기본 15도)
public:
  ServoController(int sPin) : servoPin(sPin), pos(15) {}

  void begin() {
    servo.attach(servoPin);
    servo.write(pos);
  }
  
  // start에서 end까지 1도씩 움직이면서 delay (부드러운 회전)
  void setAngleSmoothly(int start, int end) {
    if (start < end) {
      for (int i = start; i <= end; i++) {
        servo.write(i);
        delay(50);
      }
    } else {
      for (int i = start; i >= end; i--) {
        servo.write(i);
        delay(50);
      }
    }
    pos = end;
  }

  // 중앙(원점)으로 복귀
  void center() {
    if (pos != 20) {
    setAngleSmoothly(pos, 20);
    }
  }

  // 좌측 회전 (15도에서 0도로)
  void turnLeft() {
    if (pos != 20) {
      center();
    }
    setAngleSmoothly(pos, 0);
  }
  
  // 우측 회전 (15도에서 30도로)
  void turnRight() {
    if (pos != 20) {  // 원점(15도)으로 복귀 후 좌측 회전
      center();
    }
    setAngleSmoothly(pos, 40);
  }
};

/* ------------------------------------------------------------------
   3. 초음파 센서를 위한 클래스 (UltrasonicSensor)
   - PCF8574 모듈을 이용하여 확장된 디지털핀을 사용한 초음파 센서로 거리를 측정
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
    // Trig 핀으로 10μs 펄스 발생
    pcfPtr->write(trigPin, HIGH);
    delayMicroseconds(10);
    pcfPtr->write(trigPin, LOW);

    unsigned long startTime = millis();
    // Echo 핀이 HIGH가 될 때까지 대기 (타임아웃 1초)
    while (pcfPtr->read(echoPin) == LOW) {
      if (millis() - startTime > 1000) {
        return -1;
      }
    }
    
    unsigned long pulseStart = micros();
    while (pcfPtr->read(echoPin) == HIGH) {
      if (millis() - startTime > 1000) {
        return -1;
      }
    }
    unsigned long pulseEnd = micros();
    
    long duration = pulseEnd - pulseStart;
    float distance = duration * 0.0343 / 2; // cm 단위
    return distance;
  }
};

/* ------------------------------------------------------------------
   4. RC카 제어 클래스 (RC_Car)
   - 모터, 서보, 초음파 센서, 블루투스 수신을 종합하여 
     수동 및 자율 주행 모드를 구현
------------------------------------------------------------------- */
class RC_Car {
  MotorController motor;
  ServoController servo;
  UltrasonicSensor sensorFront;
  UltrasonicSensor sensorLeft;
  UltrasonicSensor sensorRight;
  SoftwareSerial* btSerial;  // 블루투스 통신용 소프트웨어 시리얼
  bool autoMode;             // false: 수동, true: 자율 주행
  int speed;                 // 현재 모터 속도
  int command;               // 현재 동작 명령 (1~9)
  int modeToggleCount;       // 모드 변경 카운트
  AutoState currentState = STATE_FORWARD;
  unsigned long stateStartTime = 0;
public:
  RC_Car(SoftwareSerial* serial)
    : motor(SPEED_L, DC_IN1_L, DC_IN2_L, SPEED_R, DC_IN1_R, DC_IN2_R),
      servo(9),
      sensorFront(&pcf, FRONTTRIG, FRONTECHO),
      sensorLeft(&pcf, LEFTTRIG, LEFTECHO),
      sensorRight(&pcf, RIGHTTRIG, RIGHTECHO),
      btSerial(serial),
      autoMode(false),
      speed(150),
      command(5),
      modeToggleCount(0)
  {}
  
  // 각 구성요소 초기화
  void begin() {
    motor.begin();
    servo.begin();
    // PCF8574의 트리거 핀 LOW 설정
    pcf.write(LEFTTRIG, LOW);
    pcf.write(RIGHTTRIG, LOW);
    pcf.write(FRONTTRIG, LOW);
  }
  
  // 블루투스 명령 및 주행 모드 업데이트 (메인 loop()에서 호출)
  void update() {
    if (btSerial->available()) {
      String btStr = btSerial->readStringUntil('c');  // 'c'까지 읽기
      int btNum = btStr.toInt();
      if (btNum > 255) {         // 최대 속도 제한
        btNum = 255;
      }
      if (btNum > 9) {           // 10 이상이면 속도로 인식
        speed = btNum;
      } else {
        if (btNum == 0) {        // 모드 변경 명령 (0)
          Serial.println("Mode Change");
          modeToggleCount++;
          autoMode = (modeToggleCount % 2 == 1);
        } else {
          command = btNum;       // 1~9 명령
        }
      }
    }
    
    if (autoMode) {
      autoDrive();
    } else {
      manualDrive(command);
    }
    int front1 = sensorFront.getDistance();
    int left1 = sensorLeft.getDistance();
    int right1 = sensorRight.getDistance();
    Serial.print("Front Sensor : ");
    Serial.print(front1);
    Serial.println("cm");
    Serial.print("Left Sensor : ");
    Serial.print(left1);
    Serial.println("cm");
    Serial.print("Right Sensor : ");
    Serial.print(right1);
    Serial.println("cm");
    delay(500);
  }
  
  // 수동 조종 동작 (명령에 따라 동작)
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
  
  // 동작 함수들 (각 동작은 모터 방향, 서보 회전, 속도 제어, 지연 포함)
  void leftForward() {
    motor.setDirection('F');
    servo.turnLeft();
    motor.setSpeed(speed);
  }
  
  void forward() {
    motor.setDirection('F');
    servo.center();
    motor.setSpeed(speed);
  }
  
  void rightForward() {
    motor.setDirection('F');
    servo.turnRight();
    motor.setSpeed(speed);
  }
  
  void turnLeft() {
    servo.turnLeft();
  }
  
  void stop() {
    servo.center();
    motor.setSpeed(0);
    motor.setDirection('S');
  }
  
  void turnRight() {
    servo.turnRight();
  }
  
  void leftBack() {
    motor.setDirection('B');
    servo.turnLeft();
    motor.setSpeed(speed);
  }
  
  void back() {
    motor.setDirection('B');
    servo.center();
    motor.setSpeed(speed);
  }
  
  void rightBack() {
    motor.setDirection('B');
    servo.turnRight();
    motor.setSpeed(speed);
  }
  
  // 자율 주행 로직 FSM 스타일로 개선
  void autoDrive() {
    unsigned long currentTime = millis();
    int front = sensorFront.getDistance();
    int left  = sensorLeft.getDistance();
    int right = sensorRight.getDistance();
  
    switch (currentState) {
      case STATE_FORWARD:
        if (front < SAFE_DISTANCE) {
          stop();
          currentState = STATE_WAIT;
          stateStartTime = currentTime;
        } else {
          forward();
        }
        break;
      case STATE_WAIT:
        if (currentTime - stateStartTime >= 200) {
          if (left > right && left > SAFE_DISTANCE) {
            currentState = STATE_TURN_LEFT;
          } else if (right > SAFE_DISTANCE) {
            currentState = STATE_TURN_RIGHT;
          } else {
            currentState = STATE_BACKWARD;
          }
          stateStartTime = currentTime;
        }
        break;
      case STATE_TURN_LEFT:
        if (currentTime - stateStartTime == 0) {
          leftForward();
        }
        if (currentTime - stateStartTime >= 400) {
          servo.center();
          currentState = STATE_FORWARD;
        }
        break;
      case STATE_TURN_RIGHT:
        if (currentTime - stateStartTime == 0) {
          rightForward();
        }
        if (currentTime - stateStartTime >= 400) {
          servo.center();
          currentState = STATE_FORWARD;
        }
        break;
      case STATE_BACKWARD:
        if (currentTime - stateStartTime < 600) {
          back();
        } else if (currentTime - stateStartTime < 1000) {
          stop();
          rightForward();
        } else if (currentTime - stateStartTime >= 1400) {
          servo.center();
          currentState = STATE_FORWARD;
        }
        break;
    }
  }
};

/* ------------------------------------------------------------------
   전역 객체 생성 및 초기화
------------------------------------------------------------------- */
SoftwareSerial btSerial(TXD, RXD);  // 블루투스 통신용 SoftwareSerial
RC_Car car(&btSerial);              // RC_Car 객체 (구성요소 모두 포함)

void setup() {
  Serial.begin(9600);       // 디버깅용 시리얼 모니터
  btSerial.begin(9600);
  Wire.begin();
  pcf.begin();
  
  car.begin();              // 각 구성요소 초기화
}

void loop() {
  car.update();             // RC카 동작 업데이트
}
